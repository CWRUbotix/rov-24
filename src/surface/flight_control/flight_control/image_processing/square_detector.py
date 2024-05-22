from math import sqrt
from typing import Any, TypeGuard

import cv2
import matplotlib.pyplot as plt
import numpy as np
from cv2.typing import MatLike
from numpy.typing import NDArray
from PIL import Image, ImageDraw, ImageFont
# from scipy.ndimage.morphology import binary_dilation, binary_erosion
# This version has some types :)
from scipy.ndimage import binary_dilation, binary_erosion
from scipy.signal import convolve2d

EDGE_VALUES_THRESHOLD = 500
BORDER_CLOSING_RADIUS = 3

BORDER_ADJ_MAX_DIST = 10
BORDER_ADJ_CLOSING_RADIUS = 2
BORDER_ADJ_STRELM_WIDTH = 3

BREAK_REGION_WIDTH = 500
BREAK_REGION_HEIGHT = 500
BREAK_REGION_AREA_PERCENT = 0.3

SHAPE_SCORE_WEIGHT = 10

CIRCLE_INNER_RADIUS = 10
CIRCLE_OUTER_RADIUS = 15
LABEL_FONT_SIZE = 30
LINE_WIDTH = 1

SOBEL_X = np.array([[-1, 0, 1],
                    [-2, 0, 2],
                    [-1, 0, 1]])

SOBEL_Y = np.array([[-1, -2, -1],
                    [0, 0, 0],
                    [1, 2, 1]])

# Structuring element for morphological operations
STREL_Y, STREL_X = np.ogrid[
    -BORDER_CLOSING_RADIUS:BORDER_CLOSING_RADIUS + 1,
    -BORDER_CLOSING_RADIUS:BORDER_CLOSING_RADIUS + 1
]
STREL = (STREL_X ** 2 + STREL_Y ** 2) <= BORDER_CLOSING_RADIUS ** 2

CORNER_ADJUST_STREL = np.ones((BORDER_ADJ_STRELM_WIDTH, BORDER_ADJ_STRELM_WIDTH), dtype=bool)


# Should be row, col
Coordinate = tuple[int, int]
Dimensions = tuple[int, int]


def is_uint8(numpy_array: NDArray[np.generic]) -> TypeGuard[NDArray[np.uint8]]:
    """Typeguard the provided Numpy array to make sure it contains uint8s.

    Parameters
    ----------
    numpy_array : NDArray[np.generic]
        The array to guard

    Returns
    -------
    TypeGuard[NDArray[np.uint8]]
        A guarantee that the provided array contains uint8s
    """
    return numpy_array.dtype.type == np.uint8


def area_of_triangle(corner1: Coordinate, corner2: Coordinate,
                     corner3: Coordinate) -> float:
    """Get the area of a triangle formed by the provided vertices. Order does not matter.

    Parameters
    ----------
    corner1 : Coordinate
        First vertex
    corner2 : Coordinate
        Second vertex
    corner3 : Coordinate
        Third vertex

    Returns
    -------
    float
        The area of the triangle
    """
    return 0.5 * abs(
        (
            corner1[1] * corner2[0] +
            corner2[1] * corner3[0] +
            corner3[1] * corner1[0]
        ) -
        (
            corner2[1] * corner1[0] +
            corner3[1] * corner2[0] +
            corner1[1] * corner3[0]
        )
    )


def get_four_neighborhood(coord: Coordinate) -> list[Coordinate]:
    """Get the four neighborhood (orthogonal adjacencies) of the provided point.

    Parameters
    ----------
    coord : Coordinate
        The coordinates of the point to get the neighborhood around

    Returns
    -------
    list[Coordinate]
        A list of a the neighbors
    """
    return [
        (coord[0] + 1, coord[1]),
        (coord[0] - 1, coord[1]),
        (coord[0], coord[1] + 1),
        (coord[0], coord[1] - 1)
    ]


def get_coords_in_box(img_dims: Dimensions, top_left: Coordinate,

                      bottom_right: Coordinate) -> set[Coordinate]:
    """Return the set of coordinates bounded by the given corners and img_dims

    Parameters
    ----------
    img_dims : Dimensions
        Dimensions of the image, coords outside these are discarded
    top_left : Coordinate
        Top left corner of the bounding box
    bottom_right : Coordinate
        Bottom right corner of the bounding box

    Returns
    -------
    set[Coordinate]
        The set of coordinates in the bounding box
    """
    pixels_set: set[Coordinate] = set()
    for col in range(top_left[1], bottom_right[1] + 1):
        for row in range(top_left[0], bottom_right[0] + 2):
            if 0 <= row < img_dims[0] and 0 <= col < img_dims[1]:
                # img[y][x] = color
                pixels_set.add((row, col))
    return pixels_set


def get_threshold_mask(high_contrast_img: MatLike) -> NDArray[Any]:
    """Get a mask with target color thresholding by ((r/g > 3) and (r/b > 3))

    Parameters
    ----------
    high_contrast_img : MatLike
        A contrast enhanced image to threshold

    Returns
    -------
    Matlike
        The resulting mask
    """
    # TODO might break things
    if is_uint8(high_contrast_img):
        # reveal_type(np.dtype(np.uint8))
        red_green_ratio = high_contrast_img[:, :, 0] / (high_contrast_img[:, :, 1] + 1e-8)
        red_blue_ratio = high_contrast_img[:, :, 0] / (high_contrast_img[:, :, 2] + 1e-8)
    else:
        raise ValueError("Somehow your image is not uint8")

    mask = np.logical_and(red_green_ratio > 3.0, red_blue_ratio > 3.0)
    return mask


class Island:
    def __init__(self, do_debug_logs: bool = False) -> None:
        self.pixels_set: set[Coordinate] = set()
        self.bounding_box: tuple[Coordinate, Coordinate]
        self.do_debug_logs = do_debug_logs

        self.considered = False
        self.validated = False
        self.is_target = False
        self.is_leak_disqualified = False
        self.is_frame_edge_disqualified = False
        self.center_pos: tuple[float, float] = (0, 0)
        self.border_extended_pixels: set[Coordinate] = set()
        self.border_failed_stack: set[Coordinate] = set()

        self.min_row: int
        self.max_row: int
        self.min_col: int
        self.max_col: int

        self.val_border_pixels_set: set[Coordinate] = set()
        self.val_inner_err_pixels_set: set[Coordinate] = set()

        # TODO No one uses this
        self.val_outer_err_pixels_set: set[Coordinate] = set()

        # TODO if possible refactor to tuple[Coordinate, Coordinate, Coordinate, Coordinate]
        self.corners: list[Coordinate] = []
        self.error_percent: float

        self.order_number: int
        self.is_fallback_case: bool
        self.region_mask: NDArray[np.bool_]
        self.target_score: float
        self.sort_score: float

    def debug_log(self, msg):
        if self.do_debug_logs:
            print(msg)

    def update_min_max(self, coord: Coordinate) -> None:
        """Update this island's min/max row/col given the provided coords are in the island.

        Parameters
        ----------
        coord : Coordinate
            The coordinate point to incorporate into the island
        """
        self.min_row = min(coord[0], self.min_row)
        self.max_row = max(coord[0], self.max_row)
        self.min_col = min(coord[1], self.min_col)
        self.max_col = max(coord[1], self.max_col)

    def calc_bounding_box(self) -> None:
        """Set the min/max row/col & bounding_box of this island based on its pixels_set."""
        self.min_row = min(row for row, _ in self.pixels_set)
        self.max_row = max(row for row, _ in self.pixels_set)
        self.min_col = min(col for _, col in self.pixels_set)
        self.max_col = max(col for _, col in self.pixels_set)

        self.bounding_box = (
            (self.min_row, self.min_col),
            (self.max_row, self.max_col)
        )

    def calc_target_score(self, img_dims: Dimensions) -> None:
        """Calculate the target score for this island.

        Parameters
        ----------
        img_dims : Dimensions
            The dimensions of the image this island is in
        """
        # Get center of this island in normalized [0,1] space
        # (where 0 and 1 are opposite edges of the image)
        region_pos = (
            (self.bounding_box[0][0] + self.bounding_box[1][0]) / 2 / img_dims[0],
            (self.bounding_box[0][1] + self.bounding_box[1][1]) / 2 / img_dims[1]
        )
        region_dist_from_center = (region_pos[0] - 0.5, region_pos[1] - 0.5)

        center_score = -1 * sqrt(
            np.square(region_dist_from_center[0]) + np.square(region_dist_from_center[1]))
        shape_score = -1 * self.error_percent * SHAPE_SCORE_WEIGHT

        self.target_score = center_score + shape_score

        self.debug_log(f"region target score: {region_dist_from_center} {center_score} vs {self.error_percent} {shape_score} = {self.target_score}")

    def calc_corners(self, img_dims: Dimensions,
                     border_leak_rollback: bool = False) -> None:
        """Find this corners of this island.

        Parameters
        ----------
        img_dims : Dimensions
            The dimensions of the image this island is in
        border_leak_rollback : bool, optional
            Whether to be more aggressive in preventing border leaks, by default False
        """
        corner_candidates: list[set[Coordinate]] = [set(), set(), set(), set()]

        for (row, col) in self.pixels_set:
            if self.min_row == row:  # top corner
                corner_candidates[0].add((row, col))
            if self.max_row == row:  # bottom corner
                corner_candidates[1].add((row, col))
            if self.min_col == col:  # left corner
                corner_candidates[2].add((row, col))
            if self.max_col == col:  # right corner
                corner_candidates[3].add((row, col))

        flat_corner_max_len = min(
            (self.max_row - self.min_row) / 3,
            (self.max_col - self.min_col) / 3
        )

        # each index corresponds to a known side for shape validation.
        new_corners: list[Coordinate] = []

        self.is_fallback_case = False
        for i in range(0, 4):
            corner_candidates_for_edge = corner_candidates[i]
            row_min = min(row for row, _ in corner_candidates_for_edge)
            col_min = min(col for _, col in corner_candidates_for_edge)
            row_max = max(row for row, _ in corner_candidates_for_edge)
            col_max = max(col for _, col in corner_candidates_for_edge)

            # The min/max for one of the dimensions (row or col) should be the same
            candidate_diff = (row_max - row_min) + (col_max - col_min)
            if candidate_diff <= flat_corner_max_len:
                # This is a real corner; set it to average of candidate positions
                new_corners.append((
                    int((row_max + row_min) / 2),
                    int((col_max + col_min) / 2)
                ))
            else:
                # This is a straight/axis-aligned square edge case
                if row_min == 0 or col_min == 0 or \
                   row_max == img_dims[0] - 1 or col_max == img_dims[1] - 1:
                    self.debug_log("At edge of frame - not counting as target.")
                    self.is_frame_edge_disqualified = True
                else:
                    self.is_fallback_case = True
                    break

        # If corners are too close together do fallback corner detection
        min_corner_interdistance = min(
            (self.max_row-self.min_row) / 3,
            (self.max_col-self.min_col) / 3
        )
        for (row_i, col_i) in new_corners:
            for (row_j, col_j) in new_corners:
                dist = np.sqrt(np.square(row_i - row_j) + np.square(col_i - col_j))
                if dist < min_corner_interdistance:
                    self.is_fallback_case = True

        # Fallback corner detection: use OpenCV
        if self.is_fallback_case:
            unsorted_corners = []
            region_mask = np.uint8(self.region_mask) * 255
            contours, _ = cv2.findContours(region_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                # Get corners through approximated contour:
                epsilon = 0.05 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # Check if the contour has 4 corners:
                if len(approx) == 4:
                    # Get corner Coordinate of the square
                    corners = [tuple(approx[i][0]) for i in range(4)]
                    self.debug_log(f"Corner Points: {corners}")
                    for corner in corners:
                        unsorted_corners.append((corner[1], corner[0]))

            # Re-order corners for region validation
            if len(unsorted_corners) == 4:
                new_corners = []

                closest_pixel = list(unsorted_corners)[0]
                closest_dist = float('inf')
                for (row, col) in unsorted_corners:
                    dist = np.abs(row - self.min_row)
                    if dist < closest_dist or (dist == closest_dist and col > closest_pixel[1]):
                        closest_dist = dist
                        closest_pixel = (row, col)
                new_corners.append(closest_pixel)
                unsorted_corners.remove(closest_pixel)

                closest_pixel = list(unsorted_corners)[0]
                closest_dist = float('inf')
                for (row, col) in unsorted_corners:
                    dist = np.abs(row - self.max_row)
                    if dist < closest_dist or (dist == closest_dist and col < closest_pixel[1]):
                        closest_dist = dist
                        closest_pixel = (row, col)
                new_corners.append(closest_pixel)
                unsorted_corners.remove(closest_pixel)

                closest_pixel = list(unsorted_corners)[0]
                closest_dist = float('inf')
                for (row, col) in unsorted_corners:
                    dist = np.abs(col - self.min_col)
                    if dist < closest_dist or (dist == closest_dist and row < closest_pixel[0]):
                        closest_dist = dist
                        closest_pixel = (row, col)
                new_corners.append(closest_pixel)
                unsorted_corners.remove(closest_pixel)

                closest_pixel = list(unsorted_corners)[0]
                closest_dist = float('inf')
                for (row, col) in unsorted_corners:
                    dist = np.abs(col - self.max_col)
                    if dist < closest_dist or (dist == closest_dist and row > closest_pixel[0]):
                        closest_dist = dist
                        closest_pixel = (row, col)
                new_corners.append(closest_pixel)
                unsorted_corners.remove(closest_pixel)

        apply_corner_adjustment = True
        if border_leak_rollback:
            for i in range(0, 4):
                if len(new_corners) != 4 or len(self.corners) != 4:
                    apply_corner_adjustment = False
                    break
                corner = new_corners[i]
                old_corner = self.corners[i]
                dist_from_old_corner = np.sqrt(
                    np.square(corner[0] - old_corner[0]) + np.square(corner[1] - old_corner[1]))
                should_reset = dist_from_old_corner > BORDER_ADJ_MAX_DIST
                self.debug_log(f"CORNER COMPARE: {corner} {old_corner} {dist_from_old_corner} {should_reset}")
                if should_reset:
                    apply_corner_adjustment = False
                    self.debug_log("! adjusted corner dist too far, rolling back.")

        if apply_corner_adjustment:
            self.corners = new_corners

        self.debug_log(f"Corners: {self.corners}")

    def validate_shape_is_target_square(self, img_dims: Dimensions,
                                        debug_shape_image: NDArray[np.generic] | None = None
                                        ) -> None:
        """Check that this island looks like a target square.

        Parameters
        ----------
        img_dims : Dimensions
            The dimensions of the image containing this island
        debug_shape_image : NDArray[np.generic] | None, optional
            If you're making debug images, pass one here to highlight borders, by default None
        """
        if len(self.corners) == 0:
            self.debug_log("Island had no corners, skipping")
            return
        self.considered = True

        sum_pos = (0, 0)
        for corner in self.corners:
            sum_pos = (sum_pos[0] + corner[0], sum_pos[1] + corner[1])
        self.center_pos = (
            sum_pos[0] / len(self.corners),
            sum_pos[1] / len(self.corners)
        )

        if len(self.corners) != 4:
            return

        top_corner = self.corners[0]
        bottom_corner = self.corners[1]
        left_corner = self.corners[2]
        right_corner = self.corners[3]

        # Triangle areas via Shoelace formula
        left_triangle_area = area_of_triangle(top_corner, left_corner, bottom_corner)
        right_triangle_area = area_of_triangle(top_corner, right_corner, bottom_corner)

        expected_area = left_triangle_area + right_triangle_area

        expected_area_2 = 0
        inside_error_pixels = set()
        outside_error_pixels = set()

        self.val_border_pixels_set = set()
        self.val_inner_err_pixels_set = set()
        self.val_outer_err_pixels_set = set()

        def validate_square_edge(from_point: Coordinate, to_point: Coordinate,
                                 is_inside_upwards: bool) -> None:
            nonlocal expected_area_2
            nonlocal inside_error_pixels
            nonlocal outside_error_pixels

            debug_inside_errors_count = 0
            debug_outside_errors_count = 0

            y_intercept = from_point[0]

            denom: int | float = from_point[1] - to_point[1]
            if denom == 0:
                denom = 0.00001
            slope = (from_point[0] - to_point[0]) / denom
            # self.debug_log("validate edge - from", from_point, "to", to_point, "slope:", slope)
            for x in range(from_point[1], to_point[1] + 1):
                x_relative = x - from_point[1]
                slope_expected_y = int(y_intercept + x_relative * slope)
                # self.debug_log("row: ", slope_expected_y, ", col: ", x)
                self.val_border_pixels_set.update(get_coords_in_box(
                    img_dims,
                    (slope_expected_y - LINE_WIDTH, x - LINE_WIDTH),
                    (slope_expected_y + LINE_WIDTH, x + LINE_WIDTH)
                ))

                min_y = min(from_point[0], to_point[0])
                max_y = max(from_point[0], to_point[0])

                for y in range(min_y, max_y):
                    is_shape_pixel = (y, x) in self.pixels_set
                    if (is_inside_upwards and y <= slope_expected_y) or \
                       (not is_inside_upwards and y >= slope_expected_y):
                        expected_area_2 += 1
                        if not is_shape_pixel:
                            debug_inside_errors_count += 1
                            inside_error_pixels.add((y, x))
                    else:
                        if is_shape_pixel:
                            debug_outside_errors_count += 1
                            outside_error_pixels.add((y, x))

            error_percent = (debug_inside_errors_count +
                             debug_outside_errors_count) / expected_area
            self.debug_log(f"error %: {error_percent} {debug_inside_errors_count} {debug_outside_errors_count}")

        validate_square_edge(left_corner, bottom_corner, True)
        validate_square_edge(bottom_corner, right_corner, True)
        validate_square_edge(top_corner, right_corner, False)
        validate_square_edge(left_corner, top_corner, False)

        # Along with iterating the pixels along the edge boxes,
        #  there is also an internal box, between all of the edge boxes,
        #  that needs to be covered:
        #  (Note that in the AABB edge case, this performs all error checks:)
        # self.debug_log("corners:", island.corners)
        # self.debug_log("ranges:", bottom_corner[1], top_corner[1], left_corner[0], right_corner[0])
        for x in range(bottom_corner[1], top_corner[1]):
            for y in range(left_corner[0], right_corner[0]):
                is_shape_pixel = (y, x) in self.pixels_set
                if not is_shape_pixel:
                    inside_error_pixels.add((y, x))

        for inside_error_pixel in inside_error_pixels:
            (y, x) = inside_error_pixel
            self.val_inner_err_pixels_set.update(
                get_coords_in_box(img_dims, (y, x), (y, x)))

        for outside_error_pixel in outside_error_pixels:
            (y, x) = outside_error_pixel
            self.val_outer_err_pixels_set.update(
                get_coords_in_box(img_dims, (y, x), (y, x)))

        self.error_percent = (
            len(inside_error_pixels) + len(outside_error_pixels)) / expected_area
        self.debug_log(f"total error %: {self.error_percent} {expected_area} {expected_area_2}")

        if self.error_percent < 0.30:
            self.validated = True

            # Draw edges and error pixels if debugging
            if debug_shape_image is not None:
                for (y, x) in self.val_border_pixels_set:
                    debug_shape_image[y][x] = [0, 255, 0]
                # for (y, x) in island.val_inner_err_pixels_set:
                #     debug_shape_image[y][x] = [255, 100, 0]
                # for (y, x) in island.val_outer_err_pixels_set:
                #     debug_shape_image[y][x] = [255, 255, 0]


def get_target_color_shapes_debug_image(size_template: MatLike, islands: list[Island],
                                        root_pixels_debug_image: MatLike) -> MatLike:
    """Generate a debugging image that highlights the anatomy of islands.

    Parameters
    ----------
    size_template : MatLike
        An image with the desired debug image size
    islands : list[Island]
        The islands to highlight
    root_pixels_debug_image : MatLike
        A mask converted to RGB by setting True pixels to red and False pixels to black

    Returns
    -------
    MatLike
        An image highlighting islands for debugging
    """
    connected_shape_img = np.zeros_like(size_template)
    for island in islands:
        for (row, col) in island.pixels_set:
            if root_pixels_debug_image[row][col][0] == 255:
                connected_shape_img[row][col] = [150, 0, 0]
            elif (row, col) in island.border_extended_pixels:
                connected_shape_img[row][col] = [255, 150, 150]
            else:
                connected_shape_img[row][col] = [255, 0, 0]
    return connected_shape_img


def get_final_corners_overlay_debug_img(islands: list[Island], base_img: MatLike) -> MatLike:
    """Generate a debugging image that circles the corners of islands

    Parameters
    ----------
    islands : list[Island]
        The islands found for this image
    base_img : MatLike
        The image to annotate

    Returns
    -------
    MatLike
        The resulting annotated image
    """

    img = np.copy(base_img)
    for island in islands:
        if not island.considered:
            continue

        color = [255, 0, 0]
        if island.validated:
            color = [0, 255, 0 if island.is_target else 255]
        elif island.is_frame_edge_disqualified:
            color = [255, 255, 0]
        else:
            color = [255, 0, 0]

        for corner in island.corners:
            y, x = np.ogrid[
                -corner[0]:img.shape[0] - corner[0],
                -corner[1]:img.shape[1] - corner[1]
            ]
            mask = np.logical_and(
                x * x + y * y >= CIRCLE_INNER_RADIUS ** 2,
                x * x + y * y <= CIRCLE_OUTER_RADIUS ** 2
            )

            img[mask] = color

        image_pil = Image.fromarray(img)

        draw = ImageDraw.Draw(image_pil)

        font = ImageFont.load_default()

        text = str(island.order_number)
        # (x, y) Coordinate
        position = (island.center_pos[1], island.center_pos[0])

        _, _, text_width, text_height = draw.textbbox((0, 0), text, font=font)
        position = (position[0] - text_width // 2,
                    position[1] - text_height // 2)

        draw.text(position, text, font=font, fill=(color[0], color[1], color[2]))

        img = np.array(image_pil)

    return img


class SquareDetector:
    def __init__(self, do_debug_logs: bool = False) -> None:
        self.do_debug_logs = do_debug_logs

        self.rows: int
        self.cols: int
        self.img_dims: Dimensions
        self.eroded_border: NDArray[np.float64] | NDArray[Any] | Any

    def debug_log(self, msg):
        if self.do_debug_logs:
            print(msg)

    def dfs(self, coord: Coordinate, visited_map: dict[Coordinate, Island],
            merges: set[tuple[Island, Island]], high_contrast_img: MatLike,
            target_color_mask: MatLike, edge_image: MatLike) -> Island:
        """Perform a depth-first search starting at the provided (r, c) to fill island borders.

        Parameters
        ----------
        r : int
            _description_
        c : int
            _description_
        visited_map : dict[Coordinate, Island]
            A map of which island covers the given coordinate (should start out empty)
        merges : set[tuple[Island, Island]]
            _description_
        high_contrast_img : MatLike
            _description_
        target_color_mask : MatLike
            _description_
        edge_image : MatLike
            _description_

        Returns
        -------
        Island
            The island created by filling the edges around the provided coords
        """

        total_pixels_count = self.rows * self.cols

        # First find root intensity islands:
        root_stack = [coord]
        extended_stack = []  # Stack for the non root pixels.
        island = Island(self.do_debug_logs)

        while root_stack:
            row, col = root_stack.pop()
            is_not_visited = (row, col) not in visited_map
            is_in_bounds = (0 <= row < self.rows and 0 <= col < self.cols)
            if is_not_visited and is_in_bounds:
                is_root_pixel = target_color_mask[row][col]
                is_edge_pixel = edge_image[row][col] is True

                # Adding `not is_edge_pixel` here fixed a leak where
                # region filling was rarely blowing past part of the border because
                # those edge pixels were also root pixels:
                if is_root_pixel and not is_edge_pixel:
                    visited_map[(row, col)] = island
                    island.pixels_set.add((row, col))
                    root_stack.extend(get_four_neighborhood((row, col)))
                else:
                    extended_stack.extend(get_four_neighborhood((row, col)))

        island.min_row = self.img_dims[0]
        island.max_row = 0
        island.min_col = self.img_dims[1]
        island.max_col = 0

        island.border_failed_stack = set()
        # Then extend the root intensities to the full islands based on connected pixels:
        while extended_stack:
            row, col = extended_stack.pop()
            is_not_visited = (row, col) not in visited_map
            is_in_bounds = (0 <= row < self.rows and 0 <= col < self.cols)
            if is_not_visited and is_in_bounds:  # If valid pixel coordinate:
                # If a leak is detected, then just stop the search to save performance.
                region_height = island.max_row - island.min_row
                region_width = island.max_col - island.min_col
                if region_width > BREAK_REGION_WIDTH or \
                   region_height > BREAK_REGION_HEIGHT or \
                   len(island.pixels_set) > total_pixels_count * BREAK_REGION_AREA_PERCENT:
                    self.debug_log("\nStopped spreading artificially - leak detected, so" +
                          "stopped spreading connected pixels.")
                    island.is_leak_disqualified = True
                    break

                is_root_pixel = target_color_mask[row][col]

                raw_pixel = high_contrast_img[row][col]
                is_edge_pixel = edge_image[row][col] is True

                anti_leak_check = (
                    (raw_pixel[1] < raw_pixel[0] or raw_pixel[1] < raw_pixel[2]) and
                    not (raw_pixel[1] > 200 and raw_pixel[2] > 200)
                )
                is_connected_pixel = not is_edge_pixel and anti_leak_check

                # If this pixel belongs to  the island, then add it:
                if is_connected_pixel:
                    visited_map[(row, col)] = island
                    island.pixels_set.add((row, col))
                    island.update_min_max((row, col))
                    extended_stack.extend(get_four_neighborhood((row, col)))
                else:
                    island.border_failed_stack.update(get_four_neighborhood((row, col)))
            elif (row, col) in visited_map and visited_map[(row, col)] != island:
                merges.add((island, visited_map[(row, col)]))

        return island

    def find_islands(self, high_contrast_img: MatLike, target_color_mask: MatLike,
                     edge_image: MatLike) -> tuple[list[Island], dict[Coordinate, Island]]:
        """Get the islands and visited_map (which island is at this coord?) for an image.

        Parameters
        ----------
        high_contrast_img : MatLike
            _description_
        target_color_mask : MatLike
            _description_
        edge_image : MatLike
            _description_

        Returns
        -------
        tuple[list[Island], dict[Coordinate, Island]]
            The list of islands and the visited_map from coordinates to their islands
        """
        islands: list[Island] = []
        visited_map: dict[Coordinate, Island] = {}
        merges: set[tuple[Island, Island]] = set()
        total_pixels_count = self.rows * self.cols

        island_min_pixels_count = total_pixels_count * 0.0025

        # Find island searches for all (unvisited thus far) root pixels:
        for r in range(self.rows):
            for c in range(self.cols):
                # If correct color and not on an edge and not visited
                if target_color_mask[r][c] and not edge_image[r][c] and (r, c) not in visited_map:
                    new_island = self.dfs((r, c), visited_map, merges,
                                          high_contrast_img, target_color_mask, edge_image)
                    if len(new_island.pixels_set) >= island_min_pixels_count:
                        islands.append(new_island)

        # Merge islands that overlapped with each other:
        #  (merges is a set, so there's no duplicate items.)
        for (island_a, island_b) in merges:
            if island_a in islands and island_b in islands:
                self.debug_log(f"merged: {len(island_a.pixels_set)} {len(island_b.pixels_set)}")
                island_a.pixels_set.update(island_b.pixels_set)
                island_a.border_failed_stack.update(island_b.border_failed_stack)
                islands.remove(island_b)

        return (islands, visited_map)

    def calc_better_region(self, island: Island, visited_map: dict[Coordinate, Island]) -> None:
        """Expand this island's region to get a better estimate of corners.

        Parameters
        ----------
        island : Island
            The island to expand
        visited_map : dict[Coordinate, Island]
            This image's visited_map
        """
        region_plus_eroded_border = np.copy(self.eroded_border)
        for (row, col) in island.pixels_set:
            region_plus_eroded_border[row, col] = True

        dilated_image = binary_dilation(region_plus_eroded_border, structure=STREL)
        closed_image = binary_erosion(dilated_image, structure=STREL)

        border_expand_stack = list(island.border_failed_stack)
        while border_expand_stack:
            entry = border_expand_stack.pop()
            island.border_failed_stack.discard(entry)
            is_not_visited = entry not in visited_map
            row, col = entry
            is_in_bounds = (0 <= row < self.rows and 0 <= col < self.cols)
            # If valid pixel coordinate:
            if is_not_visited and is_in_bounds:
                is_in_border = closed_image[row][col]
                if is_in_border:
                    visited_map[(row, col)] = island
                    island.pixels_set.add((row, col))
                    island.border_extended_pixels.add((row, col))
                    border_expand_stack.extend(get_four_neighborhood((row, col)))

    def process_image(self, original_img: MatLike, make_debug_imgs: bool = False,
                      final_image_is_collage: bool = False, show_debug_imgs: bool = False
                      ) -> tuple[list[Coordinate] | None, MatLike | None]:
        """_summary_

        Parameters
        ----------
        original_img : MatLike
            OpenCV image IN RGB FORMAT!!!!
        do_collage : bool
            _description_
        show_debug_imgs : bool, optional
            _description_, by default True

        Returns
        -------
        _type_
            _description_
        """

        # TODO: Replace with enum?
        if (show_debug_imgs or final_image_is_collage) and not make_debug_imgs:
            raise Exception("Error: can only show debug images or make collage if make_debug_imgs is true")

        self.rows = len(original_img)
        self.cols = len(original_img[0])

        self.img_dims = (original_img.shape[0], original_img.shape[1])

        bgr_image = cv2.cvtColor(original_img, cv2.COLOR_RGB2BGR)

        # LAB space histogram equalization for better contrast
        lab_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2Lab)
        l_channel, a_channel, b_channel = cv2.split(lab_image)
        equalized_l_channel = cv2.equalizeHist(l_channel)
        equalized_lab_image = cv2.merge((equalized_l_channel, a_channel, b_channel))
        high_contrast_img = cv2.cvtColor(equalized_lab_image, cv2.COLOR_Lab2RGB)

        high_contrast_combined_channels = np.dot(high_contrast_img[..., :3], [1, 1, 1])

        # Sobel edge detection
        edges_x = convolve2d(high_contrast_combined_channels,
                             SOBEL_X, mode='same', boundary='symm')
        edges_y = convolve2d(high_contrast_combined_channels,
                             SOBEL_Y, mode='same', boundary='symm')
        edge_image_raw = np.sqrt(edges_x ** 2 + edges_y ** 2)

        # Edge thresholding
        edge_image_thresholded = edge_image_raw > EDGE_VALUES_THRESHOLD

        # Morphological closing
        dilated_image = binary_dilation(edge_image_thresholded, structure=STREL)
        closed_image = binary_erosion(dilated_image, structure=STREL)
        sobel_edges_image = closed_image

        # Compute the eroded border for later use in the corner adjustment step
        self.eroded_border = binary_erosion(closed_image, structure=CORNER_ADJUST_STREL)

        # Threshold by ratios of the target color
        target_color_mask = get_threshold_mask(high_contrast_img)

        # DEBUG: convert mask to RGB for display
        if make_debug_imgs and (final_image_is_collage or show_debug_imgs):
            target_color_mask_rgb_dimg = np.zeros_like(high_contrast_img)
            target_color_mask_rgb_dimg[target_color_mask] = [255, 0, 0]
            target_color_mask_rgb_dimg[~target_color_mask] = [0, 0, 0]

        # Find islands (region filling)
        self.debug_log(f"size: {target_color_mask.shape}")
        islands, visited_map = self.find_islands(
            high_contrast_img, target_color_mask,
            sobel_edges_image
        )

        # Find island corners
        self.debug_log(f"{len(islands)} islands:")
        for island in islands:
            island.calc_bounding_box()

            island.region_mask = np.zeros(self.img_dims, dtype=bool)
            for pixel in island.pixels_set:
                island.region_mask[pixel] = True

            self.debug_log(f"island bounding box: {island.bounding_box}")
            island.calc_corners(self.img_dims, False)

        debug_shape_image = np.copy(original_img) if make_debug_imgs else None

        # Validate islands are squares
        for island in islands:
            island.validate_shape_is_target_square(self.img_dims, debug_shape_image)
            if island.validated:
                island.calc_target_score(self.img_dims)
                island.sort_score = island.target_score
            else:
                island.sort_score = float('-inf')

        islands = sorted(islands, key=lambda x: x.sort_score, reverse=True)
        for i, island in enumerate(islands):
            island.order_number = i + 1

            if i == 0 and island.validated:  # Best scoring island
                island.is_target = True
                # Calc better region+corners for the target region: DISABLED FOR PERFORMANCE
                # self.calc_better_region(island, visited_map)
                # island.calc_bounding_box()
                # island.calc_corners(self.img_dims, True)

            self.debug_log(f"sorted island: {island.order_number} {island.sort_score}")

        final_image_output: MatLike | None = None
        if make_debug_imgs:
            if final_image_is_collage or show_debug_imgs:
                edge_rgb = np.zeros((*sobel_edges_image.shape, 3), dtype=np.uint8)
                edge_rgb[sobel_edges_image == 1] = [255, 255, 255]
                edge_annotated_img = get_final_corners_overlay_debug_img(islands, edge_rgb)

                target_color_shapes_debug_image = get_target_color_shapes_debug_image(
                        high_contrast_img, islands, target_color_mask_rgb_dimg)
                target_color_shapes_annotated_img = get_final_corners_overlay_debug_img(
                        islands, target_color_shapes_debug_image)

            if debug_shape_image is not None:
                final_annotated_img = get_final_corners_overlay_debug_img(
                        islands, debug_shape_image)
            else:
                raise Exception("Debug shape image was None, but make_debug_imgs is true!")

            if show_debug_imgs:
                output_img_1 = original_img

                # See edge overlayed in blue on final image to see what's wrong:
                # mask = edge_image == True
                # output_img_6[mask] = [0, 0, 255]
                # output_img_4 = mask

                # Debug edge images:
                # output_img_3 = edge_image_input
                # output_img_4 = edge_image_raw
                # output_img_5 = edge_image_thresholded
                # output_img_6 = edge_image

                output_img_2 = high_contrast_img
                # output_img_3 = root_pixels_debug_image
                output_img_3 = edge_rgb
                output_img_4 = target_color_shapes_debug_image
                output_img_5 = target_color_shapes_annotated_img
                output_img_6 = final_annotated_img

                # Display the original and stretched images side by side
                plt.figure(figsize=(12, 8))  # fig size is the size of the window.

                figure_rows = 2
                figure_cols = 3

                # Plot original image
                plt.subplot(figure_rows, figure_cols, 1)
                plt.imshow(output_img_1)
                plt.title('Original Image')
                plt.axis('off')

                # Plot stretched image
                plt.subplot(figure_rows, figure_cols, 2)
                plt.imshow(output_img_2)
                plt.title('Img 2')
                plt.axis('off')

                # Plot stretched image
                plt.subplot(figure_rows, figure_cols, 3)
                plt.imshow(output_img_3)
                plt.title('Img 3')
                plt.axis('off')

                # Plot stretched image
                plt.subplot(figure_rows, figure_cols, 4)
                plt.imshow(output_img_4)
                plt.title('Img 4')
                plt.axis('off')

                plt.subplot(figure_rows, figure_cols, 5)
                plt.imshow(output_img_5)
                plt.title('Img 5')
                plt.axis('off')

                plt.subplot(figure_rows, figure_cols, 6)
                plt.imshow(output_img_6)
                plt.title('Img 6')
                plt.axis('off')

            if final_image_is_collage:
                top_row = np.hstack((original_img, final_annotated_img))
                bottom_row = np.hstack(
                    (edge_annotated_img, target_color_shapes_annotated_img))
                collage = np.vstack((top_row, bottom_row))
                final_image_output = collage
            else:
                final_image_output = final_annotated_img

        final_corners_output: list[Coordinate] | None = None
        for island in islands:
            if island.is_target:
                final_corners_output = island.corners

        return (final_corners_output, final_image_output)
