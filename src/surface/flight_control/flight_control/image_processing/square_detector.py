from PIL import Image, ImageDraw, ImageFont
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import convolve2d
from scipy.ndimage.morphology import binary_dilation, binary_erosion
import cv2
from cv2.typing import MatLike

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

# Structuring elements for morphological operations
CLOSING_STREL_Y, CLOSING_STREL_X = np.ogrid[
    -BORDER_CLOSING_RADIUS:BORDER_CLOSING_RADIUS + 1,
    -BORDER_CLOSING_RADIUS:BORDER_CLOSING_RADIUS + 1
]
CLOSING_STREL = (CLOSING_STREL_X ** 2 + CLOSING_STREL_Y ** 2) <= BORDER_CLOSING_RADIUS ** 2

CORNER_ADJUST_STREL = np.ones((BORDER_ADJ_STRELM_WIDTH, BORDER_ADJ_STRELM_WIDTH), dtype=bool)


# Should be row, cow
Coordinate = tuple[int, int]


class Island:
    def __init__(self) -> None:
        self.pixels_set: set[Coordinate] = set()
        self.bounding_box = None
        # If the shape should even be considered shown at all, or if it is todo / shouldn't be shown
        self.considered = False
        self.validated = False
        self.is_target = False
        self.is_leak_disqualified = False
        self.is_frame_edge_disqualified = False
        self.center_pos = (0, 0)
        self.border_extended_pixels: set[Coordinate] = set()
        self.border_failed_stack: set[list[Coordinate]] = set()

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
        self.error_percent = None

    def update_min_max(self, row: int, col: int) -> None:
        self.min_row = min(row, self.min_row)
        self.max_row = max(row, self.max_row)
        self.min_col = min(col, self.min_col)
        self.max_col = max(col, self.max_col)


class SquareDetector:
    def __init__(self):
        self.rows = None
        self.cols = None
        self.img_size = None
        self.visited_map = None
        self.eroded_border = None

    def is_connected_target_shape_pixel(self, rgb: list[int]):
        condition_red = rgb[0] > 30
        condition_other_color = (rgb[1] > 10 or rgb[2] > 10)
        condition = np.logical_and(condition_red, ~condition_other_color)
        return condition

    def get_harsh_target_colors(self, high_contrast_img: MatLike) -> MatLike:
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
        red_green_ratio = high_contrast_img[:, :, 0] / (high_contrast_img[:, :, 1] + 1e-8)
        red_blue_ratio = high_contrast_img[:, :, 0] / (high_contrast_img[:, :, 2] + 1e-8)

        mask = np.logical_and(red_green_ratio > 3.0, red_blue_ratio > 3.0)
        return mask

    def get_target_color_shapes_debug_image(self, size_template, islands, root_pixels_debug_image):
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

    def get_island_debug_image(self, size_template, island):
        connected_shape_img = np.zeros_like(size_template)
        for (row, col) in island.pixels_set:
            connected_shape_img[row][col] = [255, 0, 0]
        return connected_shape_img

    def get_shapes_corners_debug_image(self, islands, root_pixels_debug_image):
        img = np.copy(root_pixels_debug_image)
        for island in islands:
            for corner in island.corners:
                inner_radius = CIRCLE_INNER_RADIUS
                outer_radius = CIRCLE_OUTER_RADIUS
                center_x = corner[1]
                center_y = corner[0]
                y, x = np.ogrid[
                    -center_y:img.shape[0] - center_y,
                    -center_x:img.shape[1] - center_x
                ]
                mask = np.logical_and(
                    x * x + y * y >= inner_radius * inner_radius,
                    x * x + y * y <= outer_radius * outer_radius
                )
                img[mask] = [255, 255, 0]  # Set color to green (RGB)

        return img

    def get_final_corners_overlay_debug_img(self, islands, base_img):
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
                inner_radius = CIRCLE_INNER_RADIUS
                outer_radius = CIRCLE_OUTER_RADIUS
                center_x = corner[1]
                center_y = corner[0]
                y, x = np.ogrid[
                    -center_y:img.shape[0] - center_y,
                    -center_x:img.shape[1] - center_x
                ]
                mask = np.logical_and(
                    x * x + y * y >= inner_radius * inner_radius,
                    x*x + y * y <= outer_radius * outer_radius
                )

                img[mask] = color

            image_pil = Image.fromarray(img)

            draw = ImageDraw.Draw(image_pil)

            font_color = tuple(color)
            font = ImageFont.load_default()

            text = str(island.order_number)
            # (x, y) Coordinate
            position = (island.center_pos[1], island.center_pos[0])

            _, _, text_width, text_height = draw.textbbox((0, 0), text, font=font)
            position = (position[0] - text_width // 2,
                        position[1] - text_height // 2)

            draw.text(position, text, font=font, fill=font_color)

            img = np.array(image_pil)

        # for (row, col) in island.pixels_set:
        #     if (root_pixels_debug_image[row][col][0] == 255):
        #         connected_shape_img[row][col] = [150, 0, 0]
        #     else:
        #         connected_shape_img[row][col] = [255, 0, 0]
        # mask = edge_img == 1
        # img[mask] = [0, 0, 255]
        # img = edge_img

        return img

    def make_extend_list(self, row: int, col: int) -> list[Coordinate]:
        return [
            (row + 1, col),
            (row - 1, col),
            (row, col + 1),
            (row, col - 1)
        ]

    def find_islands(self, high_contrast_img: MatLike,
                     target_color_mask: MatLike, edge_image: MatLike):
        islands = []
        visited_map = {}
        merges: set[list[Island]] = set()
        total_pixels_count = len(target_color_mask) * len(target_color_mask[0])

        def dfs(r, c):
            # First find root intensity islands:
            root_stack = [(r, c)]
            extended_stack = []  # Stack for the non root pixels.
            island = Island()

            while root_stack:
                row, col = root_stack.pop()
                is_not_visited = (row, col) not in visited_map
                is_in_bounds = (0 <= row < self.rows and 0 <= col < self.cols)
                if is_not_visited and is_in_bounds:
                    is_root_pixel = target_color_mask[row][col]

                    is_edge_pixel = edge_image[row][col] is True

                    # Adding `not is_edge_pixel` here fixed a leak where
                    #  region filling was rarely blowing past part of the border because
                    #  those edge pixels were also root pixels:
                    if is_root_pixel and not is_edge_pixel:
                        visited_map[(row, col)] = island
                        island.pixels_set.add((row, col))
                        root_stack.extend(self.make_extend_list(row, col))
                    else:
                        extended_stack.extend(self.make_extend_list(row, col))

            island.min_row = self.img_size[0]
            island.max_row = 0
            island.min_col = self.img_size[1]
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
                        print("\nStopped spreading artificially - leak detected, so" +
                              "stopped spreading connected pixels.")
                        island.is_leak_disqualified = True
                        break

                    is_root_pixel = target_color_mask[row][col]

                    raw_pixel = high_contrast_img[row][col]
                    is_edge_pixel = edge_image[row][col] is True

                    anti_leak_check = (
                        (raw_pixel[1] < raw_pixel[0] or raw_pixel[1] < raw_pixel[2]) and \
                        not (raw_pixel[1] > 200 and raw_pixel[2] > 200)
                    )
                    is_connected_pixel = (not is_edge_pixel) and anti_leak_check

                    # If this pixel belongs to  the island, then add it:
                    if is_connected_pixel:
                        visited_map[(row, col)] = island
                        island.pixels_set.add((row, col))
                        island.update_min_max(row, col)
                        extended_stack.extend(self.make_extend_list(row, col))
                    else:
                        island.border_failed_stack.update(self.make_extend_list(row, col))
                elif (row, col) in visited_map and visited_map[(row, col)] != island:
                    merges.add((island, visited_map[(row, col)]))

            return island

        island_min_pixels_count = total_pixels_count * 0.0025

        # Find island searches for all (unvisited thus far) root pixels:
        for r in range(self.rows):
            for c in range(self.cols):
                # If correct color and not on an edge and not visited
                if target_color_mask[r][c] and not edge_image[r][c] and (r, c) not in visited_map:
                    new_island = dfs(r, c)
                    if len(new_island.pixels_set) >= island_min_pixels_count:
                        islands.append(new_island)

        # Merge islands that overlapped with each other:
        #  (merges is a set, so there's no duplicate items.)
        for (island_a, island_b) in merges:
            if island_a in islands and island_b in islands:
                print("merged: ", len(island_a.pixels_set), len(island_b.pixels_set))
                island_a.pixels_set.update(island_b.pixels_set)
                island_a.border_failed_stack.update(island_b.border_failed_stack)
                islands.remove(island_b)

        self.visited_map = visited_map

        return islands

    def calc_better_region(self, island):
        region_plus_eroded_border = np.copy(self.eroded_border)
        for (y, x) in island.pixels_set:
            region_plus_eroded_border[y, x] = True

        radius = BORDER_ADJ_CLOSING_RADIUS
        y, x = np.ogrid[-radius:radius+1, -radius:radius + 1]
        dilate_struct_element = x ** 2 + y ** 2 <= radius ** 2
        dilated_image = binary_dilation(
            region_plus_eroded_border, structure=dilate_struct_element)

        # erode_struct_element = np.ones((5, 5), dtype=bool)
        radius = BORDER_ADJ_CLOSING_RADIUS
        y, x = np.ogrid[-radius:radius + 1, -radius:radius + 1]
        erode_struct_element = x ** 2 + y ** 2 <= radius ** 2
        closed_image = binary_erosion(
            dilated_image, structure=erode_struct_element)

        border_expand_stack = list(island.border_failed_stack)
        while border_expand_stack:
            entry = border_expand_stack.pop()
            # print("border extend:", entry)
            is_border_failed_stack = island.border_failed_stack.discard(entry)
            row, col = entry
            is_not_visited = (row, col) not in self.visited_map
            is_in_bounds = (0 <= row < self.rows and 0 <= col < self.cols)
            # If valid pixel coordinate:
            if (is_not_visited or is_border_failed_stack) and is_in_bounds:
                is_in_border = closed_image[row][col]
                if is_in_border:
                    self.visited_map[(row, col)] = island
                    island.pixels_set.add((row, col))
                    island.border_extended_pixels.add((row, col))
                    border_expand_stack.extend(self.make_extend_list(row, col))

    def calc_bounding_boxes(self, island):
        relevant_pixels = island.pixels_set
        island.min_row = min(row for row, _ in relevant_pixels)
        island.max_row = max(row for row, _ in relevant_pixels)
        island.min_col = min(col for _, col in relevant_pixels)
        island.max_col = max(col for _, col in relevant_pixels)

        island.bounding_box = (
            (island.min_row, island.min_col),
            (island.max_row, island.max_col)
        )

    def calc_corners(self, island, border_leak_rollback=False):
        corner_candidate_min_maxes = [set(), set(), set(), set()]

        for (row, col) in island.pixels_set:
            if island.min_row == row:  # top corner:
                corner_candidate_min_maxes[0].add((row, col))

            if island.max_row == row:  # bottom corner:
                corner_candidate_min_maxes[1].add((row, col))

            if island.min_col == col:  # left corner:
                corner_candidate_min_maxes[2].add((row, col))

            if island.max_col == col:  # right corner:
                corner_candidate_min_maxes[3].add((row, col))

        flat_corner_max_len = min(
            (island.max_row - island.min_row) / 3,
            (island.max_col - island.min_col) / 3
        )

        # Is an array where each index corresponds to a known side for shape validation.
        new_corners = []

        is_aabb_edge_case = False
        for i in range(0, 4):
            corner_candidates_for_edge = corner_candidate_min_maxes[i]
            row_min = min(row for row, _ in corner_candidates_for_edge)
            col_min = min(col for _, col in corner_candidates_for_edge)
            row_max = max(row for row, _ in corner_candidates_for_edge)
            col_max = max(col for _, col in corner_candidates_for_edge)

            # Keep in mind the min and max for one of the dimensions (row or col) should be the same.
            candidate_diff = (row_max - row_min) + (col_max - col_min)
            if candidate_diff <= flat_corner_max_len:
                # If this is a real corner, then set it to the average of the candidate positions:
                new_corners.append((
                    (int)((row_max + row_min) / 2),
                    (int)((col_max + col_min) / 2)
                ))
            else:
                # If this is a straight/axis-aligned square edge case:
                if row_min == 0 or col_min == 0 or row_max == self.img_size[0]-1 or \
                   col_max == self.img_size[1]-1:
                    print("At edge of frame - not counting as target.")
                    island.is_frame_edge_disqualified = True
                else:
                    is_aabb_edge_case = True
                    break

        # If corners are too close together, then do fallback corner detection:
        min_corner_interdistance = min(
            (island.max_row-island.min_row) / 3,
            (island.max_col-island.min_col) / 3
        )
        for (row_i, col_i) in new_corners:
            for (row_j, col_j) in new_corners:
                dist = np.sqrt(np.square(row_i - row_j) + np.square(col_i - col_j))
                if dist < min_corner_interdistance:
                    is_aabb_edge_case = True

        # Failed attempt at handling the AABB case - was trying to match pixels to the closest AABB corners.
        # if (is_aabb_edge_case):
            # new_corners = []
            # print("AABB edge case!")
            # # Get pixels closest to bounding box corners:
            # bounding_box_corners = [ # order of corners is important for later.
            #     (island.min_row, island.max_col), # top corner
            #     (island.max_row, island.min_col), # bottom corner
            #     (island.min_row, island.min_col), # left corner
            #     (island.max_row, island.max_col)] # right corner

            # for i in range(0,4):
            #     (corner_row, corner_col) = bounding_box_corners[i]
            #     closest_pixel = list(island.pixels_set)[0]
            #     closest_dist = float('inf')
            #     for (row, col) in island.pixels_set:
            #         dist = np.sqrt(np.square(row - corner_row) + np.square(col - corner_col))
            #         if (dist < closest_dist):
            #             closest_dist = dist
            #             closest_pixel = (row, col)
            #     new_corners.append(closest_pixel)

        island.is_fallback_case = is_aabb_edge_case or False
        if island.is_fallback_case:
            unsorted_corners = []
            region_mask = np.uint8(island.region_mask) * 255
            contours, _ = cv2.findContours(
                region_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                # Get corners through approximated contour:
                epsilon = 0.05 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # Check if the contour has 4 corners:
                if len(approx) == 4:
                    # Draw contour for debugging:
                    # cv2.drawContours(self.DEBUG_IMG, [approx], 0, (0, 255, 0), 2)

                    # Get corner Coordinate of the square
                    corners = [tuple(approx[i][0]) for i in range(4)]
                    print("!!!!!!Corner Points:", corners)
                    for corner in corners:
                        unsorted_corners.append((corner[1], corner[0]))
                        # cv2.circle(self.DEBUG_IMG, corner, 5, (0, 255, 0), -1)

            # If we have our corners, we now need to re-order them properly so that region validation happens correctly:
            if len(unsorted_corners) == 4:
                new_corners = []

                closest_pixel = list(unsorted_corners)[0]
                closest_dist = float('inf')
                for (row, col) in unsorted_corners:
                    dist = np.abs(row - island.min_row)
                    if dist < closest_dist or (dist == closest_dist and col > closest_pixel[1]):
                        closest_dist = dist
                        closest_pixel = (row, col)
                new_corners.append(closest_pixel)
                unsorted_corners.remove(closest_pixel)

                closest_pixel = list(unsorted_corners)[0]
                closest_dist = float('inf')
                for (row, col) in unsorted_corners:
                    dist = np.abs(row - island.max_row)
                    if dist < closest_dist or (dist == closest_dist and col < closest_pixel[1]):
                        closest_dist = dist
                        closest_pixel = (row, col)
                new_corners.append(closest_pixel)
                unsorted_corners.remove(closest_pixel)

                closest_pixel = list(unsorted_corners)[0]
                closest_dist = float('inf')
                for (row, col) in unsorted_corners:
                    dist = np.abs(col - island.min_col)
                    if dist < closest_dist or (dist == closest_dist and row < closest_pixel[0]):
                        closest_dist = dist
                        closest_pixel = (row, col)
                new_corners.append(closest_pixel)
                unsorted_corners.remove(closest_pixel)

                closest_pixel = list(unsorted_corners)[0]
                closest_dist = float('inf')
                for (row, col) in unsorted_corners:
                    dist = np.abs(col - island.max_col)
                    if dist < closest_dist or (dist == closest_dist and row > closest_pixel[0]):
                        closest_dist = dist
                        closest_pixel = (row, col)
                new_corners.append(closest_pixel)
                unsorted_corners.remove(closest_pixel)

        apply_corner_adjustment = True
        if border_leak_rollback:
            for i in range(0, 4):
                if len(new_corners) != 4 or len(island.corners) != 4:
                    apply_corner_adjustment = False
                    break
                corner = new_corners[i]
                old_corner = island.corners[i]
                dist_from_old_corner = np.sqrt(
                    np.square(corner[0] - old_corner[0]) + np.square(corner[1] - old_corner[1]))
                should_reset = dist_from_old_corner > BORDER_ADJ_MAX_DIST
                print("CORNER COMPARE:", corner, old_corner,
                      dist_from_old_corner, should_reset)
                if should_reset:
                    apply_corner_adjustment = False
                    print("! adjusted corner dist too far, rolling back.")

        if apply_corner_adjustment:
            island.corners = new_corners

        print("corners: ", island.corners)

    def area_of_triangle(self, corner1, corner2, corner3):
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

    def draw_box(self, pixels_set, color, min_row, max_row, min_col, max_col):
        for x in range(min_col, max_col + 1):
            for y in range(min_row, max_row + 2):
                if 0 <= y and y < self.img_size[0] and 0 <= x and x < self.img_size[1]:
                    # img[y][x] = color
                    pixels_set.add((y, x))

    def validate_shape_is_target_square(self, island: Island, debug_shape_image):
        if len(island.corners) == 0:
            print("island had zero corners, skipping... (TODO functionality?)")
            return
        island.considered = True

        sum_pos = (0, 0)
        for corner in island.corners:
            sum_pos = (sum_pos[0] + corner[0], sum_pos[1] + corner[1])
        island.center_pos = (
            sum_pos[0] / len(island.corners),
            sum_pos[1] / len(island.corners)
        )

        if len(island.corners) != 4:
            return

        top_corner = island.corners[0]
        bottom_corner = island.corners[1]
        left_corner = island.corners[2]
        right_corner = island.corners[3]

        # min_pos = (top_corner[0], left_corner[1])
        # max_pos = (bottom_corner[0], right_corner[1])
        # island.center_pos = ((min_pos[0] + max_pos[0])/2, (min_pos[1] + max_pos[1])/2)

        # Triangle areas via Shoelace formula:
        left_triangle_area = self.area_of_triangle(top_corner, left_corner, bottom_corner)
        right_triangle_area = self.area_of_triangle(bottom_corner, right_corner, top_corner)

        expected_area = left_triangle_area + right_triangle_area

        # island.debug_shape_validation_image = debug_shape_image

        inside_errors_total_count = 0
        outside_errors_total_count = 0

        expected_area_2 = 0

        inside_error_pixels = set()
        outside_error_pixels = set()

        island.val_border_pixels_set = set()
        island.val_inner_err_pixels_set = set()
        island.val_outer_err_pixels_set = set()

        def validate_square_edge(from_point, to_point, is_inside_upwards):
            nonlocal expected_area_2
            nonlocal inside_error_pixels
            nonlocal outside_error_pixels
            nonlocal inside_errors_total_count
            nonlocal outside_errors_total_count

            inside_errors_count = 0
            outside_errors_count = 0

            y_intercept = from_point[0]

            denom = from_point[1] - to_point[1]
            if denom == 0:
                denom = 0.00001
            slope = (from_point[0] - to_point[0]) / denom
            # print("validate edge - from", from_point, "to", to_point, "slope:", slope)
            for x in range(from_point[1], to_point[1] + 1):
                x_relative = x - from_point[1]
                slope_expected_y = (int)(y_intercept + x_relative * slope)
                # print("row: ", slope_expected_y, ", col: ", x)
                self.draw_box(
                    island.val_border_pixels_set,
                    [0, 255, 0],
                    slope_expected_y - LINE_WIDTH,
                    slope_expected_y + LINE_WIDTH,
                    x - LINE_WIDTH,
                    x + LINE_WIDTH
                )

                min_y = min(from_point[0], to_point[0])
                max_y = max(from_point[0], to_point[0])

                for y in range(min_y, max_y):
                    is_shape_pixel = (y, x) in island.pixels_set
                    if (is_inside_upwards and y <= slope_expected_y) or \
                       (not is_inside_upwards and y >= slope_expected_y):
                        expected_area_2 += 1
                        if not is_shape_pixel:
                            inside_errors_count += 1
                            inside_error_pixels.add((y, x))
                    else:
                        if is_shape_pixel:
                            outside_errors_count += 1
                            outside_error_pixels.add((y, x))

            error_percent = (inside_errors_count + outside_errors_count) / expected_area
            print("error %: ", error_percent, inside_errors_count, outside_errors_count)

            inside_errors_total_count += inside_errors_count
            outside_errors_total_count += outside_errors_count

        validate_square_edge(left_corner, bottom_corner, True)
        validate_square_edge(bottom_corner, right_corner, True)
        validate_square_edge(top_corner, right_corner, False)
        validate_square_edge(left_corner, top_corner, False)

        # Along with iterating the pixels along the edge boxes,
        #  there is also an internal box, between all of the edge boxes,
        #  that needs to be covered:
        #  (Note that in the AABB edge case, this performs all error checks:)
        # print("corners:", island.corners)
        # print("ranges:", bottom_corner[1], top_corner[1], left_corner[0], right_corner[0])
        for x in range(bottom_corner[1], top_corner[1]):
            for y in range(left_corner[0], right_corner[0]):
                is_shape_pixel = (y, x) in island.pixels_set
                if not is_shape_pixel:
                    inside_errors_total_count += 1
                    inside_error_pixels.add((y, x))

        for inside_error_pixel in inside_error_pixels:
            (y, x) = inside_error_pixel
            self.draw_box(island.val_inner_err_pixels_set, [255, 100, 0], y, y, x, x)

        for outside_error_pixel in outside_error_pixels:
            (y, x) = outside_error_pixel
            self.draw_box(island.val_outer_err_pixels_set, [255, 255, 0], y, y, x, x)

        island.error_percent = (
            inside_errors_total_count + outside_errors_total_count) / expected_area
        print("total error %: ", island.error_percent,
              expected_area, expected_area_2)

        if (island.error_percent < 0.30):
            island.validated = True

            # Draw edges and error pixels:
            for (y, x) in island.val_border_pixels_set:
                debug_shape_image[y][x] = [0, 255, 0]
            # for (y, x) in island.val_inner_err_pixels_set:
            #     debug_shape_image[y][x] = [255, 100, 0]
            # for (y, x) in island.val_outer_err_pixels_set:
            #     debug_shape_image[y][x] = [255, 255, 0]

    def calculate_region_target_score(self, island):
        # Get the center position of the region in normalized [0,1] space, where 0 and 1 are opposite edges of the image:
        region_pos = (((island.bounding_box[0][0] + island.bounding_box[1][0]) / 2) / self.img_size[0], ((
            island.bounding_box[0][1] + island.bounding_box[1][1]) / 2) / self.img_size[1])
        region_dist_from_center = (region_pos[0] - .5, region_pos[1] - .5)

        center_score = - \
            np.sqrt(
                np.square(region_dist_from_center[0]) + np.square(region_dist_from_center[1]))
        shape_score = (-island.error_percent) * SHAPE_SCORE_WEIGHT

        island.target_score = center_score + shape_score

        print("region target score:", region_dist_from_center, center_score,
              "vs", island.error_percent, shape_score, "=", island.target_score)

    def process_image(self, original_img: MatLike, do_collage: bool, show_debug_imgs: bool = True):
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

        self.rows = len(original_img)
        self.cols = len(original_img[0])

        default_img = np.zeros((1, 1, 3), dtype=np.uint8)
        output_img_2 = default_img
        output_img_3 = default_img
        output_img_4 = default_img
        output_img_5 = default_img
        output_img_6 = default_img
        self.img_size = (original_img.shape[0], original_img.shape[1])

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
        dilated_image = binary_dilation(edge_image_thresholded, structure=CLOSING_STREL)
        closed_image = binary_erosion(dilated_image, structure=CLOSING_STREL)
        sobel_edges_image = closed_image

        # Compute the eroded border for later use in the corner adjustment step
        self.eroded_border = binary_erosion(closed_image, structure=CORNER_ADJUST_STREL)

        # Threshold by ratios of the target color
        target_color_mask = self.get_harsh_target_colors(high_contrast_img)

        # DEBUG: convert mask to RGB for display
        target_color_thresholding_dimg = np.zeros_like(high_contrast_img)
        target_color_thresholding_dimg[target_color_mask] = [255, 0, 0]
        target_color_thresholding_dimg[~target_color_mask] = [0, 0, 0]

        # Find islands (region filling)
        print("size:", target_color_mask.shape)
        islands = self.find_islands(
            high_contrast_img, target_color_mask,
            sobel_edges_image
        )

        print(len(islands), "islands: ")
        for island in islands:
            self.calc_bounding_boxes(island)

            island.region_mask = np.zeros(self.img_size, dtype=bool)
            for pixel in island.pixels_set:
                island.region_mask[pixel] = True

            print("island bounding box: ", island.bounding_box)
            self.calc_corners(island, False)

        debug_shape_image = np.copy(original_img)

        for island in islands:
            self.validate_shape_is_target_square(island, debug_shape_image)
            if island.validated:
                self.calculate_region_target_score(island)
                island.sort_score = island.target_score
            else:
                island.sort_score = float('-inf')

        islands = sorted(islands, key=lambda x: x.sort_score, reverse=True)
        i = 0
        for island in islands:
            i += 1
            island.order_number = i

            is_top_score = island.order_number == 1
            if is_top_score:
                if island.validated:
                    island.is_target = True
                    # Calc better region+corners for the target region:
                    self.calc_better_region(island)
                    self.calc_bounding_boxes(island)
                    self.calc_corners(island, True)

            print("sorted island:", island.order_number, island.sort_score)

        edge_rgb = np.zeros((*sobel_edges_image.shape, 3), dtype=np.uint8)
        edge_rgb[sobel_edges_image == 1] = [255, 255, 255]
        edge_annotated_img = self.get_final_corners_overlay_debug_img(islands, edge_rgb)

        target_color_shapes_debug_image = self.get_target_color_shapes_debug_image(
            high_contrast_img, islands, target_color_thresholding_dimg)
        target_color_shapes_annotated_img = self.get_final_corners_overlay_debug_img(
            islands, target_color_shapes_debug_image)
        final_annotated_img = self.get_final_corners_overlay_debug_img(
            islands, debug_shape_image)

        if show_debug_imgs:
            output_img_1 = original_img

            edge_rgb = np.zeros((*sobel_edges_image.shape, 3), dtype=np.uint8)
            edge_rgb[sobel_edges_image == 1] = [255, 255, 255]

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
            # output_img_4 = self.DEBUG_IMG
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

        final_image_output = None
        # Show the plot
        if do_collage:
            top_row = np.hstack((original_img, final_annotated_img))
            bottom_row = np.hstack(
                (edge_annotated_img, target_color_shapes_annotated_img))
            collage = np.vstack((top_row, bottom_row))
            final_image_output = collage
        else:
            final_image_output = final_annotated_img

        final_corners_output = None
        for island in islands:
            if island.is_target:
                final_corners_output = island.corners

        return (final_corners_output, final_image_output)
