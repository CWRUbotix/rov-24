from gui.event_nodes.subscriber import GUIEventSubscriber
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget

from rov_msgs.msg import IPAddress


class IPWidget(QWidget):

    signal = pyqtSignal(IPAddress)

    def __init__(self) -> None:
        super().__init__()
        # Boilerplate PyQt Setup to link to ROS through a signal/subscriber
        self.signal.connect(self.refresh)
        self.sub = GUIEventSubscriber(IPAddress, 'ip_address', self.signal)

        ip_layout = QVBoxLayout()
        self.ethernet_label = QLabel(f'Ethernet: {IPAddress.ETHERNET_ADDRESS__DEFAULT}')
        self.wireless_label = QLabel(f'Wireless: {IPAddress.WIRELESS_ADDRESS__DEFAULT}')

        ip_layout.addWidget(self.ethernet_label)
        ip_layout.addWidget(self.wireless_label)
        self.setLayout(ip_layout)

    @pyqtSlot(IPAddress)
    def refresh(self, msg: IPAddress) -> None:
        self.ethernet_label.setText(f'Ethernet: {msg.ethernet_address}')
        self.wireless_label.setText(f'Wireless: {msg.wireless_address}')
