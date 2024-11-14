from rclpy.node import Node
import sensor_msgs.msg
from math import pi as PI


### AUXILIARY FUNCTIONS
class LidarData:

    def __init__(self):

        self.values = [] # Actual point data, size is (row_step*height)

        # 2D structure of the point cloud. If the cloud is unordered, height is
        # 1 and width is the length of the point cloud.
        self.height = 0
        self.width = 0

        self.point_step = 0 # Length of a point in bytes
        self.row_step = 0 # Length of a row in bytes
        self.fields = [] # Describes the channels and their layout in the binary data blob.
        self.is_dense = False # True if there are no invalid points
        self.timeStamp = 0  # seconds

    def __str__(self):
        s = (
            "LidarData: {\n   height: "
            + str(self.height)
            + "\n   width: "
            + str(self.width)
        )
        s = (
            s
            + "\n   point_step: "
            + str(self.point_step)
            + "\n   row_step: "
            + str(self.row_step)
        )
        s = (
            s
            + "\n   fields: "
            + str(self.fields)
            + "\n   is_dense: "
            + str(self.is_dense)
        )
        s = (
            s
            + "\n   timeStamp: "
            + str(self.timeStamp)
            + "\n   values: "
            + str(self.values)
            + "\n}"
        )
        return s


def lidarScan2LidarData(scan):
    """
    Translates from ROS LidarScan to JderobotTypes LidarData.
    @param scan: ROS LidarScan to translate
    @type scan: LidarScan
    @return a LidarData translated from scan
    """
    lidar = LidarData()
    lidar.values = scan.data
    lidar.height = scan.height
    lidar.width = scan.width
    lidar.point_step = scan.point_step
    lidar.row_step = scan.row_step
    lidar.fields = scan.fields
    lidar.is_dense = scan.is_dense
    lidar.timeStamp = scan.header.stamp.sec + (scan.header.stamp.nanosec * 1e-9)
    return lidar


### HAL INTERFACE ###
class LidarNode(Node):

    def __init__(self, topic):
        super().__init__("lidar_node")
        self.sub = self.create_subscription(
            sensor_msgs.msg.LidarScan, topic, self.listener_callback, 10
        )
        self.last_scan_ = sensor_msgs.msg.LidarScan()

    def listener_callback(self, scan):
        self.last_scan_ = scan

    def getLidarData(self):
        return lidarScan2LidarData(self.last_scan_)
