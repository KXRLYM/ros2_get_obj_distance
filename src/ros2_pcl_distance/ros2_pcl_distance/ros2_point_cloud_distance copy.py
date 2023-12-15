import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np

from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image
import sensor_msgs.msg as msg
from yolov8_msgs.msg import Detection, DetectionArray, Mask, Point2D
import message_filters
from shapely.geometry import Point, Polygon
from std_msgs.msg import Header

class PointCloudDistanceNode(Node):
    def __init__(self):
        super().__init__('ros2_point_cloud_distance')
        
        self.point_sub = message_filters.Subscriber(self, PointCloud2, "points2")
        self.mask_sub = message_filters.Subscriber(self, DetectionArray, "yolo/detections_stamped")
        self.cam_info_sub = message_filters.Subscriber(self, CameraInfo, "/rgb/camera_info")

        self.point_pub = self.create_publisher(
            PointCloud2,
            '/matching_points',
            10
        )

        ts = message_filters.ApproximateTimeSynchronizer([self.point_sub, self.mask_sub], 10, 3)
        ts.registerCallback(self.cloudCallback)


        """
        self.point_sub = self.create_subscription(
            PointCloud2,
            '/points2',
            self.cloudCallback,
            10
        )
        self.point_sub

        self.mask_sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.maskCallback,
            10
        )
        """
        
    def cloudCallback(self, point_msg, mask_msg):
        self.get_logger().info("receiving points")
        points = rnp.numpify(point_msg)
        
        nan_mask = np.isnan(points['xyz']).any(axis=1)
        
        # Filter out nan rows in 'xyz' and 'rgb'
        filtered_xyz = points['xyz'][~nan_mask]
        filtered_rgb = points['rgb'][~nan_mask]
        point_cloud = filtered_xyz

        for detection in mask_msg.detections:
            mask_array = None
            if (len(detection.mask.data) > 0) :
                mask_array = np.array([[int(ele.x), 0, int(ele.y)]
                for ele in detection.mask.data])

                matching_points_mask = np.isin(point_cloud[:, [0,2]], mask_array).all(axis=1)
                matching_points = point_cloud[matching_points_mask]
                print(mask_array)
                print("-----------------------------------------------------------------------------------------")
                print(point_cloud)
                # pt_msg = rnp.msgify(PointCloud2, matching_points)
                # self.point_pub.publish(matching_points)
            
                self.publishMatchingPoint(mask_array)

    def publishMatchingPoint(self, matching_points):
        if len(matching_points) > 0:
            
            header = Header()

            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = '/camera_base'
            points = point_cloud2.create_cloud_xyz32(header, matching_points)
            self.point_pub.publish(points)
            self.get_logger().info("Published matching points as PointCloud2")



    """
    def maskCallback(self, msg):
        print(type(msg))
        for detection in msg.detections:
             if (len(detection.mask.data) > 0) :
                self.get_logger().info("incoming mask")
    """

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
