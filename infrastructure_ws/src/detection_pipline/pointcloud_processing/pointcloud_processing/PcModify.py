import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

class PcModify(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        
        # 创建订阅者和发布者
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.pointcloud_callback,
            10)
        self.publisher = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/pointcloud',
            10)
        
        # 预定义点云字段结构
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

    def pointcloud_callback(self, msg):
        try:
            # 原始点云数量
            original_count = msg.width * msg.height
            
            # 过滤NaN值
            points = list(pc2.read_points(
                msg, 
                field_names=("x", "y", "z", "intensity"), 
                skip_nans=True
            ))
            filtered_count = len(points)
            
            # # 打印过滤结果
            # self.get_logger().info(
            #     f"Points - Original: {original_count}, Filtered: {filtered_count}"
            # )

            if filtered_count == 0:
                return

            # # 计算点云范围
            # x_coords = [p[0] for p in points]
            # y_coords = [p[1] for p in points]
            # z_coords = [p[2] for p in points]
            
            # min_x, max_x = min(x_coords), max(x_coords)
            # min_y, max_y = min(y_coords), max(y_coords)
            # min_z, max_z = min(z_coords), max(z_coords)
            
            # self.get_logger().info(
            #     f"Original Range - X: [{min_x:.2f}, {max_x:.2f}] "
            #     f"Y: [{min_y:.2f}, {max_y:.2f}] "
            #     f"Z: [{min_z:.2f}, {max_z:.2f}]"
            # )

            # # 坐标范围过滤
            # processed_points = [
            #     p for p in points
            #     if 0 <= p[0] <= 60
            #     and -40 <= p[1] <= 40
            # ]

            # filtered_count_range = len(processed_points)
            # self.get_logger().info(
            #     f"Points - Original: {original_count}, Null_Filtered: {filtered_count}, Range_Filtered: {filtered_count_range}"
            # )
            
            # 创建新点云消息
            header = msg.header
            header.stamp = self.get_clock().now().to_msg()
            
            processed_cloud = pc2.create_cloud(
                header,
                self.fields,
                points
            )
            
            # 发布处理后的点云
            self.publisher.publish(processed_cloud)

        except Exception as e:
            self.get_logger().error(f"Processing failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = PcModify()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()