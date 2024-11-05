import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PBVSSubscriber(Node):
    def __init__(self):
        super().__init__('pbvs_subscriber')
        self.subscription = self.create_subscription(
            String,
            'pbvs_data',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # Print the received message data to the terminal
        self.get_logger().info(f'Received:\n{msg.data}\n')

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create the subscriber node
    pbvs_subscriber = PBVSSubscriber()

    # Keep the node active until interrupted
    rclpy.spin(pbvs_subscriber)

    # Shutdown after interrupt
    pbvs_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
