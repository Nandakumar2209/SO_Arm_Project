import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class SineWaveNode(Node):
    def __init__(self):
        super().__init__('sine_wave_node')
        # This topic is created by the controller we defined in Step 2
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_position_controller/commands', 10)
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9 # Convert to seconds
        
        # Sine wave parameters
        amplitude = 0.8  # Radians
        frequency = 0.5  # Hz
        angle = amplitude * math.sin(2 * math.pi * frequency * t)

        msg = Float64MultiArray()
        msg.data = [angle] # Add more values if controlling multiple joints
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = SineWaveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()