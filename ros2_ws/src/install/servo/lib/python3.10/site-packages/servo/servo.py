import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time
import math

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_cotroller')
        
        self.subscription = self.create_subscription(Float32, '/hardware/servo_goal_angle', self.listener_callback, 10)
        self.subscription 
        
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 500000, timeout=1)
            time.sleep(2) 
            self.get_logger().info('Arduino ready')
        except Exception as e:
            self.get_logger().error(f' Error connceting with Arduino: {e}')
            self.arduino = None
           
    def listener_callback(self, msg):
           
        wheel_angle = str(round(msg.data,3))+'\n'
        time.sleep(0.1)
        self.arduino.write(wheel_angle.encode())  
        print(msg.data)      
        
    


def main(args = None):
    print('Hi from servo.')
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
    
