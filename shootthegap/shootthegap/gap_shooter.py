import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Float64

from sensor_msgs.msg import LaserScan

class GapShooter(Node):

    def __init__(self):
        # super().__init__('minimal_publisher')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

        super().__init__('gap_shooter')
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.scan_sub  # prevent unused variable warning

        self.odom_pub = self.create_publisher(Float64, '/commands/motor/duty_cycle', 10)
        self.steer_pub = self.create_publisher(Float64, '/commands/servo/position', 10)

    def scan_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        largest_gap = 0.0
        largest_gap_angle = 0.0
        largest_gap_distance = 0.0
        last_point_r = msg.ranges[0]
        last_point_t = msg.angle_min


        point_t = msg.angle_min
        for point_r in msg.ranges[:-1]:
            
            if point_r <= msg.angle_min or point_r >= msg.angle_max:
                continue
            point_t += msg.angle_increment

            val = float(point_r * point_r * last_point_r * last_point_r) - float(2.0 * last_point_r * point_r * math.cos((last_point_t - point_t)))
            dist = math.sqrt(abs(val))
            # if val < 0:
            #     print(val) 
            # dist = math.sqrt(point_r * point_r * last_point_r * last_point_r - 2.0 * last_point_r * point_r * math.cos(last_point_t - point_t));

            if (dist > largest_gap and point_t > -3.14 / 2.0 and point_t < 3.14 / 2.0 and last_point_t > -3.14 / 2.0 and last_point_t < 3.14 / 2.0):
                
                largest_gap = dist
                largest_gap_dist = point_r > last_point_r if last_point_r else point_r
                avgt = (point_t + last_point_t) / 2.0
                largest_gap_angle = avgt

            last_point_r = point_r
            last_point_t = point_t

        if (largest_gap_dist > 3.5):
            odom_val = -1.0 / 10.0
        else:
            odom_val = -1.0 / 10.0
    
        interp = (-largest_gap_angle - (-3.1415 / 2.0)) * (1 - 0) / ((3.1415 / 2.0) - (-3.1415 / 2.0));
        steer_val = interp;




        # for i in range(1, len(msg.ranges)):
        #     point_r = msg.ranges[i]
        #     point_t = msg.angle_min + (i * msg.angle_increment)

        #     if (point_r <= msg.angle_min or point_r >= msg.angle_max):
        #         continue;

        #     val = float(point_r * point_r * last_point_r * last_point_r) - float(2.0 * last_point_r * point_r * math.cos((last_point_t - point_t)))
        #     # dist = math.sqrt(val)
        #     if val < 0:
        #         print(val) 

        odom_val = 0
        odom_msg = Float64()
        odom_msg.data = float(odom_val)
        # self.odom_pub.publish(odom_msg)


        steer_val = 0.7
        steer_msg = Float64()
        steer_msg.data = float(steer_val)
        self.steer_pub.publish(steer_msg)




    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1


def main(args=None):
    rclpy.init(args=args)

    gapShooter = GapShooter()

    rclpy.spin(gapShooter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gapShooter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()