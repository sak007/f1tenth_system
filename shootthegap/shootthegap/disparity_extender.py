import rclpy
import math
import numpy as np
import threading
import time

from rclpy.node import Node

from std_msgs.msg import Float64

from sensor_msgs.msg import LaserScan

from std_srvs.srv import Empty

scan_queue_size = 1
packet_time_window = 0.1

# diff between mean and midpoint
d_side_thresh = 5  # skirk, mtl etc
# d_side_thresh = 7  # mtl_obs
# for use with corner detection
# based off of testing with 5
default_max_angle = 60

class PID:
    def __init__(self, kP, kI, kD, max_out = np.radians(34), min_out=None):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.max_out = max_out
        if min_out == None:
            self.min_out = -max_out
        else:
            self.min_out = min_out
        self.integral = 0
        self.last_error = 0
        self.max_i = np.radians(3)

    def step(self, error, dt=1):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.kP * error + self.kI * self.integral + self.kD * derivative
        output = np.clip(output, self.min_out, self.max_out)
        self.last_error = error
        self.integral = np.clip(self.integral, -self.max_i, self.max_i)
        return output

    def reset(self):
        self.integral = 0
        self.last_error = 0

class DisparityExtender(Node):

    # def __init__(self):
        # super().__init__('minimal_publisher')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

        # super().__init__('gap_shooter')
        # self.scan_sub = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.scan_callback,
        #     10)
        # self.scan_sub  # prevent unused variable warning

        # self.odom_pub = self.create_publisher(Float64, '/commands/motor/duty_cycle', 10)
        # self.steer_pub = self.create_publisher(Float64, '/commands/servo/position', 10)

    def __init__(self):

        super().__init__('disparity_extender')
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.lidar_sub  # prevent unused variable warning

        self.speed_pub = self.create_publisher(Float64, '/commands/motor/duty_cycle', 1)
        self.steering_pub = self.create_publisher(Float64, '/commands/servo/position', 1)

        # self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        # self.speed_pub = rospy.Publisher("throttle_out", Float64, queue_size=1)
        # self.steering_pub = rospy.Publisher("steering_out", Float64, queue_size=1)



        # allow resetting by exposing a service
        self.reset_srv = self.create_service(Empty, "reset_drive", self.reset_callback)
        # This is actually "half" of the car width, plus some tolerance.
        # Controls the amount disparities are extended by.
        self.car_width = 0.4
        # This is the difference between two successive LIDAR scan points that
        # can be considered a "disparity". (As a note, at 7m there should be
        # ~0.04m between scan points.)
        self.disparity_threshold = 0.2
        # This is the arc width of the full LIDAR scan data, in degrees
        self.scan_width = 270.0
        # This is the radius to the left or right of the car that must be clear
        # when the car is attempting to turn left or right.
        self.turn_clearance = 0.1
        # This is the maximum steering angle of the car, in degrees.
        self.max_turn_angle = 34.0
        # The slowest speed the car will go
        # Good value here is 0.1
        self.min_speed = 0.07
        # The maximum speed the car will go (the absolute max for the motor is
        # 0.5, which is *very* fast). 0.15 is a good max for slow testing.
        self.max_speed = 0.15  #.20
        self.absolute_max_speed = 0.25  # 0.3
        # The forward distance at which the car will go its minimum speed.
        # If there's not enough clearance in front of the car it will stop.
        self.min_distance = 0.1
        # The forward distance over which the car will go its maximum speed.
        # Any distance between this and the minimum scales the speed linearly.
        self.max_distance = 5
        # The forward distance over which the car will go its *absolute
        # maximum* speed. This distance indicates there are no obstacles in
        # the near path of the car. Distance between this and the max_distance
        # scales the speed linearly.
        self.no_obstacles_distance = 10
        # If forward distance is lower than this, use the alternative method
        # for choosing an angle (where the angle favors a disparity rather than
        # simple distance)
        self.no_u_distance = 1
        # These allow us to adjust the angle of points to consider based on how
        # the wheels are oriented.
        self.min_considered_angle = -default_max_angle
        self.max_considered_angle = default_max_angle
        # We'll use this lock to essentially drop LIDAR packets if we're still
        # processing an older one.
        self.should_stop = False
        self.total_packets = 0
        self.dropped_packets = 0
        # This is just a field that will hold the LIDAR distances.
        self.lidar_distances = None
        # This contains the LIDAR distances, but only "safe" reachable
        # distances, generated by extending disparities.
        self.masked_disparities = None
        # If we're constraining turns to be in the direction of a disparity,
        # then use this array to find indices of possible directions to move.
        self.possible_disparity_indices = None
        # This field will hold the number of LIDAR samples per degree.
        # Initialized when we first get some LIDAR scan data.
        self.samples_per_degree = 0

        self.wall_place_steps = 0

        self.position = [0, 0, 0]
        # self.wall_manager = WallManager()
        self.steps = 0
        # output smoothing
        self.target_angle = 0
        self.current_angle = 0
        self.steerPID = PID(0.8, 0.01, 0.1, np.radians(90))
        self.lastSpeed = 0

    def find_disparities(self):
        """ Scans each pair of subsequent values, and returns an array of indices
        where the difference between the two values is larger than the given
        threshold. The returned array contains only the index of the first value
        in pairs beyond the threshold. """
        to_return = []
        values = self.lidar_distances
        for i in range(len(values) - 1):
            if abs(values[i] - values[i + 1]) >= self.disparity_threshold:
                to_return.append(i)
        return to_return

    def half_car_samples_at_distance(self, distance):
        """ Returns the number of points in the LIDAR scan that will cover half of
        the width of the car along an arc at the given distance. """
        # This isn't exact, because it's really calculated based on the arc length
        # when it should be calculated based on the straight-line distance.
        # However, for simplicty we can just compensate for it by inflating the
        # "car width" slightly.
        distance_between_samples = math.pi * distance / (180.0 * self.samples_per_degree)
        return int(math.ceil(self.car_width / distance_between_samples))

    def extend_disparities(self):
        """ For each disparity in the list of distances, extends the nearestN
        value by the car width in whichever direction covers up the more-
        distant points. Puts the resulting values in self.masked_disparities.
        """
        values = self.lidar_distances
        masked_disparities = np.copy(values)
        disparities = self.find_disparities()
        # Keep a list of disparity end points corresponding to safe driving
        # angles directly past a disparity. We will find the longest of these
        # constrained distances in situations where we need to turn towards a
        # disparity.
        self.possible_disparity_indices = []
        # print("Got %d disparities." % (len(disparities), ))
        for d in disparities:
            a = values[d]
            b = values[d + 1]
            # If extend_positive is true, then extend the nearer value to
            # higher indices, otherwise extend it to lower indices.
            nearer_value = a
            nearer_index = d
            extend_positive = True
            if b < a:
                extend_positive = False
                nearer_value = b
                nearer_index = d + 1
            samples_to_extend = self.half_car_samples_at_distance(nearer_value)
            current_index = nearer_index
            for i in range(samples_to_extend):
                # Stop trying to "extend" the disparity point if we reach the
                # end of the array.
                if current_index < 0:
                    current_index = 0
                    break
                if current_index >= len(masked_disparities):
                    current_index = len(masked_disparities) - 1
                    break
                # Don't overwrite values if we've already found a nearer point
                if masked_disparities[current_index] > nearer_value:
                    masked_disparities[current_index] = nearer_value
                # Finally, move left or right depending on the direction of the
                # disparity.
                if extend_positive:
                    current_index += 1
                else:
                    current_index -= 1
            self.possible_disparity_indices.append(current_index)
        self.masked_disparities = masked_disparities

    def angle_from_index(self, i):
        """ Returns the angle, in degrees, corresponding to index i in the
        LIDAR samples. """
        min_angle = -(self.scan_width / 2.0)
        return min_angle + (float(i) / self.samples_per_degree)

    def index_from_angle(self, i):
        center_index = int(self.scan_width * (self.samples_per_degree / 2))
        return center_index + int(i * float(self.samples_per_degree))

    def find_widest_disparity_index(self):
        """ Returns the index of the distance corresponding to the "widest"
        disparity that we can safely target. """
        masked_disparities = self.masked_disparities
        # Keep this at 0.1 so that we won't identify noise as a disparity
        max_disparity = 0.1
        max_disparity_index = None
        for d in self.possible_disparity_indices:
            # Ignore disparities that are behind the car.
            angle = self.angle_from_index(d)
            if (angle < self.min_considered_angle) or (angle > self.max_considered_angle):
                continue
            angle = d * self.samples_per_degree
            distance = masked_disparities[d]
            prev = distance
            after = distance
            # The disparity must have been extended from one of the two
            # directions, so we can calculate the distance of the disparity by
            # checking the distance between the points on either side of the
            # index (note that something on the endpoint won't matter here
            # either. The inequalities are just for bounds checking, if either
            # one is outside the array, then we already know the disparity was
            # extended from a different direction.
            if (d - 1) > 0:
                prev = masked_disparities[d - 1]
            if (d + 1) < len(masked_disparities):
                after = masked_disparities[d + 1]
            difference = abs(prev - after)
            if difference > max_disparity:
                max_disparity = difference
                max_disparity_index = d
        return max_disparity_index

    def find_new_angle(self, steer_offset):
        """ Returns the angle of the farthest possible distance that can be reached
        in a direct line without bumping into edges. Returns the distance in meters
        and the angle in degrees. """
        self.extend_disparities()
        limited_values = self.masked_disparities
        max_distance = -1.0e10
        angle = 0.0
        # Constrain the arc of possible angles we consider.
        min_considered = self.min_considered_angle
        # if steer_offset == 1:
        #     min_considered += 45
        max_considered = self.max_considered_angle
        # if steer_offset == -1:
        #     max_considered -= 45
        # self.pub_angle_limits[0].publish(Float32(min_considered))
        # self.pub_angle_limits[1].publish(Float32(max_considered))
        if (steer_offset < 0):
            min_sample_index = self.index_from_angle(0)
            self.min_considered_angle = 0
        else:
            min_sample_index = self.index_from_angle(min_considered)

        if steer_offset > 0:
            max_sample_index = self.index_from_angle(0)
            self.max_considered_angle = 0
        else:
            max_sample_index = self.index_from_angle(max_considered)
        limited_values = limited_values[min_sample_index:max_sample_index]
        for i in range(len(limited_values)):
            distance = limited_values[i]
            if distance > max_distance:
                max_distance = distance
                angle = self.min_considered_angle + float(i) / self.samples_per_degree
        return distance, angle

    def scale_speed_linearly(self, speed_low, speed_high, distance, distance_low, distance_high):
        """ Scales the speed linearly in [speed_low, speed_high] based on the
        distance value, relative to the range [distance_low, distance_high]. """
        distance_range = distance_high - distance_low
        ratio = (distance - distance_low) / distance_range
        speed_range = speed_high - speed_low
        return speed_low + (speed_range * ratio)

    def duty_cycle_from_distance(self, distance):
        """ Takes a forward distance and returns a duty cycle value to set the
        car's velocity. Fairly unprincipled, basically just scales the speed
        directly based on distance, and stops if the car is blocked. """
        if distance <= self.min_distance:
            return 0.0
        if distance >= self.no_obstacles_distance:
            return self.absolute_max_speed
        if distance >= self.max_distance:
            return self.scale_speed_linearly(self.max_speed, self.absolute_max_speed, distance, self.max_distance,
                                             self.no_obstacles_distance)
        return self.scale_speed_linearly(self.min_speed, self.max_speed, distance, self.min_distance, self.max_distance)

    def degrees_to_steering_percentage(self, degrees):
        """ Returns a steering "percentage" value between 0.0 (left) and 1.0
        (right) that is as close as possible to the requested degrees. The car's
        wheels can't turn more than max_angle in either direction. """
        max_angle = self.max_turn_angle
        if degrees > max_angle:
            return max_angle
        if degrees < -max_angle:
            return -max_angle
        # This maps degrees from -max_angle to +max_angle to values from 0 to 1.
        #   (degrees - min_angle) / (max_angle - min_angle)
        # = (degrees - (-max_angle)) / (max_angle - (-max_angle))
        # = (degrees + max_angle) / (max_angle * 2)
        # return 1.0 - ((degrees + max_angle) / (2 * max_angle))
        return degrees

    def adjust_angle_for_car_side(self, target_angle):
        """ Takes the target steering angle, the distances from the LIDAR, and the
        angle covered by the LIDAR distances. Basically, this function attempts to
        keep the car from cutting corners too close to the wall. In short, it will
        make the car go straight if it's currently turning right and about to hit
        the right side of the car, or turning left or about to hit the left side 
        f the car. """
        scan_width = self.scan_width
        car_tolerance = self.turn_clearance
        distances = self.lidar_distances
        turning_left = target_angle > 0.0
        # Get the portion of the LIDAR samples facing sideways and backwards on
        # the side of the car in the direction of the turn.
        samples_per_degree = float(len(distances)) / scan_width
        number_of_back_degrees = (scan_width / 2.0) - 90.0
        needed_sample_count = int(number_of_back_degrees * samples_per_degree)
        side_samples = []
        if turning_left:
            side_samples = distances[len(distances) - needed_sample_count:]
        else:
            side_samples = distances[:needed_sample_count]
        # Finally, just make sure no point in the backwards scan is too close.
        # This could definitely be more exact with some proper math.
        for v in side_samples:
            if v <= car_tolerance:
                return 0.0
        return target_angle

    def adjust_angle_to_avoid_uturn(self, target_angle, forward_distance):
        """ When the car's forward distance is small, it can favor turning to
        the side of a wide track. This function attempts to detect when such a
        case may occur and force the steering angle to follow a disparity
        instead. """
        if forward_distance > self.no_u_distance:
            return target_angle
        target_index = self.find_widest_disparity_index()
        if target_index is None:
            return target_angle
        return self.angle_from_index(target_index)

    def update_considered_angle(self, steering_angle):
        actual_angle = steering_angle
        if actual_angle < -self.max_turn_angle:
            actual_angle = -self.max_turn_angle
        if actual_angle > self.max_turn_angle:
            actual_angle = self.max_turn_angle
        self.min_considered_angle = -default_max_angle
        self.max_considered_angle = default_max_angle
        if actual_angle > 0:
            self.min_considered_angle -= actual_angle
        if actual_angle < 0:
            self.max_considered_angle += actual_angle

    def lidar_callback(self, msg):
        ranges = msg.ranges
        start_time = time.time()
        # leftover from the ros -> gym conversion
        # TODO: refactor this out once it all runs
        distances = ranges
        self.lidar_distances = distances
        self.samples_per_degree = float(len(distances)) / self.scan_width
        target_distance, target_angle = self.find_new_angle(0)
        safe_distances = self.masked_disparities
        # forward_distance = safe_distances[len(safe_distances) / 2]
        # forward_distance = self.get_safe_forward_distance(
        #     self.lidar_distances, index=self.index_from_angle(target_angle))
        forward_distance = self.get_safe_forward_distance(self.lidar_distances, index=None)
        #target_angle = self.adjust_angle_to_avoid_uturn(target_angle,
        #    forward_distance)
        # target_angle = self.adjust_angle_for_car_side(target_angle)
        # print("tgt angle:", target_angle)
        desired_speed = self.duty_cycle_from_distance(forward_distance)
        # desired_speed = self.max_speed
        self.update_considered_angle(target_angle)
        steering_percentage = self.degrees_to_steering_percentage(target_angle)
        target_position = np.radians(target_angle)
        # print(target_angle)

        steer_out = self.steerPID.step(0, -target_position)
        steering_angle = steer_out
        # speed = self.deacc_limit(desired_speed)
        speed = desired_speed
        self.lastSpeed = speed

        duration = time.time() - start_time
        self.wall_place_steps += 1
        self.steps += 1

        print(((target_angle/60.0) + 1)/2.0) #, np.degrees(steering_angle))
        steer_msg = Float64()

        steer_msg.data = ((target_angle/60.0) + 1)/2.0

        self.steering_pub.publish(steer_msg)
        # self.speed_pub.publish(speed)
        # return steering_angle, speed

    def get_safe_forward_distance(self, lidar_ranges, index=None):
        window = int(self.samples_per_degree * 0.5)
        if index is None:
            mid = int(len(lidar_ranges) / 2)
        else:
            mid = index
        dist = np.mean(lidar_ranges[mid - window:mid + window])
        return dist

    # def check_if_corner(self, ranges):
    #     if (len(ranges) <= 0):
    #         return False, 0
    #     adjusted = ranges - np.min(ranges)
    #     mean = np.mean(adjusted)
    #     # of couse, min should be 0 now
    #     midpoint = (np.max(adjusted) + np.min(adjusted)) / 2.0
    #     score = abs(mean - midpoint)
    #     result = score > 2
    #     return result, score


    def deacc_limit(self, speed):
        if (speed < self.lastSpeed):
            diff = self.lastSpeed - speed
            diff = min(diff, 0.4)
            return self.lastSpeed - diff
        return speed

    def reset_callback(self, msg):
        self.steerPID.reset()
        return []



def main(args=None):
    rclpy.init(args=args)

    disparityExtender = DisparityExtender()

    rclpy.spin(disparityExtender)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    disparityExtender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()