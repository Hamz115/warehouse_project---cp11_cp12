import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from attach_service.srv import GoToLoading
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import cos, sin, pi

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Client for shelf lifting service
class ClientAsync(Node):
    def __init__(self):
        super().__init__('go_to_loading')
        self.client = self.create_client(GoToLoading, 'approach_shelf')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self):
        req = GoToLoading.Request()
        req.attach_to_shelf = True
        self.future = self.client.call_async(req)

class ElevatorPublisher(Node):
    def __init__(self):
        super().__init__('elevator_publisher')
        self.liftdown_publisher_ = self.create_publisher(String, '/elevator_down', 10)
        self.liftup_publisher_ = self.create_publisher(String, '/elevator_up', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.duration = 6  # Set the duration for which the robot should move back

    def drop(self):
        msg = String()
        self.liftdown_publisher_.publish(msg)
        time.sleep(6)
        self.get_logger().info(f'Cart down')
        return None

    def lift(self):
        msg = String()
        self.liftup_publisher_.publish(msg)
        time.sleep(6)
        self.get_logger().info(f'Cart up')
        return None

    def move_back(self):
        # Start time
        start_time = time.time()

        # Publish a message to move the robot backwards
        msg = Twist()
        msg.linear.x = -0.2  # Move backwards
        self.publisher_.publish(msg)
        self.get_logger().info('Moving the robot backwards')

        while time.time() - start_time < self.duration:
            self.publisher_.publish(msg)
            self.get_logger().info('Moving the robot backwards')
            time.sleep(0.1)  # Adjust the sleep time as needed for responsiveness

        # Stop the robot by publishing zero velocities
        self.publisher_.publish(Twist())
        self.get_logger().info('Stopping the robot')

    def wait(self):
        duration = Duration(seconds=3)
        rate = self.create_rate(10, self.get_clock())
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            rate.sleep
       
class footprintPublisher(Node):
    def __init__(self):
        super().__init__('footprint_publisher')
        self.global_publisher_ = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.local_publisher_ = self.create_publisher(Polygon, '/local_costmap/footprint', 10)

    def publish_cart_footprint(self):

        footprint = Polygon()
        point1 = Point32()
        point1.x = -0.45
        point1.y = 0.45

        point2 = Point32()
        point2.x = 0.45
        point2.y = 0.45

        point3 = Point32()
        point3.x = 0.45
        point3.y = -0.45

        point4 = Point32()
        point4.x = -0.45
        point4.y = -0.45

        footprint.points = [point1, point2, point3, point4]
            
        self.local_publisher_.publish(footprint)
        self.global_publisher_.publish(footprint)

    def publish_robot_footprint(self):
        footprint = Polygon()
        points = []
        for angle in range(0, 360, 10):
            point = Point32()
            point.x = 0.25 * cos(angle * pi / 180)  
            point.y = 0.25 * sin(angle * pi / 180)  
            points.append(point)

            footprint.points = points
            
            self.local_publisher_.publish(footprint)
            self.global_publisher_.publish(footprint)

# Shelf positions for picking
shelf_positions = {
    "init_position": [ -0.29141, -0.463275, -0.106878, 0.994272],
    "loading_position": [ 3.95411, -1.32686, -0.837662, 0.54619],
    "Midpoint": [1.04646, -0.864842, 0.957607, 0.288078],
    "shipping_position": [1.4481, 0.38586, 0.678121, 0.73495]
    }

# Shipping destination for picked products
shipping_destinations = {}

'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''


def main():

    rclpy.init()

    navigator = BasicNavigator()

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = shelf_positions['init_position'][0]
    initial_pose.pose.position.y = shelf_positions['init_position'][1]
    initial_pose.pose.orientation.z = shelf_positions['init_position'][2]
    initial_pose.pose.orientation.w = shelf_positions['init_position'][3]
    

    # Wait for navigation to activate fully
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    request_item_location = 'loading_position'
    request_destination = ''
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_positions[request_item_location][2]
    shelf_item_pose.pose.orientation.w = shelf_positions[request_item_location][3]
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Got product from ' + request_item_location +
              '! Bringing product to shipping destination (' + request_destination + ')...')

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

    # Instance the elevator publisher
    elevator_publisher = ElevatorPublisher()
    #the service client for shelf lifting
    for n in range(5):
        client = ClientAsync()
        print(f'Calling the attach_shelf service.')
        client.send_request()

        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    client.get_logger().info(f'Result of service call: {response.complete}')
                    
                break

        client.destroy_node()
        if response.complete == True:
            elevator_publisher.lift()
            break

    if n + 1 == 5: exit(-1)
    # Change the robot footprint 
    footprint_publisher = footprintPublisher()
    footprint_publisher.publish_cart_footprint()

    # move the robot back
    elevator_publisher.move_back()

    # move the robot to midpoint to avoid collision
    request_item_location = 'Midpoint'
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_positions[request_item_location][2]
    shelf_item_pose.pose.orientation.w = shelf_positions[request_item_location][3]
    print(f'Wait for few seconds before moving to shipping position')
    elevator_publisher.wait()
    print('Moving robot to ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Reached the middle of the corridor.')

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

    # go to the shipping position
    request_item_location = 'shipping_position'
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_positions[request_item_location][2]
    shelf_item_pose.pose.orientation.w = shelf_positions[request_item_location][3]
    print('Moving robot to ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Unloading the shelf.')
        footprint_publisher.publish_robot_footprint()
        elevator_publisher.drop()
        elevator_publisher.move_back()

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

    #return to the initial position
    request_item_location = 'init_position'
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_positions[request_item_location][2]
    shelf_item_pose.pose.orientation.w = shelf_positions[request_item_location][3]
    print('Moving robot to ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Task completed.')

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)


    while not navigator.isTaskComplete():
        pass

    exit(0)

if __name__ == '__main__':
    main()