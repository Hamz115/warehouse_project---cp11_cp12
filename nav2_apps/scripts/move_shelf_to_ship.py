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
        self.future = self.client.call_async(req)

class PolygonPublisher(Node):
    def __init__(self):
        super().__init__('polygon_publisher')
        self.global_publisher_ = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.local_publisher_ = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.timer = self.create_timer(1, self.publish_polygon)

        self.square_size = 0.9
        self.half_size = self.square_size / 2 

        self.circle_shape = [
            Point32(x=0.25, y=0.0, z=0.0),
            Point32(x=0.23096988312782168, y=0.09567085809127245, z=0.0),
            Point32(x=0.1767766952966369, y=0.17677669529663687, z=0.0),
            Point32(x=0.09567085809127246, y=0.23096988312782168, z=0.0),
            Point32(x=0.0, y=0.25, z=0.0),
            Point32(x=-0.09567085809127243, y=0.23096988312782168, z=0.0),
            Point32(x=-0.17677669529663687, y=0.1767766952966369, z=0.0),
            Point32(x=-0.23096988312782168, y=0.09567085809127247, z=0.0),
            Point32(x=-0.25, y=0.0, z=0.0),
            Point32(x=-0.2309698831278217, y=-0.09567085809127242, z=0.0),
            Point32(x=-0.17677669529663692, y=-0.17677669529663687, z=0.0),
            Point32(x=-0.09567085809127258, y=-0.23096988312782163, z=0.0),
            Point32(x=0.0, y=-0.25, z=0.0),
            Point32(x=0.0956708580912725, y=-0.23096988312782166, z=0.0),
            Point32(x=0.17677669529663684, y=-0.17677669529663692, z=0.0),
            Point32(x=0.23096988312782163, y=-0.0956708580912726, z=0.0)
        ]

        self.square_shape = [
            Point32(x=-self.half_size, y=self.half_size, z=0.0), 
            Point32(x=self.half_size, y=self.half_size, z=0.0),    
            Point32(x=self.half_size, y=-self.half_size, z=0.0),   
            Point32(x=-self.half_size, y=-self.half_size, z=0.0)   
        ]

    def publish_polygon(self, mode):
        polygon_msg = Polygon()
        if (mode == 'circle'):
            polygon_msg.points = self.circle_shape
        elif (mode == 'square'):
            polygon_msg.points = self.square_shape
        else:
            self.get_logger().info(f'Invalid shape type {mode}')
            return None
        self.global_publisher_.publish(polygon_msg)
        self.local_publisher_.publish(polygon_msg)
        self.get_logger().info(f'Publishing polygon footprint of type {mode}')
        return None

class ElevatorPublisher(Node):
    def __init__(self):
        super().__init__('elevator_publisher')
        self.publisher_ = self.create_publisher(String, '/elevator_down', 10)
        self.lift_publisher_ = self.create_publisher(String, '/elevator_up', 10)

    def drop(self):
        msg = String()
        self.publisher_.publish(msg)
        time.sleep(6)
        self.get_logger().info(f'Shelf unloaded')
        return None

    def lift(self):
        msg = String()
        self.lift_publisher_.publish(msg)
        time.sleep(6)
        self.get_logger().info(f'Shelf loaded')
        return None

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.duration = 6  # Set the duration for which the robot should move back

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

# Shelf positions for picking
shelf_positions = {
    "init": [-0.520478, -0.0911187, -0.108273, 0.994121],
    "loading_position": [1.85362, -0.193069, -0.699828, 0.714312],
    "corridor": [1.04646, -0.864842, 0.957607, 0.288078],
    "shipping_position": [1.4481, 0.38586, 0.678121, 0.73495]
    }

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = shelf_positions['init'][0]
    initial_pose.pose.position.y = shelf_positions['init'][1]
    initial_pose.pose.orientation.z = shelf_positions['init'][2]
    initial_pose.pose.orientation.w = shelf_positions['init'][3]

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()
    navigator.setInitialPose(initial_pose)

    # Got to the loading position
    request_item_location = 'loading_position'
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_positions[request_item_location][2]
    shelf_item_pose.pose.orientation.w = shelf_positions[request_item_location][3]
    print('Received request for item picking at ' + request_item_location + '.')

    for n in range(5):
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
            print(f'Task completed successfully on {n} attempt.')
            break

        elif result == TaskResult.CANCELED:
            print('Task at ' + request_item_location +
                ' was canceled. Returning to staging point...')
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            navigator.goToPose(initial_pose)
            break

        elif result == TaskResult.FAILED:
            print('Task at ' + request_item_location + f' failed on its {n + 1} attempt.')
            if n + 1 == 5: exit(-1)
            time.sleep(1.0)

    # Instance the elevator publisher
    elevator_publisher = ElevatorPublisher()

    # Instance the footprint publisher
    footprint_publisher = PolygonPublisher()
    
    # Instance the service client for shelf lifting
    for n in range(5):
        client = ClientAsync()
        print(f'Attempt {n+1}: Calling shelf lifting service.')
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

    # Handle lift and backwards movement
    footprint_publisher.publish_polygon('square')
    mover = RobotMover()
    mover.move_back()

    # Go to the corridor
    request_item_location = 'corridor'
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
        print('Reached the middle of the corridor.')

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

    

    # Got to the shipping position
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
        footprint_publisher.publish_polygon('circle')
        elevator_publisher.drop()
        mover.move_back()

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)


     # Got to the init position
    request_item_location = 'init'
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
        print('Task finished.')

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