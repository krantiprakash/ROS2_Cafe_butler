#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
from std_srvs.srv import Empty


class SequentialNav(Node):
    def __init__(self):
        super().__init__('task6node')
        
        # Create action client for navigating to a pose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create publisher for /initialpose topic
        self._initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self._confirmation_service = self.create_service(Empty, '/confirmation', self.handle_confirmation)
    
        self.waiting_for_confirmation = False
        self.confirmation_timer = None
        self.count_confirmation = 0
        self.timeout_duration = 10.0
        self.home_goal = self.create_home_goal()
        self.kitchen_goal = self.create_kitchen_goal()
        
        # Define fixed positions
        self.kitchen_position = (1.0, 7.0)
        self.home_position = (0.0, 0.0)
        self.table_positions = {
            1: (5.5, 5.5),  # Table 1 position
            2: (5.5, 3.5),  # Table 2 position
            3: (5.5, 1.5)   # Table 3 position
        }

        # Input for multiple tables
        table_order_input = input("Enter table numbers separated by spaces (e.g., '1 2 3'): ")
        # table_numbers = [int(x) for x in table_order_input.split()].sort()
        self.table_numbers = sorted([int(x) for x in table_order_input.split()])

        # Prepare goals
        self.goals = self.prepare_goals(self.table_numbers)
        
        # Define initial pose and publish it
        self.publish_initial_pose()
        
        # Start processing goals
        self.current_goal_index = 0
        self.send_next_goal()

    def prepare_goals(self, table_numbers):
        goals = []

        # Add the kitchen as the first goal
        goals.append(self.create_goal(self.kitchen_position))

        # Add goals for each table in the order received
        for table in table_numbers:
            table_position = self.table_positions.get(table)
            if table_position:
                goals.append(self.create_goal(table_position))

        # Add the home position as the final goal
        goals.append(self.create_goal(self.home_position))

        return goals

    def create_goal(self, position):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        return goal
    
    def create_home_goal(self):
        home_goal = PoseStamped()
        home_goal.header.frame_id = 'map'
        home_goal.pose.position.x = 0.0
        home_goal.pose.position.y = 0.0
        home_goal.pose.orientation.w = 1.0
        return home_goal

    def create_kitchen_goal(self):
        kitchen_goal = PoseStamped()
        kitchen_goal.header.frame_id = 'map'
        kitchen_goal.pose.position.x = 1.0
        kitchen_goal.pose.position.y = 7.0
        kitchen_goal.pose.orientation.w = 1.0
        return kitchen_goal

    def publish_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header = Header()
        initial_pose.header.stamp.sec = 0
        initial_pose.header.stamp.nanosec = 0
        initial_pose.header.frame_id = 'map'
        
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        
        # Covariance values
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06
        ]
        
        self._initialpose_publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published.')

    def send_next_goal(self):
        if self.current_goal_index < len(self.goals):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.goals[self.current_goal_index]
            self._action_client.wait_for_server()
            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info('All goals have been processed.')
            rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle1 = future.result()
        if not goal_handle1.accepted:
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle1.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal {self.current_goal_index + 1} result received.')
        
        self.current_goal_index += 1
        if self.current_goal_index > 1:
            self.waiting_for_confirmation = True
            self.start_confirmation_timer()
        else:
            self.send_next_goal()
       


    def handle_confirmation(self, request, response):
        self.get_logger().info('Confirmation service called.')
        if self.waiting_for_confirmation:
            self.waiting_for_confirmation = False
            self.count_confirmation += 1
            if self.confirmation_timer is not None:
                self.confirmation_timer.cancel()
                self.confirmation_timer = None

            if self.current_goal_index == len(self.goals) - 1:  # At the last goal (table)
                if self.count_confirmation < len(self.table_numbers):
                    self.get_logger().info('Confirmation received at the table. But all the table did not confirm .')
                    self.navigate_to_kitchen()
                    return response

                else:
                    self.get_logger().info('Confirmation received at all the tables.')
                    self.send_next_goal()
                    return response
            self.send_next_goal()
        else:
            self.get_logger().warn('Confirmation received, but not waiting for confirmation.')
        return response

    def start_confirmation_timer(self):
        self.get_logger().info(f'Waiting for confirmation for {self.timeout_duration} seconds...')
        self.confirmation_timer = self.create_timer(
            self.timeout_duration, self.confirmation_timeout
        )

    def confirmation_timeout(self):
        if self.waiting_for_confirmation:
            self.get_logger().info('No confirmation received.')
            self.waiting_for_confirmation = False
            self.confirmation_timer.cancel()
            self.confirmation_timer = None

            if self.current_goal_index == len(self.goals) - 1:  # At the table
                self.get_logger().info('Navigating to kitchen before home.')
                self.navigate_to_kitchen()
            else:
                self.send_next_goal()
           
    def navigate_to_kitchen(self):
        kitchen_goal_msg = NavigateToPose.Goal()
        kitchen_goal_msg.pose = self.kitchen_goal
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            kitchen_goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.kitchen_goal_response_callback)

    def kitchen_goal_response_callback(self, future):
        goal_handle2 = future.result()
        if not goal_handle2.accepted:
            self.get_logger().info('Kitchen goal rejected.')
            self.go_home()
            return

        self.get_logger().info('Kitchen goal accepted.')
        self._get_result_future = goal_handle2.get_result_async()
        self._get_result_future.add_done_callback(self.kitchen_get_result_callback)

    def kitchen_get_result_callback(self, future):
        self.get_logger().info('Reached kitchen. Navigating to home.')
        self.go_home()

    def go_home(self):
        home_goal_msg = NavigateToPose.Goal()
        home_goal_msg.pose = self.home_goal
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            home_goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.home_goal_response_callback)

    def home_goal_response_callback(self, future):
        goal_handle3 = future.result()
        if not goal_handle3.accepted:
            self.get_logger().info('Home goal rejected.')
            rclpy.shutdown()
            return
        self.get_logger().info('Home goal accepted.')
        self._get_result_future = goal_handle3.get_result_async()
        self._get_result_future.add_done_callback(self.home_get_result_callback)

    def home_get_result_callback(self, future):
        self.get_logger().info('Returned to home position.')
        rclpy.shutdown()


    def feedback_callback(self, feedback_msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SequentialNav()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
