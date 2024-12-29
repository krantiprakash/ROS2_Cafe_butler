#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header

class SequentialNav(Node):
    def __init__(self):
        super().__init__('task5node')
        
        # Create action client for navigating to a pose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create publisher for /initialpose topic
        self._initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
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
        table_numbers = sorted([int(x) for x in table_order_input.split()])

        # Prepare goals
        self.goals = self.prepare_goals(table_numbers)
        
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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal {self.current_goal_index + 1} result received.')
        self.current_goal_index += 1
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SequentialNav()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
