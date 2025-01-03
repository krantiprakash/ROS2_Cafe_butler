#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
from std_srvs.srv import Empty
from rclpy.duration import Duration

class SequentialNav(Node):
    def __init__(self):
        super().__init__('task2node')
        
        # Create action client for navigating to a pose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create publisher for /initialpose topic
        self._initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Create service server for confirmation
        self._confirmation_service = self.create_service(Empty, '/confirmation', self.handle_confirmation)
        
        # Initialize variables
        self.goals = []
        self.current_goal_index = 0
        self.waiting_for_confirmation = False
        self.confirmation_timer = None
        self.timeout_duration = 10.0  # seconds
        
        # Get table order
        self.get_table_order()
        
        # Define home position
        self.home_goal = self.create_home_goal()
        
        # Publish initial pose once
        self.publish_initial_pose()
        
        # Start processing goals
        self.send_next_goal()

    def get_table_order(self):
        try:
            table_name = int(input("Your order is from which table?\n Press 1 for Table 1\n Press 2 for Table 2\n Press 3 for Table 3\n "))
            
            if table_name == 1:
                self.goals = self.define_goals_table1()
            elif table_name == 2:
                self.goals = self.define_goals_table2()
            elif table_name == 3:
                self.goals = self.define_goals_table3()
            else:
                self.get_logger().error("Invalid table number. Please enter 1, 2, or 3.")
                rclpy.shutdown()
        except ValueError:
            self.get_logger().error("Invalid input! Please enter a valid number (1, 2, or 3).")
            rclpy.shutdown()

    def define_goals_table1(self):
        goals = []
        # Define Goal 1 - Kitchen
        goal1 = PoseStamped()
        goal1.header.frame_id = 'map'
        goal1.pose.position.x = 1.0
        goal1.pose.position.y = 7.0
        goal1.pose.position.z = 0.0
        goal1.pose.orientation.x = 0.0
        goal1.pose.orientation.y = 0.0
        goal1.pose.orientation.z = 0.0
        goal1.pose.orientation.w = 1.0
        goals.append(goal1)

        # Define Goal 2 - Table
        goal2 = PoseStamped()
        goal2.header.frame_id = 'map'
        goal2.pose.position.x = 5.5
        goal2.pose.position.y = 5.5
        goal2.pose.position.z = 0.0
        goal2.pose.orientation.x = 0.0
        goal2.pose.orientation.y = 0.0
        goal2.pose.orientation.z = 0.0
        goal2.pose.orientation.w = 1.0
        goals.append(goal2)

        # Define Goal 3 - Home
        goal3 = PoseStamped()
        goal3.header.frame_id = 'map'
        goal3.pose.position.x = 0.0
        goal3.pose.position.y = 0.0
        goal3.pose.position.z = 0.0
        goal3.pose.orientation.x = 0.0
        goal3.pose.orientation.y = 0.0
        goal3.pose.orientation.z = 0.0
        goal3.pose.orientation.w = 1.0
        goals.append(goal3)

        return goals

    def define_goals_table2(self):
        goals = []
        # Define Goal 1 - Kitchen
        goal1 = PoseStamped()
        goal1.header.frame_id = 'map'
        goal1.pose.position.x = 1.0
        goal1.pose.position.y = 7.0
        goal1.pose.position.z = 0.0
        goal1.pose.orientation.x = 0.0
        goal1.pose.orientation.y = 0.0
        goal1.pose.orientation.z = 0.0
        goal1.pose.orientation.w = 1.0
        goals.append(goal1)

        # Define Goal 2 - Table
        goal2 = PoseStamped()
        goal2.header.frame_id = 'map'
        goal2.pose.position.x = 5.5
        goal2.pose.position.y = 3.5
        goal2.pose.position.z = 0.0
        goal2.pose.orientation.x = 0.0
        goal2.pose.orientation.y = 0.0
        goal2.pose.orientation.z = 0.0
        goal2.pose.orientation.w = 1.0
        goals.append(goal2)

        # Define Goal 3 - Home
        goal3 = PoseStamped()
        goal3.header.frame_id = 'map'
        goal3.pose.position.x = 0.0
        goal3.pose.position.y = 0.0
        goal3.pose.position.z = 0.0
        goal3.pose.orientation.x = 0.0
        goal3.pose.orientation.y = 0.0
        goal3.pose.orientation.z = 0.0
        goal3.pose.orientation.w = 1.0
        goals.append(goal3)

        return goals

    def define_goals_table3(self):
        goals = []
        # Define Goal 1 - Kitchen
        goal1 = PoseStamped()
        goal1.header.frame_id = 'map'
        goal1.pose.position.x = 1.0
        goal1.pose.position.y = 7.0
        goal1.pose.position.z = 0.0
        goal1.pose.orientation.x = 0.0
        goal1.pose.orientation.y = 0.0
        goal1.pose.orientation.z = 0.0
        goal1.pose.orientation.w = 1.0
        goals.append(goal1)

        # Define Goal 2 - Table
        goal2 = PoseStamped()
        goal2.header.frame_id = 'map'
        goal2.pose.position.x = 5.5
        goal2.pose.position.y = 1.5
        goal2.pose.position.z = 0.0
        goal2.pose.orientation.x = 0.0
        goal2.pose.orientation.y = 0.0
        goal2.pose.orientation.z = 0.0
        goal2.pose.orientation.w = 1.0
        goals.append(goal2)

        # Define Goal 3 - Home
        goal3 = PoseStamped()
        goal3.header.frame_id = 'map'
        goal3.pose.position.x = 0.0
        goal3.pose.position.y = 0.0
        goal3.pose.position.z = 0.0
        goal3.pose.orientation.x = 0.0
        goal3.pose.orientation.y = 0.0
        goal3.pose.orientation.z = 0.0
        goal3.pose.orientation.w = 1.0
        goals.append(goal3)

        return goals

    def create_home_goal(self):
        home_goal = PoseStamped()
        home_goal.header.frame_id = 'map'
        home_goal.pose.position.x = 0.0
        home_goal.pose.position.y = 0.0
        home_goal.pose.position.z = 0.0
        home_goal.pose.orientation.x = 0.0
        home_goal.pose.orientation.y = 0.0
        home_goal.pose.orientation.z = 0.0
        home_goal.pose.orientation.w = 1.0
        return home_goal

    def publish_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header = Header()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'
        
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        
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
        
        # After reaching the goal, wait for confirmation
        self.waiting_for_confirmation = True
        self.start_confirmation_timer()

    def feedback_callback(self, feedback_msg):
        pass

    def handle_confirmation(self, request, response):
        self.get_logger().info('Confirmation service called.')
        if self.waiting_for_confirmation:
            self.waiting_for_confirmation = False
            if self.confirmation_timer is not None:
                self.confirmation_timer.cancel()
                self.confirmation_timer = None
            self.get_logger().info('Confirmation received. Proceeding to next goal.')
            self.current_goal_index +=1
            self.send_next_goal()
        else:
            self.get_logger().warn('Confirmation received, but not waiting for confirmation.')
        return response

    def start_confirmation_timer(self):
        self.get_logger().info(f'Waiting for confirmation for {self.timeout_duration} seconds...')
        self.confirmation_timer = self.create_timer(
            self.timeout_duration, 
            self.confirmation_timeout
        )
    
    def confirmation_timeout(self):
        if self.waiting_for_confirmation:
            self.get_logger().info('No confirmation received. Returning home.')
            self.waiting_for_confirmation = False
            self.confirmation_timer = None
            self.go_home()

    def go_home(self):
        self.get_logger().info('Navigating to home position.')
        # Send home goal
        home_goal_msg = NavigateToPose.Goal()
        home_goal_msg.pose = self.home_goal
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            home_goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.home_goal_response_callback)

    def home_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Home goal rejected.')
            rclpy.shutdown()
            return
        self.get_logger().info('Home goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.home_get_result_callback)

    def home_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Returned to home position.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SequentialNav()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

