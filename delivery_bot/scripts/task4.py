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
        super().__init__('task4node')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._confirmation_service = self.create_service(Empty, '/confirmation', self.handle_confirmation)
        self._cancellation_service = self.create_service(Empty, '/cancellation', self.handle_cancellation)

        self.goals = []
        self.current_goal_index = 0
        self.waiting_for_confirmation = False
        self.confirmation_timer = None
        self.timeout_duration = 10.0

        self.get_table_order()
        self.home_goal = self.create_home_goal()
        self.kitchen_goal = self.create_kitchen_goal()
        self.publish_initial_pose()
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
        goal1 = PoseStamped()
        goal1.header.frame_id = 'map'
        goal1.pose.position.x = 1.0
        goal1.pose.position.y = 7.0
        goal1.pose.orientation.w = 1.0
        goals.append(goal1)

        goal2 = PoseStamped()
        goal2.header.frame_id = 'map'
        goal2.pose.position.x = 5.5
        goal2.pose.position.y = 5.5
        goal2.pose.orientation.w = 1.0
        goals.append(goal2)

        return goals

    def define_goals_table2(self):
        goals = []
        goal1 = PoseStamped()
        goal1.header.frame_id = 'map'
        goal1.pose.position.x = 1.0
        goal1.pose.position.y = 7.0
        goal1.pose.orientation.w = 1.0
        goals.append(goal1)

        goal2 = PoseStamped()
        goal2.header.frame_id = 'map'
        goal2.pose.position.x = 5.5
        goal2.pose.position.y = 3.5
        goal2.pose.orientation.w = 1.0
        goals.append(goal2)

        return goals

    def define_goals_table3(self):
        goals = []
        goal1 = PoseStamped()
        goal1.header.frame_id = 'map'
        goal1.pose.position.x = 1.0
        goal1.pose.position.y = 7.0
        goal1.pose.orientation.w = 1.0
        goals.append(goal1)

        goal2 = PoseStamped()
        goal2.header.frame_id = 'map'
        goal2.pose.position.x = 5.5
        goal2.pose.position.y = 1.5
        goal2.pose.orientation.w = 1.0
        goals.append(goal2)

        return goals

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
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.7071068
        initial_pose.pose.pose.orientation.w = 0.7071068
        self._initialpose_publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published.')

    def send_next_goal(self):
        if self.current_goal_index < len(self.goals):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.goals[self.current_goal_index]
            self._action_client.wait_for_server()
            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info('All goals have been processed.')
            rclpy.shutdown()

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = self.goal_handle.get_result_async()
       
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().status
        if result == 4: # if result is sucessfull
            self.get_logger().info(f'Goal {self.current_goal_index + 1} result received.')
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

            if self.current_goal_index == len(self.goals) - 1:  # At the last goal (table)
                self.get_logger().info('Confirmation received at the table. Navigating to home.')
                self.go_home()
                
            else:
                self.get_logger().info('Confirmation received. Proceeding to next goal.')
                self.current_goal_index += 1
                self.send_next_goal()
            return response
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

            if self.current_goal_index == 1:  # At the table
                self.get_logger().info('Navigating to kitchen before home.')
                self.navigate_to_kitchen()
            else:
                self.get_logger().info('Navigating directly to home.')
                self.go_home()

    def navigate_to_kitchen(self):
        kitchen_goal_msg = NavigateToPose.Goal()
        kitchen_goal_msg.pose = self.kitchen_goal
        self._action_client.wait_for_server()
        self._send_goal_future1 = self._action_client.send_goal_async(
            kitchen_goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future1.add_done_callback(self.kitchen_goal_response_callback)

    def kitchen_goal_response_callback(self, future1):
        goal_handle2 = future1.result()
        if not goal_handle2.accepted:
            self.get_logger().info('Kitchen goal rejected.')
            self.go_home()
            return

        self.get_logger().info('Kitchen goal accepted.')
        self._get_result_future1 = goal_handle2.get_result_async()
        self._get_result_future1.add_done_callback(self.kitchen_get_result_callback)

    def kitchen_get_result_callback(self, future):
        self.get_logger().info('Reached kitchen. Navigating to home.')
        self.go_home()

    def go_home(self):
        home_goal_msg = NavigateToPose.Goal()
        home_goal_msg.pose = self.home_goal
        self._action_client.wait_for_server()
        self._send_goal_future2 = self._action_client.send_goal_async(
            home_goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future2.add_done_callback(self.home_goal_response_callback)

    def home_goal_response_callback(self, future2):
        goal_handle3 = future2.result()
        if not goal_handle3.accepted:
            self.get_logger().info('Home goal rejected.')
            rclpy.shutdown()
            return
        self.get_logger().info('Home goal accepted.')
        self._get_result_future2 = goal_handle3.get_result_async()
        self._get_result_future2.add_done_callback(self.home_get_result_callback)

    def home_get_result_callback(self, future):
        self.get_logger().info('Returned to home position.')
        rclpy.shutdown()

    def handle_cancellation(self, request, response):
        self.waiting_for_confirmation = False
        self.confirmation_timer = None
        self._action_client._cancel_goal_async(self.goal_handle)
        self.get_logger().info('Cancellation service called.')
        if self.current_goal_index == 1:
            self.navigate_to_kitchen()
      
        else:
            self.go_home() 
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SequentialNav()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

