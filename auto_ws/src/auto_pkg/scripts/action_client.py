#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from clucky_interfaces.action import AutoCommand

class ActionClientNode(Node):
    def __init__(self):
        super().__init__("auto_action_client")
        # ActionClient(node, action_type, action_name *must be same as server)
        self.auto_action_client = ActionClient(self, AutoCommand, "auto_command") 
        
    def send_goal(self, target_number, period):
        # Wait for action server
        self.auto_action_client.wait_for_server() # handle timeout
        
        # Create goal request
        goal = AutoCommand.Goal()
        goal.target_number = target_number
        goal.period = period
        
        # Send goal request
        self.get_logger().info("Sending goal from macula...")
        self.auto_action_client. \
            send_goal_async(goal). \
                add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        self.goal_handle_:ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
            
    def goal_result_callback(self, future):
        result = future.result().result # final_result
        self.get_logger().info(f"Result: " + str(result.final_result))
            
        
def main(args=None):
    rclpy.init(args=args)
    node = ActionClientNode()
    node.send_goal(4, 1)
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
