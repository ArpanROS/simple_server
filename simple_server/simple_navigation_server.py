import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose
import random
import time

class SimpleNavigationServer(Node):
    def __init__(self):
        super().__init__('simple_navigation_server')
        
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )
        
        self.get_logger().info('Simple Navigation Action Server has been started')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal request...')
        
        
        feedback_msg = NavigateToPose.Feedback()
        
        for _ in range(3):
            if not goal_handle.is_active:
                goal_handle.abort()
                return NavigateToPose.Result()
                
            time.sleep(1)
            
            feedback_msg.current_pose = goal_handle.request.pose
            goal_handle.publish_feedback(feedback_msg)
            
        success = random.random() < 0.8
        
        if success:
            goal_handle.succeed()
            self.get_logger().info('Goal_Reached')
        else:
            goal_handle.abort()
            self.get_logger().info('Goal_Failed')
            
        return NavigateToPose.Result()

def main(args=None):
    rclpy.init(args=args)
    
    navigation_server = SimpleNavigationServer()
    
    try:
        rclpy.spin(navigation_server)
    except KeyboardInterrupt:
        pass
    
    navigation_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()