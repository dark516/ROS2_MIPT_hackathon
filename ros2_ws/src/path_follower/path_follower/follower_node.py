#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, PoseArray
from nav_msgs.msg import Path
import math
import numpy as np
import time

# States
STATE_IDLE = 0
STATE_FOLLOW = 1
STATE_RECOVER_BACK = 2
STATE_RECOVER_TURN = 3
STATE_RECOVER_PUSH = 4

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        
        self.current_pose = None
        self.enemy_pose = None
        self.path = []
        self.obstacles = []
        
        # State
        self.state = STATE_IDLE
        self.recovery_start_time = 0.0
        
        # Recovery params
        self.stuck_check_time = 0.0
        self.last_moving_pose = None
        self.stuck_timeout = 2.0 # If not moved far in 2s while trying to, trigger recovery
        
        # Subs/Pubs
        self.sub_pose = self.create_subscription(Pose2D, '/robot/pose', self.pose_cb, 10)
        self.sub_path = self.create_subscription(Path, '/path', self.path_cb, 10)
        self.sub_obs = self.create_subscription(PoseArray, '/obstacles', self.obs_cb, 10)
        self.sub_enemy = self.create_subscription(Pose2D, '/enemy/pose', self.enemy_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Path Follower Started.")

    def pose_cb(self, msg): self.current_pose = msg
    def obs_cb(self, msg): self.obstacles = msg.poses
    def enemy_cb(self, msg): self.enemy_pose = msg
    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if self.path:
            self.state = STATE_FOLLOW
            self.last_moving_pose = self.current_pose
            self.stuck_check_time = time.time()

    def stop(self):
        self.pub_cmd.publish(Twist())

    def check_enemy_safety(self):
        if not self.current_pose or not self.enemy_pose: return True
        dist = math.hypot(self.current_pose.x - self.enemy_pose.x, 
                          self.current_pose.y - self.enemy_pose.y)
        if dist < 0.35: # Too close!
            return False
        return True

    def check_stuck_condition(self):
        # If trying to follow but not moving
        if self.state == STATE_FOLLOW and self.current_pose and self.last_moving_pose:
            dist_moved = math.hypot(self.current_pose.x - self.last_moving_pose.x,
                                    self.current_pose.y - self.last_moving_pose.y)
            
            if dist_moved > 0.1:
                # Reset stuck timer if moved enough
                self.last_moving_pose = self.current_pose
                self.stuck_check_time = time.time()
            elif (time.time() - self.stuck_check_time) > self.stuck_timeout:
                return True
        return False
        
    def get_cmd_for_follow(self):
        if not self.path or not self.current_pose: return Twist()
        
        # Lookahead
        lookahead = 0.3
        target = self.path[-1]
        
        for pt in self.path:
            d = math.hypot(pt[0] - self.current_pose.x, pt[1] - self.current_pose.y)
            if d > lookahead:
                target = pt
                break
                
        dx = target[0] - self.current_pose.x
        dy = target[1] - self.current_pose.y
        target_angle = math.atan2(dy, dx)
        
        angle_diff = target_angle - self.current_pose.theta
        while angle_diff > math.pi: angle_diff -= 2*math.pi
        while angle_diff < -math.pi: angle_diff += 2*math.pi
        
        cmd = Twist()
        if abs(angle_diff) > 0.5:
            cmd.angular.z = 1.5 * np.sign(angle_diff)
        else:
            cmd.angular.z = 2.0 * angle_diff
            cmd.linear.x = 0.4
            
        # Check if reached goal (end of path)
        end_pt = self.path[-1]
        d_end = math.hypot(end_pt[0] - self.current_pose.x, end_pt[1] - self.current_pose.y)
        if d_end < 0.1:
            cmd = Twist() # Stop
            
        return cmd

    def control_loop(self):
        if not self.current_pose: return

        # Safety Override
        if not self.check_enemy_safety():
            self.get_logger().warn("Enemy too close! Emergency Stop.")
            self.stop()
            self.state = STATE_IDLE # Reset state or wait?
            return

        cmd = Twist()
        
        # State Machine
        if self.state == STATE_FOLLOW:
            if self.check_stuck_condition():
                self.get_logger().warn("Stuck detected! Starting Recovery: BACK UP")
                self.state = STATE_RECOVER_BACK
                self.recovery_start_time = time.time()
            else:
                cmd = self.get_cmd_for_follow()
                
        elif self.state == STATE_RECOVER_BACK:
            # Move back for 1.5s
            if (time.time() - self.recovery_start_time) < 1.5:
                cmd.linear.x = -0.2
            else:
                # Transition to Turn
                self.state = STATE_RECOVER_TURN
                self.recovery_start_time = time.time()
                self.get_logger().info("Recovery: TURN")
                
        elif self.state == STATE_RECOVER_TURN:
            # Turn for 1.0s
            if (time.time() - self.recovery_start_time) < 1.0:
                cmd.angular.z = 1.0
            else:
                # Try following again or escalate to Push
                # Let's escalate to push if we still haven't cleared the issue?
                # Actually, simpler to try Pushing now as "last resort" implies trying other things first.
                # But here we just assume the sequence Back -> Turn -> Push -> Resume
                self.state = STATE_RECOVER_PUSH
                self.recovery_start_time = time.time()
                self.get_logger().info("Recovery: PUSH (Last Resort)")
                
        elif self.state == STATE_RECOVER_PUSH:
            # Force forward for 1.0s
            if (time.time() - self.recovery_start_time) < 1.0:
                cmd.linear.x = 0.5 # Strong push
            else:
                # Resume normal operation
                self.state = STATE_FOLLOW
                self.last_moving_pose = self.current_pose
                self.stuck_check_time = time.time()
                self.get_logger().info("Recovery Complete. Resuming Follow.")
                
        elif self.state == STATE_IDLE:
            # Do nothing, wait for path
            pass

        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
