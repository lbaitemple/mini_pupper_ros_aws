#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mini_pupper_interfaces.srv import PlayMusic, StopMusic
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from .math_operations import *
import math, sys, os
import time
import asyncio
from .util import parse_movement_string

class MiniPupperMusicClientAsync(Node):

    def __init__(self):
        super().__init__('mini_pupper_music_client_async')
        self.play_music_cli = self.create_client(PlayMusic, 'play_music')
        self.stop_music_cli = self.create_client(StopMusic, 'stop_music')
        while not self.play_music_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service music not available, waiting again...')

    
    def send_play_music_request(self, file_name, start_second):
        req = PlayMusic.Request()
        req.file_name = file_name
        req.start_second = start_second
        self.play_music_cli.call_async(req)  # Fire-and-forget style communication

    def send_stop_music_request(self):
        req = StopMusic.Request()
        self.stop_music_cli.call_async(req)  # Fire-and-forget style communication

        

class DanceDemo(Node):
    def __init__(self):
        super().__init__('dance_demo')
        self.r = self.create_rate(100)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.declare_parameter('dance_config_path', rclpy.Parameter.Type.STRING) 

        self.dance_config_path = self.get_parameter('dance_config_path').get_parameter_value().string_value
        self.get_logger().info(self.dance_config_path)
        self.get_logger().info(str(self.dance_config_path))
        self.dance_config_name = ' '
        self.commands = []
        self.ready_to_dance = 0
        self.dance_config_sub = self.create_subscription(String, '/dance_config', self.dance_config_callback, 10)
        
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 100)
        self.music_publisher = self.create_publisher(String, '/music_config', 100)
        self.music_client= MiniPupperMusicClientAsync()        

        #self.pose_publisher = self.create_publisher(Pose, 'target_body_pose', 100)
        self.pose_publisher = self.create_publisher(Pose, 'reference_body_pose', 100)

        
    def dance_config_callback(self, msg):
        self.dance_config_name = msg.data
        self.dance_config = __import__(self.dance_config_name)
        self.commands = self.dance_config.dance_commands
        self.get_logger().info('executing command: .... ' )
        self.commands.insert(0, 'stop:0.0:0.5')
        self.commands.append('stop:0.0:0.5')
        self.ready_to_dance = 1
        self.dance()

    def dance(self):
 
        while (self.dance_config_name == ' ' or not self.ready_to_dance) and rclpy.ok():
            self.r.sleep()

        for readcommand in self.commands:
            velocity_cmd = Twist()
            pose_cmd = Pose()
            self.roll = 0
            self.pitch = 0
            self.yaw = 0
            
            # parse the string for the readin commands
            self.get_logger().info(readcommand)
            result = parse_movement_string(readcommand)
            # get the value from each key
            command = result['action']
            value = result['value']
            interval = result['interval']
            self.dance_config.interval_time = interval


            if rclpy.ok():
                if command == 'move_forward':
                    self.get_logger().info('executing command: ' + str(command))
                    velocity_cmd.linear.x = value
                    self.velocity_publisher.publish(velocity_cmd)
                    time.sleep(self.dance_config.interval_time)

                elif command == 'move_backward':
                    self.get_logger().info('executing command: ' + str(command))
                    if (value <0 ):
                        velocity_cmd.linear.x = value
                    self.velocity_publisher.publish(velocity_cmd)
                    time.sleep(self.dance_config.interval_time)
                elif(command == 'move_left' ):
                    velocity_cmd.linear.y = value
                    self.velocity_publisher.publish(velocity_cmd)
                    self.get_logger().info('Publishing: "%s"' % command)
                    time.sleep(self.dance_config.interval_time)

                elif(command == 'move_right' ):
                    if (value <0 ): # make sure the value is negative
                        velocity_cmd.linear.y = value
                    self.velocity_publisher.publish(velocity_cmd)
                    self.get_logger().info('Publishing: "%s"' % command)
                    time.sleep(self.dance_config.interval_time)
                    
                elif(command == 'turn_left' ):
                    velocity_cmd.angular.z = value
                    self.velocity_publisher.publish(velocity_cmd)
                    self.get_logger().info('Publishing: "%s"' % command)
                    time.sleep(self.dance_config.interval_time)
 
                elif(command == 'turn_right' ):
                    if (value <0 ):
                        velocity_cmd.angular.z = value
                    self.velocity_publisher.publish(velocity_cmd)
                    self.get_logger().info('Publishing: "%s"' % command)
                    time.sleep(self.dance_config.interval_time)
        
                elif(command == 'look_up' ):
                    #pose_cmd.orientation.x, pose_cmd.orientation.y, pose_cmd.orientation.z, pose_cmd.orientation.w = quaternion_from_euler(0.0, -0.3, 0.0)
                    if (value <0 ):
                        pose_cmd.orientation.x, pose_cmd.orientation.y, pose_cmd.orientation.z, pose_cmd.orientation.w = quaternion_from_euler(0.0, value, 0.0)
                    self.pose_publisher.publish(pose_cmd)
                    self.get_logger().info('Publishing: "%s"' % command)
                    time.sleep(self.dance_config.interval_time)
        
                elif(command == 'look_down' ):
                   # pose_cmd.orientation.x, pose_cmd.orientation.y, pose_cmd.orientation.z, pose_cmd.orientation.w = quaternion_from_euler(0.0, 0.3, 0.0)
                    pose_cmd.orientation.x, pose_cmd.orientation.y, pose_cmd.orientation.z, pose_cmd.orientation.w = quaternion_from_euler(0.0, value, 0.0)
                    self.pose_publisher.publish(pose_cmd)
                    self.get_logger().info('Publishing: "%s"' % command)
                    time.sleep(self.dance_config.interval_time)
        
                elif(command == 'look_left' ):
#                    pose_cmd.orientation.x, pose_cmd.orientation.y, pose_cmd.orientation.z, pose_cmd.orientation.w = quaternion_from_euler(0.0, 0.0, 0.3)
                    pose_cmd.orientation.x, pose_cmd.orientation.y, pose_cmd.orientation.z, pose_cmd.orientation.w = quaternion_from_euler(0.0, 0.0, value)
                    self.pose_publisher.publish(pose_cmd)
                    self.get_logger().info('Publishing: "%s"' % command)
                    time.sleep(self.dance_config.interval_time)
        
                elif(command == 'look_right' ):
#                    pose_cmd.orientation.x, pose_cmd.orientation.y, pose_cmd.orientation.z, pose_cmd.orientation.w = quaternion_from_euler(0.0, 0.0, -0.3)
                    if (value <0 ):
                        pose_cmd.orientation.x, pose_cmd.orientation.y, pose_cmd.orientation.z, pose_cmd.orientation.w = quaternion_from_euler(0.0, 0.0, value)
                    self.pose_publisher.publish(pose_cmd)
                    self.get_logger().info('Publishing: "%s"' % command)
                    time.sleep(self.dance_config.interval_time)
        
                elif(command == 'look_middle' ):
#                    pose_cmd.orientation.x, pose_cmd.orientation.y, pose_cmd.orientation.z, pose_cmd.orientation.w = quaternion_from_euler(0.0, 0.0, 0.0)
                    pose_cmd.orientation.x, pose_cmd.orientation.y, pose_cmd.orientation.z, pose_cmd.orientation.w = quaternion_from_euler(0.0, 0.0, 0.0)
                    self.pose_publisher.publish(pose_cmd)
                    self.get_logger().info('Publishing: "%s"' % command)
                    time.sleep(self.dance_config.interval_time)
        
                elif(command == 'stay' ):
#                    time.sleep(self.interval) # do nothing
                    time.sleep(self.dance_config.interval_time)
                        
                elif(command == 'stop' ):
                    velocity_cmd.linear.x = 0.0
                    velocity_cmd.linear.y = 0.0
                    velocity_cmd.linear.z = 0.0
                    self.velocity_publisher.publish(velocity_cmd)
                    pose_cmd.orientation.x, pose_cmd.orientation.y, pose_cmd.orientation.z, pose_cmd.orientation.w = quaternion_from_euler(0.0, 0.0, 0.0)
                    self.pose_publisher.publish(pose_cmd)
                    self.get_logger().info('Publishing: "%s"' % command)
                    time.sleep(self.dance_config.interval_time)
                    
                elif (command == 'music' ):
                        # publish the music topic, call service to turn on/off music based on interval value
                    if (value):
                        msg = String()
                        msg.data = value
                        self.music_publisher.publish(msg)

                        response = self.music_client.send_play_music_request(value, 0)
                        if(response.success == True):
                            self.music_client.get_logger().info('Command Executed!')
                    else:
                       self.music_client.send_stop_music_request() 

            
                   # await asyncio.get_event_loop().run_until_complete(self.music_callback(interval))
                   # await asyncio.run(self.music_callback(interval))
                    #self.music_client.send_stop_music_request()
                        # call service to turn off music
                elif (command == 'volume' ):
                    os.system("amixer -c 0 sset 'Headphone' {}%".format(value))                        
 
                else:
                    self.get_logger().warn('wrong command: ' + str(command))

                
                velocity_cmd = Twist()
                self.velocity_publisher.publish(velocity_cmd)
                time.sleep(self.dance_config.interval_time)


def main(args=None):
    rclpy.init(args=args)
    lets_dance = DanceDemo()
    rclpy.spin(lets_dance)
    lets_dance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
