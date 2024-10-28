#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mini_pupper_interfaces.srv import PlayMusic, StopMusic
from std_msgs.msg import String  # Import the message type
from .music_player import MusicPlayer
import os
from ament_index_python.packages import get_package_share_directory

class SoundPlayerNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_music_service')
        self.sound_file =''
        self.dance_config_sub = self.create_subscription(String, '/music_config', self.music_config_callback, 10)
        self.service = self.create_service(
            SetBool,
            'music_command',
            self.play_music_callback
        )
        self.is_playing = False
        self.music_player = MusicPlayer()
        self.play_service = self.create_service(
            PlayMusic,
            'play_music',
            self.play_music_callback
        )
        self.stop_service = self.create_service(
            StopMusic,
            'stop_music',
            self.stop_music_callback
        )

    def music_config_callback(self, msg):
        self.sound_file = msg.data

    def play_music_callback(self, request, response):
        file_path = self.get_valid_file_path(request.file_name)
        if file_path is not None:
            if self.music_player.playing:
                response.success = False
                response.message = 'Another music is being played.'
            else:
                self.music_player.start_music(file_path,
                                              request.start_second,
                                              request.duration)
                response.success = True
                response.message = 'Music started playing.'
                self.get_logger().info(f"playing music at {file_path}")

        else:
            response.success = False
            response.message = f'File {request.file_name} is not found.'
        return response

    def stop_music_callback(self, request, response):
        if self.music_player.playing:
            self.music_player.stop_music()
            response.success = True
            response.message = 'Music playback stopped.'
        else:
            response.success = False
            response.message = 'No music is being played.'
        return response
    
    def get_valid_file_path(self, file_name):
        #package_name = 'mini_pupper_music'
        #package_path = get_package_share_directory(package_name)
        music_folder = os.getenv('MUSIC_FOLDER')  # Get the MUSIC_FOLDER environment variable
        if music_folder is None:
            self.get_logger().error('MUSIC_FOLDER environment variable is not set.')
            return None

        file_path = os.path.join(music_folder, file_name)
        self.get_logger().info(f"music file path is {file_path}!")
        if os.path.isfile(file_path):
            return file_path
        else:
            return None

    def destroy_node(self):
        self.music_player.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    sound_player_node = SoundPlayerNode()
    rclpy.spin(sound_player_node)
    sound_player_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
