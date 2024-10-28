#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String
import sounddevice as sd
import soundfile as sf
import threading
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
            self.play_sound_callback
        )
        self.is_playing = False
        self.playback_thread = None
        self.lock = threading.Lock()

    def music_config_callback(self, msg):
        self.sound_file = msg.data

    def play_sound_callback(self, request, response):
        if request.data:
            with self.lock:
                if not self.is_playing:
                    if (self.sound_file==''):
                        file_name = 'robot1.wav'
                    else:
                        file_name = self.sound_file
                        
                    sound_file = self.get_valid_file_path(file_name)
                    response.success = True
                    response.message = 'Sound playback started.'
                    self.music_player.start_music(sound_file, request.start_second, request.duration)
                else:
                    response.success = False
                    response.message = 'Sound is already playing.'
        else:
            with self.lock:
                if self.is_playing:
                    self.stop_sound()
                    response.success = True
                    response.message = 'Sound playback stopped.'
                else:
                    response.success = False
                    response.message = 'No sound is currently playing.'
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

    def stop_sound(self):
        self.is_playing = False
        if self.playback_thread is not None:
            self.playback_thread.join(timeout=0)
            if self.playback_thread.is_alive():
                # If the thread is still running, stop the sound playback
                sd.stop()

def main(args=None):
    rclpy.init(args=args)
    sound_player_node = SoundPlayerNode()
    rclpy.spin(sound_player_node)
    sound_player_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
