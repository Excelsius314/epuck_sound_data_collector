import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from epuck_driver_interfaces.action import CollectSoundData

import time
import threading
from epuck_driver_interfaces.msg import RecordActivation
from epuck_driver_interfaces.msg import RecordResult

class EpuckSoundDataCollector(Node):

    def __init__(self):
        super().__init__('sound_data_collection_action_server')
        self._action_servers = ActionServer(
            self,
            CollectSoundData,
            'sound_data_collection',
            self.execute_callback)

        
        #self.robot_ids = [0, 1, 2]
        #self.num_microphones = 4
        
        self.record_activation_publisher = self.create_publisher(RecordActivation, "audio_collection_activation", 1)
        self.record_result_subscriver = self.create_publisher(RecordResult, "record_result", 1)

        self.collected_sound_data = None
        self.sound_collection_done = threading.Event()


    def record_result_ceveid(self, result_msg)

        self.collected_sound_data = result_msg.averaged_audio_levels
        self.sound_collection_done.set()
        
    def execute_goal_callback(self, goal_handle):

        self.sound_collection_done.clear()

        record_activation_msg = RecordActivation()
        record_activation_msg.recording_time = goal_handle.recording_time
        self.record_activation_publisher.publish(record_activation_msg)
        
        rate = rospy.Rate(10)

        while not self.sound_collection_done.is_set():
            rate.sleep()

        goal_handle.succeed()

        result = CollectSoundData.Result()
        result.avg_dbs_robot_0 = self.collected_sound_data[0] # Average dBs for microphones of robot 0 over recording time
        result.avg_dbs_robot_1 = self.collected_sound_data[1] # Average dBs for microphones of robot 1 over recording time
f       result.avg_dbs_robot_2 = self.collected_sound_data[2] # Average dBs for microphones of robot 2 over recording time

        return result
        


    
    
def main(args=None):
    rclpy.init(args=args)

    sound_collector_action_server = EpuckSoundDataCollector()

    rclpy.spin(sound_collector_action_server)


if __name__ == '__main__':
    main()