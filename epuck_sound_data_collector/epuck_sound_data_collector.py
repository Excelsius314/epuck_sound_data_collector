import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from epuck_driver_interfaces.action import CollectSoundData

import time
import threading
from epuck_driver_interfaces.msg import RecordActivation
from epuck_driver_interfaces.msg import RecordResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
class EpuckSoundDataCollector(Node):

    def __init__(self):
        super().__init__('sound_data_collection_action_server')
        self._action_servers = ActionServer(self, CollectSoundData, 'sound_data_collection', self.execute_goal_callback)

        #self.robot_ids = [0, 1, 2]
        #self.num_microphones = 4

        self.data_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.record_activation_publisher = self.create_publisher(RecordActivation, "audio_collection_activation", 1)
        self.record_result_subscriver = self.create_subscription(RecordResult, "record_result", 
                                                                 self.record_result_received, 1, 
                                                                 callback_group=self.data_cb_group)

        self.collected_sound_data = []
        self.sound_collection_done = threading.Event()
        self.declare_parameter('average_mic_amplitudes_per_robot', True)


    def record_result_received(self, result_msg):

        print("Received")
        self.collected_sound_data = result_msg.averaged_audio_levels
        self.sound_collection_done.set()
        
    def execute_goal_callback(self, goal_handle):

        self.sound_collection_done.clear()

        avg_mic_ampls = self.get_parameter('average_mic_amplitudes_per_robot').get_parameter_value().bool_value

        record_activation_msg = RecordActivation()
        record_activation_msg.recording_time = goal_handle.request.recording_time
        record_activation_msg.average_over_robot = avg_mic_ampls
        self.record_activation_publisher.publish(record_activation_msg)
        
        rate = self.create_rate(10)

        while not self.sound_collection_done.is_set():
            print("Checking")
            rclpy.spin_once
            time.sleep(0.1)
        
        print("Done: {}".format(self.collected_sound_data))

        goal_handle.succeed()

        result = CollectSoundData.Result()

        if avg_mic_ampls:
            result.avg_dbs_robot_0 = [self.collected_sound_data[0]] # Average dBs of robot 0 over recording time
            #result.avg_dbs_robot_1 = [self.collected_sound_data[1]] # Average dBs of robot 1 over recording time
            #result.avg_dbs_robot_2 = [self.collected_sound_data[2]] # Average dBs of robot 2 over recording time

        else:       
            result.avg_dbs_robot_0 = self.collected_sound_data[0] # Average dBs for microphones of robot 0 over recording time
            #result.avg_dbs_robot_1 = self.collected_sound_data[1] # Average dBs for microphones of robot 1 over recording time
            #result.avg_dbs_robot_2 = self.collected_sound_data[2] # Average dBs for microphones of robot 2 over recording time

        return result
        


    
    
def main(args=None):
    rclpy.init(args=args)

    sound_collector_action_server = EpuckSoundDataCollector()

    executor = MultiThreadedExecutor()
    executor.add_node(sound_collector_action_server)

    try:
        sound_collector_action_server.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        sound_collector_action_server.get_logger().info('Keyboard interrupt, shutting down.\n')
    
    sound_collector_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()