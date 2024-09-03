import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from epuck_driver_interfaces.action import CollectSoundData

import numpy as np
import time
import threading

from epuck_driver_interfaces.msg import RecordActivation
from epuck_driver_interfaces.msg import RecordResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32

class EpuckSoundDataCollector(Node):

    def __init__(self):
        super().__init__('sound_data_collection_action_server')
        self._action_servers = ActionServer(self, CollectSoundData, 'sound_data_collection', self.execute_goal_callback)

        self.data_cb_group = MutuallyExclusiveCallbackGroup()

        self.robot_ids  = self.declare_parameter("robot_names", ["epuck1", "epuck2", "epuck3"]).get_parameter_value().string_array_value
        self.num_microphones = self.declare_parameter("num_microphones", 3).get_parameter_value().integer_value

        self.microphone_buffers = {id:{k:[] for k in range(self.num_microphones)} for id in self.robot_ids}
        self.microphone_data_subscribers = [[self.create_subscription(Float32, 
                                                                      "{}/microphone_{}".format(id, k), 
                                                                      self.create_microphone_data_callback(id, k), 5, callback_group=self.data_cb_group) for k in range(self.num_microphones)] for id in self.robot_ids]
        self.sound_collection_done = threading.Event()
        self.record = False

    def record_done_callback(self):

        print("Received")
        self.record = False

        for robot in self.robot_ids:
            for i in range(self.num_microphones):
                if len(self.microphone_buffers[robot][i]) == 0:
                    self.get_logger().info("No data received from robot {}, microphone {}".format(robot, i))

        self.sound_collection_done.set()
        
    def execute_goal_callback(self, goal_handle):


        recording_time = goal_handle.request.recording_time
        average_over_robot_mics = goal_handle.request.avg_microphone_amplitudes


        self.sound_collection_done.clear()
        self.clear_buffers()
        self.record = True
        timer = self.create_timer(recording_time, self.record_done_callback, callback_group=self.data_cb_group)

        rate = self.create_rate(10)

        while not self.sound_collection_done.is_set():
            rate.sleep()
        
        timer.cancel()
        timer.destroy()
        collected_sound_data = self.get_formated_data_from_buffer(average_over_robot_mics)

        print("Done: {}".format(collected_sound_data))

        goal_handle.succeed()

        result = CollectSoundData.Result()

        if average_over_robot_mics:
            result.avg_dbs_robot_0 = [collected_sound_data[0]] # Average dBs of robot 0 over recording time

            if len(self.robot_ids) > 1:
                result.avg_dbs_robot_1 = [collected_sound_data[1]] # Average dBs of robot 1 over recording time
            
            if len(self.robot_ids) > 2:
                result.avg_dbs_robot_2 = [collected_sound_data[2]] # Average dBs of robot 2 over recording time

        else:
            result.avg_dbs_robot_0 = list(collected_sound_data[0]) # Average dBs for microphones of robot 0 over recording time

            if len(self.robot_ids) > 1:
                result.avg_dbs_robot_1 = list(collected_sound_data[1]) # Average dBs for microphones of robot 1 over recording time

            if len(self.robot_ids) > 2:
                result.avg_dbs_robot_2 = list(collected_sound_data[2]) # Average dBs for microphones of robot 2 over recording time

        return result
        
    
    def create_microphone_data_callback(self, robot_id, mic_id):
         
        def callback(msg):
            if self.record:
                self.microphone_buffers[robot_id][mic_id].append(msg.data)
        
        return callback
    
    def clear_buffers(self):
         
        for id in self.robot_ids:
            for k in range(self.num_microphones):
                self.microphone_buffers[id][k].clear()
    
    def get_formated_data_from_buffer(self, average_over_robot_mics):

        robot_audio_data = []
        min_len = 10*4

        #for i, id in enumerate(self.robot_ids):

        #    robot_audio_data.append([])
        #    for k in range(self.num_microphones):
                
        #        mic_data = self.microphone_buffers[id][k]
        #        if len(mic_data) == 0:
        #            continue

        #        robot_audio_data[i].append(mic_data)
        #        min_len = min(min_len, len(mic_data))

        for robot in self.robot_ids:
            for k in range(self.num_microphones):
                min_len = min(min_len, len(self.microphone_buffers[robot][k]))
        

        robot_audio_data = np.array([[self.microphone_buffers[robot][k][:min_len] for k in range(self.num_microphones)] for robot in self.robot_ids])
        avg_axis = (1, 2) if average_over_robot_mics else 2
        averaged_result = np.average(robot_audio_data, axis=avg_axis)

        print("Averaged Result {}; average over robot: {}".format(averaged_result, average_over_robot_mics))
        return averaged_result

    
    
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