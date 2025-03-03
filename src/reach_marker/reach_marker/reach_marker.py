import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import numpy as np
from geometry_msgs.msg import Quaternion
import math
from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import tiago as robot

#for markerDetection
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PoseGoalExample(Node):
    def __init__(self, TagPose):
        super().__init__("reach_marker")
        self.__callback_group = ReentrantCallbackGroup()

        self.__synchronous = True

        self.__cartesian = False
        self.__cartesian_max_step = 0.0025
        self.__cartesian_fraction_threshold = 0.0

        self.__moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.__callback_group,
        )

        self.__moveit2.planner_id = "RRTConnectkConfigDefault" 
        self.__moveit2.max_velocity = 0.5
        self.__moveit2.max_acceleration = 0.5
        self.__moveit2.cartesian_avoid_collisions = False
        self.__moveit2.cartesian_jump_threshold = 0.0

        self.tagPose = TagPose

        self.timer = self.create_timer(10.0, self.timer_callback)
        

        
        

    def timer_callback(self):  
        #Orientation data collection
        tagQuat = self.tagPose.transform.rotation.x,self.tagPose.transform.rotation.y,self.tagPose.transform.rotation.z,self.tagPose.transform.rotation.w
        #position data collection
        position = [self.tagPose.transform.translation.x-0.224,self.tagPose.transform.translation.y,self.tagPose.transform.translation.z]
        #arm movement
        self.get_logger().info("Moving arm to pose")
        self.__move_arm_to_pose(position=position, orientation=tagQuat)

    def __move_arm_to_pose(self, position:list[float, float, float], orientation:list[float, float, float, float]):
        """
        Move the arm to a given pose relative of base footprint

        Args:
        - position: 3D position (x,y,z) relative of base footprint
        - orientation: quaternion orientation (x, y, z, w) relative of base footprint
        """
        self.__moveit2.move_to_pose(
            position=position,
            quat_xyzw=orientation,
            cartesian=self.__cartesian,
            cartesian_max_step=self.__cartesian_max_step,
            cartesian_fraction_threshold=self.__cartesian_fraction_threshold,
        )
        
        if self.__synchronous: 
            self.__moveit2.wait_until_executed()

class FrameListener(Node):

    def __init__(self):
        super().__init__('test_tf2_frame_listener')

        self.source_frame = "tag36h11:3"
        self.target_frame = "base_footprint"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.poseOfTag = None

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):

        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())

            self.poseOfTag = t
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            return

    def getPoseOfTag(self):
        return self.poseOfTag


def main(args=None):
    rclpy.init(args=args)   
    try:
        executor = MultiThreadedExecutor()
        detector = FrameListener()
       
        executor.add_node(detector)

        start_time = time.time()
        timeout = 10  # Timeout in seconds
        while time.time() - start_time < timeout:
            rclpy.spin_once(detector)  # Spin for 0.1 seconds
            tagPose = detector.getPoseOfTag()
            if tagPose:
                break  # Exit loop if tagPose is found

        if tagPose:
            
            posegoal = PoseGoalExample(tagPose)
            executor.add_node(posegoal)
        else:
            print('tagPose ist null, value:',tagPose)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            posegoal.destroy_node()
            spawner.destroy_node()
            detector.destroy_node()
    finally:
        rclpy.shutdown()
if __name__ == "__main__":
    main()