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
#for action server
from rclpy.action import ActionServer
from pick_and_place_interfaces.action import PickAndPlace
from rclpy.action import ActionClient
class PoseGoalExample(Node):
    def __init__(self, TagPose):
        super().__init__("reach_marker")

        self.__callback_group = ReentrantCallbackGroup()


        #gripper stuff
        self.__gripper_interface = GripperInterface(
            node=self,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=self.__callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )
        #collision stuff
        self.__moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.__callback_group,
        )


        #arm movement stuff
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
        #aprilTag coordiantes
        self.tagPose = TagPose

        #action server stuff
        #self.wristPose = WristPose
        self._action_server = ActionServer(
            self,
            PickAndPlace,
            'pickAndPlace',
            self.execute_callback)

        #action client stuff
        self._action_client = ActionClient(self, PickAndPlace, 'pickAndPlace')

        


        
        self.timer_count = 0
        self.timer = self.create_timer(2, self.timer_callback)
        self.removeCollision = False


        #action stuff
        self.source_frame = "gripper_link"
        self.target_frame = "base_footprint"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.wristPose = None
                   
    def timer_callback(self):  
        #Orientation data collection
        tagQuatCollision = self.tagPose.transform.rotation.x,self.tagPose.transform.rotation.y,self.tagPose.transform.rotation.z,self.tagPose.transform.rotation.w
        #position data collection and manipulation
        positionCollision = [self.tagPose.transform.translation.x+0.20,self.tagPose.transform.translation.y,self.tagPose.transform.translation.z-self.tagPose.transform.translation.z/2]
        positionBox = [self.tagPose.transform.translation.x+0.05,self.tagPose.transform.translation.y,self.tagPose.transform.translation.z+0.15/2]
        self.__add_collision_cylinder(object_id="table", 
                                        position=positionCollision,
                                        orientation=tagQuatCollision, height=self.tagPose.transform.translation.z, radius=0.26)
        # self.__add_collision_box(object_id="box", 
        #                                 position=positionBox,
        #                                 orientation=tagQuatCollision,
        #                                 dimensions=[0.06, 0.045, 0.15])



        self.timer_count += 1

        

        self.get_logger().info('Opening Gripper')
        self.__open_gripper()


        #Orientation data collection
        tagQuat = self.tagPose.transform.rotation.x,self.tagPose.transform.rotation.y,self.tagPose.transform.rotation.z,self.tagPose.transform.rotation.w
        
        #converting Quat to Euler
        tagEuler = self.euler_from_quaternion(tagQuat)
        
        #Fixing orientation
        # fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
        fixed_tagEuler = (tagEuler[0],tagEuler[1],tagEuler[2])
        #converting fixed Euler to Quat
        fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)



        #position data collection
        position = [self.tagPose.transform.translation.x,self.tagPose.transform.translation.y,self.tagPose.transform.translation.z+0.45]
        self.get_logger().info(f"Correct position that works: x={position[0]}, y={position[1]}, z={position[2]}")



        lift_position = [self.tagPose.transform.translation.x-0.11,self.tagPose.transform.translation.y,self.tagPose.transform.translation.z+0.08+0.3]
        new_position = [self.tagPose.transform.translation.x-0.11,self.tagPose.transform.translation.y-0.1,self.tagPose.transform.translation.z+0.08]

        #arm movement
        self.get_logger().info("Moving arm to pose")
        self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
        if self.timer_count <= -3:
            
           
            
            self.__remove_collision_object(object_id="box")
            self.get_logger().info('BoxCollider removed')
            
            self.get_logger().info('Partially closing Gripper')
            
            self.__partial_close_gripper()
            time.sleep(3)
            self.get_logger().info('lifting  object')
            self.__moveit2.max_velocity = 0.2
            self.__moveit2.max_acceleration = 0.2
            self.__move_arm_to_pose(position=lift_position, orientation=fixed_tagQuat)
            time.sleep(3)
            self.get_logger().info('placing  object')
            self.__move_arm_to_pose(position=new_position, orientation=fixed_tagQuat)

            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())

            self.wristPose = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]

            self.send_goal(1)
            



            self.get_logger().info('Destroying the timer after 3 executions.')
            self.destroy_timer(self.timer)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        target_pose = [0.6863669950555951, -0.044455344472087835, 0.5938897983547047]
        result = PickAndPlace.Result()  # Initialize result

        if goal_handle.request.marker_id == 1:
            if all(abs(self.wristPose[i] - target_pose[i]) < 0.01 for i in range(3)):
                self.get_logger().info('WristPose is in the correct position. Task succeeded!')
                goal_handle.succeed()
                result.result = True  # Success
            else:
                self.get_logger().warn('WristPose is not in the correct position. Task failed.')
                result.result = False  # Failure
        else:
            self.get_logger().warn('Marker_id ist nicht eins')
            result.result = False  # Failure

        return result  # Ensure the function always returns result
    

    def send_goal(self, marker_id):
        goal_msg = PickAndPlace.Goal()
        goal_msg.marker_id = marker_id

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


    

    def __open_gripper(self):
        """
        Open gripper of tiago
        """
        self.__gripper_interface.open()
        self.__gripper_interface.wait_until_executed()


    def __partial_close_gripper(self):
        """
        Close gripper of tiago
        """
        partial_close_position = 0.0195 
        #partial_close_positions = 0.3  # Example values for partial closure
        self.__gripper_interface.move_to_position(partial_close_position)
        self.__gripper_interface.wait_until_executed()


    def __remove_collision_object(self, object_id:str):
        """
        Remove collision object based on the object id
        
        Args:
          object_id : Name of the object which has to be removed
        """
        self.__moveit2.remove_collision_object(id=object_id)

    def __add_collision_cylinder(self, object_id:str, position:list[float, float, float], orientation:list[float, float, float, float], height:float, radius:float):
        """
        Add a cylinder shape as a collision box

        Args: 
        - object_id: unique object name
        - position: 3D position (x,y,z) relative of base footprint
        - orientation: quaternion orientation (x, y, z, w) relative of base footprint
        - height: height of the cylinder
        - radius: radius of the cylinder
        """
        self.__moveit2.add_collision_cylinder(
                id=object_id,
                position=position,
                quat_xyzw=orientation,
                height=height,
                radius=radius,
            )

    def __add_collision_box(self, object_id:str, position:list[float, float, float], orientation:list[float, float, float, float], dimensions:list[float, float, float]):
        """
        Add a Box shape as a collision box

        Args: 
        - object_id: unique object name
        - position: 3D position (x,y,z) relative of base footprint
        - orientation: quaternion orientation (x, y, z, w) relative of base footprint
        - dimensions: dimension of the box
        """
        self.__moveit2.add_collision_box(
                id=object_id,
                position=position,
                quat_xyzw=orientation,
                size=dimensions
            )


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

    def euler_from_quaternion(self,quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Convert results to degrees
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

        return roll_deg, pitch_deg, yaw_deg

   

    def quaternion_from_euler(self,euler_angles):
        """
        Converts Euler angles (roll, pitch, yaw) in degrees to a quaternion (x, y, z, w).
    
        Args:
            roll: Rotation around the X-axis in degrees.
            pitch: Rotation around the Y-axis in degrees.
            yaw: Rotation around the Z-axis in degrees.
    
        Returns:
            A list [x, y, z, w] representing the quaternion.
        """

        # Extract roll, pitch, and yaw from the input list
        roll, pitch, yaw = euler_angles
        # Convert degrees to radians
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

        # Perform the quaternion calculations
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        # Ensure the quaternion is in [x, y, z, w] order
        q = [0] * 4
        q[0] = cr * cp * cy + sr * sp * sy  # w
        q[1] = sr * cp * cy - cr * sp * sy  # x
        q[2] = cr * sp * cy + sr * cp * sy  # y
        q[3] = cr * cp * sy - sr * sp * cy  # z

        #    Swap to [x, y, z, w]
        return [q[1], q[2], q[3], q[0]]




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
       
        start_time = time.time()
        timeout = 10  # Timeout in seconds
        while time.time() - start_time < timeout:
            rclpy.spin_once(detector)  # Spin once
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
            detector.destroy_node()
    
    finally:
        rclpy.shutdown()
if __name__ == "__main__":
    main()