import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import DeleteEntity
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry


import time

import math
from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import tiago as robot


#for markerDetection
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RockAndRoll(Node):
    def __init__(self):
        super().__init__('rock_and_roll')
        
        
        #intiiao_pose stuff
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.publish_initial_pose()
        
       
        self.amcl_subscriber = self. create_subscription(PoseWithCovarianceStamped, '/amcl_pose',self.amcl_callback,10)

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
        #gripper stuff ends here

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
        #arm movement stuff ends here
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.check_obstacle,
            10
        )
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        #velociy data
        self.odom_subscriber = self.create_subscription(Odometry,'/mobile_base_controller/odom',self.odom_callback,10)
        self.robot_halted = False

        self.BeginMoveRequest = True
        self.reached_table = False
        self.t1 = None 
        self.t2 = None
        self.t3 = None
        self.t4 = None
        self.t5 = None
        self.t10 = None

        self.StartFrameListeningFlag=True
        #frame listener stuff
        self.source_frame1 = "tag36h11_1"
        self.source_frame2 = "tag36h11_2"
        self.source_frame3 = "tag36h11_3"
        self.source_frame4 = "tag36h11_4"
        self.source_frame5 = "tag36h11_5"
        self.source_frame10 = "tag36h11_10"
        self.robot_position = None
        




        
        self.target_frame = "base_footprint"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #frame listener stuff ends
        self.Fertig =0

        self.isObject1_moved = False
        self.isObject2_moved = True
        self.isObject3_moved = True
        self.isObject4_moved = True
        self.isObject5_moved = True

        self.isObjectGrasped = False
        

        self.robot_position = [0.0, 0.0]
        self.robot_orientation = 0.0  # In radians
  
        self.main_timer = self.create_timer(1.0, self.main_timer_callback)
       
    def main_timer_callback(self):
        if self.reached_table & self.robot_halted:
            self.get_logger().info("start doing things")
            if self.StartFrameListeningFlag==True:

                self.StartFrameListening()
      
            if self.isObject1_moved == False:
                if self.t1 is not None:
                    self.add_pickup_table_collision_object()
                    if self.isObjectGrasped ==False:
                        self.addCollisionObjectsForFartherObjects()
                        
                        self.graspAndRetractObject(self.t1)
                        self.removeAllCollisionObjects()
                        self.removeAllCollisionObjects()
                        self.moveToPlaceTableAndReturn()
                        self.isObject2_moved = False
                    

            if self.isObject2_moved == False:
                if self.isObjectGrasped ==False:
                    self.StartFrameListeningFlag=True
                    
                    tag = self.findNearestTag()
                    self.get_logger().info(f"Nearest tag found, frame Id: {tag.child_frame_id},{tag}")
  
                    self.moveRobotToGraspableDistance(tag)
                    
                    self.get_logger().info("moved robot to graspable distance")
                    while not self.robot_halted:
                        self.get_logger().info("waiting for robot to halt")
                    self.hand_brake()
                    time.sleep(0.1) 
                    self.StartFrameListening()
                    self.add_pickup_table_collision_object()
                    self.addCollisionObjectsForFartherObjects()
                    self.addCollisionObjectsForFartherObjects()
                    time.sleep(0.1) 
                    self.graspAndRetractObject(tag)
                    self.removeAllCollisionObjects()
                    self.moveToPlaceTableAndReturn2()
                    self.isObject3_moved = False  

            if self.isObject3_moved == False:
                if self.isObjectGrasped ==False:
                    self.StartFrameListeningFlag=True
                    
                    tag = self.findNearestTag()   
                    self.get_logger().info(f"Nearest tag found, frame Id: {tag.child_frame_id},{tag}")  
                    self.moveRobotToGraspableDistance(tag)       
                    self.get_logger().info("moved robot to graspable distance")
                    while not self.robot_halted:
                        self.get_logger().info("waiting for robot to halt")
                    self.hand_brake()
                    time.sleep(0.1) 
                    self.StartFrameListening()
                    self.add_pickup_table_collision_object()
                    self.addCollisionObjectsForFartherObjects()
                    self.addCollisionObjectsForFartherObjects()
                    time.sleep(0.1) 
                    self.graspAndRetractObject(tag)
                    self.removeAllCollisionObjects()
                    self.moveToPlaceTableAndReturn2()
                    self.isObject4_moved = False  

            if self.isObject4_moved == False:
                if self.isObjectGrasped ==False:
                    self.StartFrameListeningFlag=True
                    
                    tag = self.findNearestTag()  
                    self.get_logger().info(f"Nearest tag found, frame Id: {tag.child_frame_id},{tag}")       
                    self.moveRobotToGraspableDistance(tag)                  
                    self.get_logger().info("moved robot to graspable distance")
                    while not self.robot_halted:
                        self.get_logger().info("waiting for robot to halt")
                    self.hand_brake()
                    time.sleep(0.1) 
                    self.StartFrameListening()
                    self.add_pickup_table_collision_object()
                    self.addCollisionObjectsForFartherObjects()
                    self.addCollisionObjectsForFartherObjects()
                    time.sleep(0.1) 
                    self.graspAndRetractObject(tag)
                    self.removeAllCollisionObjects()
                    self.moveToPlaceTableAndReturn2()
                    self.isObject5_moved = False  #put it at last   

            if self.isObject5_moved == False:
                if self.isObjectGrasped ==False:
                    self.StartFrameListeningFlag=True
                    
                    tag = self.findNearestTag()   #implement fertig in this
                    self.get_logger().info(f"Nearest tag found, frame Id: {tag.child_frame_id},{tag}")
                    
                    
                    self.moveRobotToGraspableDistance(tag)
                    
                    self.get_logger().info("moved robot to graspable distance")
                    while not self.robot_halted:
                        self.get_logger().info("waiting for robot to halt")
                    self.hand_brake()
                    time.sleep(0.1) 
                    self.StartFrameListening()
                    self.add_pickup_table_collision_object()
                    self.addCollisionObjectsForFartherObjects()
                    self.addCollisionObjectsForFartherObjects()
                    time.sleep(0.1) 
                    self.graspAndRetractObject(tag)
                    self.removeAllCollisionObjects()
                    self.moveToPlaceTableAndReturn2()
                    
               
                
    def moveRobotToGraspableDistance(self,tag):
        if tag.transform.translation.x > 0.65:
            while self.tf_buffer.lookup_transform(self.target_frame,tag.child_frame_id, rclpy.time.Time()).transform.translation.x>0.65:
                self.get_logger().info("I am too far, moving forward")
                self.move_robot(0.06,0.06)

        if tag.transform.translation.x < 0.52:  #use 0.52
            while self.tf_buffer.lookup_transform(self.target_frame,tag.child_frame_id, rclpy.time.Time()).transform.translation.x<0.52:
                self.get_logger().info("I am too near, moving backward")
                self.move_robot(-0.06,-0.06)

    

    def findNearestTag(self):
        # Collect tags that are not None
        valid_tags = [tag for tag in [self.t1, self.t2, self.t3, self.t4, self.t5] if tag is not None]

        # Sort by x position (ascending order)
        sorted_tags = sorted(valid_tags, key=lambda tag: tag.transform.translation.x)

        # Ignore the first 'Fertig' nearest tags and return the next one
        return sorted_tags[self.Fertig] if len(sorted_tags) > self.Fertig else None


    def moveToPlaceTableAndReturn2(self):
        # self.rotate_robot(-190.0,0.5)
        # self.move_robot(2.75,1.5)
        self.rotate_robot(-120.0,0.5)
        self.get_logger().info('rotate worked')
        # self.move_and_rotate(3.5,1.5,0.0,0.0)
        self.move_robot(3.4,1.5)
        self.get_logger().info('move front worked')

        self.rotate_robot(90.0,0.5)
        #moving a little forward
        self.move_robot(-0.3,-1.0)

        tag = self.StartFrameListeningTag10()
        self.moveRobotToGraspableDistance(tag)
        self.hand_brake()
        time.sleep(0.1) 
        self.addPlaceTableCollisionObject()

        if self.isObject1_moved == False:
            position = [self.t10.transform.translation.x-0.11,self.t10.transform.translation.y,self.t10.transform.translation.z+0.08]
            tagQuat = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
            tagEuler = self.euler_from_quaternion(tagQuat)
            fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)

        if self.isObject2_moved == False:
            position = [self.t10.transform.translation.x-0.11,self.t10.transform.translation.y-0.11,self.t10.transform.translation.z+0.08]
            tagQuat = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
            tagEuler = self.euler_from_quaternion(tagQuat)
            fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)

        if self.isObject3_moved == False:
            position = [self.t10.transform.translation.x-0.11,self.t10.transform.translation.y+0.11,self.t10.transform.translation.z+0.08]
            tagQuat = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
            tagEuler = self.euler_from_quaternion(tagQuat)
            fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)

        if self.isObject4_moved == False:
            position = [self.t10.transform.translation.x,self.t10.transform.translation.y+0.11,self.t10.transform.translation.z+0.08]
            tagQuat = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
            tagEuler = self.euler_from_quaternion(tagQuat)
            fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)

        if self.isObject5_moved == False:
            position = [self.t10.transform.translation.x,self.t10.transform.translation.y-0.11,self.t10.transform.translation.z+0.08]
            tagQuat = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
            tagEuler = self.euler_from_quaternion(tagQuat)
            fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
        self.__open_gripper()
        self.retractArm()  
        self.retractArm() 
        self.__remove_collision_object(object_id="table1")
        # self.move_robot(-10.0,-1.5)
        # self.publish_goal_pose()
        self.rotate_robot(220.0,1.0)
        self.move_robot(3.3,1.5)
        self.rotate_robot(-270,1.5)
        self.move_robot(-1.2,-1.5) #use this
        #self.move_robot(0.3,1.0)
        self.StartFrameListeningFlag = True
        

        if self.isObject1_moved == False:
            self.isObject1_moved = True
        if self.isObject2_moved == False:
            self.isObject2_moved = True
        if self.isObject3_moved == False:
            self.isObject3_moved = True
        if self.isObject4_moved == False:
            self.isObject4_moved = True
        if self.isObject5_moved == False:
            self.isObject5_moved = True

        self.Fertig+=1
        self.isObjectGrasped=False

    def moveToPlaceTableAndReturn(self):
        # self.move_and_rotate(0.0,0.0,-120.0,-0.5)
        self.rotate_robot(-120.0,0.5)
        self.get_logger().info('rotate worked')
        # self.move_and_rotate(3.5,1.5,0.0,0.0)
        self.move_robot(3.5,1.5)
        self.get_logger().info('move front worked')

        self.rotate_robot(100.0,0.5)
        #moving a little forward
        self.move_robot(0.75,1.0)

        self.StartFrameListeningTag10()
        self.addPlaceTableCollisionObject()

        if self.isObject1_moved == False:
            position = [self.t10.transform.translation.x-0.11,self.t10.transform.translation.y,self.t10.transform.translation.z+0.08]
            tagQuat = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
            tagEuler = self.euler_from_quaternion(tagQuat)
            fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)

        if self.isObject2_moved == False:
            position = [self.t10.transform.translation.x-0.11,self.t10.transform.translation.y-0.11,self.t10.transform.translation.z+0.08]
            tagQuat = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
            tagEuler = self.euler_from_quaternion(tagQuat)
            fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)

        if self.isObject3_moved == False:
            position = [self.t10.transform.translation.x-0.11,self.t10.transform.translation.y+0.11,self.t10.transform.translation.z+0.08]
            tagQuat = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
            tagEuler = self.euler_from_quaternion(tagQuat)
            fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)

        if self.isObject4_moved == False:
            position = [self.t10.transform.translation.x,self.t10.transform.translation.y+0.11,self.t10.transform.translation.z+0.08]
            tagQuat = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
            tagEuler = self.euler_from_quaternion(tagQuat)
            fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)

        if self.isObject5_moved == False:
            position = [self.t10.transform.translation.x,self.t10.transform.translation.y-0.11,self.t10.transform.translation.z+0.08]
            tagQuat = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
            tagEuler = self.euler_from_quaternion(tagQuat)
            fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)

        self.__open_gripper()
        self.retractArm()  
        self.__remove_collision_object(object_id="table1")
        # self.move_robot(-10.0,-1.5)
        # self.publish_goal_pose()
        self.rotate_robot(220.0,1.0)
        self.move_robot(3.3,1.5)
        self.rotate_robot(-270,1.5)
        self.move_robot(-1.2,-1.5) #use this
        #self.move_robot(0.3,1.0)
        self.StartFrameListeningFlag = True
        

        if self.isObject1_moved == False:
            self.isObject1_moved = True
        if self.isObject2_moved == False:
            self.isObject2_moved = True
        if self.isObject3_moved == False:
            self.isObject3_moved = True
        if self.isObject4_moved == False:
            self.isObject4_moved = True
        if self.isObject5_moved == False:
            self.isObject5_moved = True

        self.Fertig+=1
        self.isObjectGrasped=False
        #self.rotate_robot(-120.0,0.5)

        # self.move_and_rotate(-3.0,-1.5,0.0,0.0)
        #self.move_robot(-3.5,-1.5)
       

        # if self.hasReachedNearTable():
        #     self.get_logger().info('reached back to table. Twist works fine')
        # else:
        #     self.get_logger().info('did not reach back to table. Twist failed')

        #self.move_and_rotate(3.5,1.5,0.0,0.0)

    def addCollisionObjectsForFartherObjects(self):
        offsetX = 0.05
        offsetY = 0.005

        # Collect tags that are not None and sort them by x position
        valid_tags = [tag for tag in [self.t1,self.t2, self.t3, self.t4, self.t5] if tag is not None]
        sorted_tags = sorted(valid_tags, key=lambda tag: tag.transform.translation.x)

        # Select tags that are farther than Fertig (e.g., 3rd, 4th, 5th nearest if Fertig = 2)
        farther_tags = sorted_tags[self.Fertig:]

        # Add collision boxes for selected tags
        for i, tag in enumerate(farther_tags, start=1):
            tagQuatCollision = (
                tag.transform.rotation.x, tag.transform.rotation.y,
                tag.transform.rotation.z, tag.transform.rotation.w
            )
            positionBox = [
                tag.transform.translation.x + offsetX,
                tag.transform.translation.y + offsetY,
                tag.transform.translation.z + 0.15 / 2
            ]
            self.__add_collision_box(
                object_id=f"box{i+self.Fertig}",
                position=positionBox,
                orientation=tagQuatCollision,
                dimensions=[0.065, 0.06, 0.15]
            )
            self.__add_collision_box(
                object_id=f"box{i+self.Fertig}",
                position=positionBox,
                orientation=tagQuatCollision,
                dimensions=[0.065, 0.06, 0.15]
            )
    
    def graspAndRetractObject(self,tag):
        
        self.__open_gripper()
        tagQuatNotHardcoded = tag.transform.rotation.x,tag.transform.rotation.y,tag.transform.rotation.z,tag.transform.rotation.w
        tagQuat = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
        
        tagEuler = self.euler_from_quaternion(tagQuat)
        fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
        fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)
       
        position = [tag.transform.translation.x-0.11,tag.transform.translation.y,tag.transform.translation.z+0.08]

        self.get_logger().info(f"printing position: {position[0]}, {position[1]}, {position[2]}")

        self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
        self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
        self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
        
        
        self.__remove_collision_object(object_id="box1")
        self.__partial_close_gripper()
        self.isObjectGrasped=True

            # lift_position = [self.t1.transform.translation.x-0.45,self.t1.transform.translation.y+0.15,self.t1.transform.translation.z+0.08]
        self.retractArm()
        
        
    def retractArm(self):
        tagQuat = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
        tagEuler = self.euler_from_quaternion(tagQuat)
        fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
        fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

        lift_position = [0.30835506040390953, 0.2002450995744438, 0.5946754351951143]

        self.__move_arm_to_pose(position=lift_position, orientation=fixed_tagQuat)
        self.__move_arm_to_pose(position=lift_position, orientation=fixed_tagQuat)



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
        # success = self.__moveit2.wait_until_executed()

        # if not success:
        #     self.get_logger().error("Motion planning failed! Unable to reach the target pose.")
        # else:
        #     self.get_logger().info("Motion successfully executed.")
    def add_pickup_table_collision_object(self):
        # self.tagQuatCollision = self.t1.transform.rotation.x,self.t1.transform.rotation.y,self.t1.transform.rotation.z,self.t1.transform.rotation.w
        self.tagQuatCollision = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
        #position data collection and manipulation
        self.positionCollision = [self.t1.transform.translation.x+0.21,self.t1.transform.translation.y-0.08,self.t1.transform.translation.z-self.t1.transform.translation.z/2]
        if self.t1 is not None:
            self.__add_collision_cylinder(object_id="table", 
                                        position=self.positionCollision,
                                        orientation=self.tagQuatCollision, height=self.t1.transform.translation.z, radius=0.28)
        # self.isPickUpTableCollisionPlaced = True

    def addPlaceTableCollisionObject(self):
        
        # self.tagQuatCollision = self.t1.transform.rotation.x,self.t1.transform.rotation.y,self.t1.transform.rotation.z,self.t1.transform.rotation.w
        self.tagQuatCollision = -0.0032561761601856507, -0.018640836049033188, 0.9992707401185019,-0.03316480802338898
        #position data collection and manipulation
        self.positionCollision = [self.t10.transform.translation.x,self.t10.transform.translation.y,self.t10.transform.translation.z-self.t10.transform.translation.z/2]
        self.__add_collision_box(object_id="table1", 
                                        position=self.positionCollision,
                                        orientation=self.tagQuatCollision, dimensions=[0.43, 0.43, self.t10.transform.translation.z])

    def euler_from_quaternion(self,quaternion):

        #source: https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
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
        #source: https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
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


    def amcl_callback(self, msg):
        self.robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        positionList = msg
        # self.get_logger().info(f'Current Position: {position}')
        if self.robot_position == [0,0,0]:
            self.publish_initial_pose()
        
        self.tolerance = 0.5
        if (-0.55-self.tolerance <= self.robot_position[0] <= -0.55+self.tolerance) and (5.5-self.tolerance <= self.robot_position[1] <= 5.5+self.tolerance):
            
            self.reached_table = True
            # if self.robot_halted:  # Only proceed if the robot is halted
            #     self.StartFrameListening1()
            #     self.get_logger().info("target reached, tranform starts now")
    def hasReachedNearTable(self):
        self.tolerance = 0.1
        if (-0.55-self.tolerance <= self.robot_position[0] <= -0.55+self.tolerance) and (5.5-self.tolerance <= self.robot_position[1] <= 5.5+self.tolerance):
            return True
        else:
            return False

    def StartFrameListening(self):
        self.StartFrameListeningFlag=False
        try:
            self.t1 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame1, rclpy.time.Time())
            self.get_logger().info(f"Transform1 received: {self.t1}")
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.source_frame1} to {self.target_frame}: {ex}')
        
        try:
            self.t2 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame2, rclpy.time.Time())
            self.get_logger().info(f"Transform2 received: {self.t2}")
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.source_frame2} to {self.target_frame}: {ex}')
        try:
            self.t3 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame3, rclpy.time.Time())
            self.get_logger().info(f"Transform3 received: {self.t3}")
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.source_frame3} to {self.target_frame}: {ex}')
        try:
            self.t4 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame4, rclpy.time.Time())
            self.get_logger().info(f"Transform4 received: {self.t4}")
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.source_frame4} to {self.target_frame}: {ex}')
        try:
            self.t5 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame5, rclpy.time.Time())
            self.get_logger().info(f"Transform5 received: {self.t5}")
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.source_frame5} to {self.target_frame}: {ex}')


        
 
        if self.t1 is None:
            self.get_logger().warn(f"Could not transform {self.source_frame1} to {self.target_frame}: {ex}")
            self.StartFrameListeningFlag=True

    def StartFrameListeningTag10(self):
       
        self.t10 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame10, rclpy.time.Time())
        self.get_logger().info(f"Transform1 received: {self.t10}")
        time.sleep(0.1) 
        return self.t10

    def odom_callback(self, msg):

        self.robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

        # Extract yaw (rotation around Z-axis) from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_orientation = math.atan2(siny_cosp, cosy_cosp)

        # self.get_logger().info(f"Robot Position from odmetry: x={self.robot_position[0]}, y={self.robot_position[1]}")
        # self.get_logger().info(f"Robot Orientation from odometry (yaw in degrees): {math.degrees(self.robot_orientation)}")

        # Extract linear and angular velocities
        linear_velocity = msg.twist.twist.linear
        angular_velocity = msg.twist.twist.angular

        # Check if the robot is halted (near zero velocity)
        velocity_threshold = 0.0001  # Adjust if needed
        if (abs(linear_velocity.x) < velocity_threshold and 
            abs(linear_velocity.y) < velocity_threshold and 
            abs(angular_velocity.z) < velocity_threshold):
            self.robot_halted = True
    
        else:
            self.robot_halted = False

    def publish_initial_pose(self):
        # self.initial_pose_count+=1
        # Specify the coordinates for the initial position
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = Time().to_msg()
        initial_pose.header.frame_id = "map"  # Ensure this matches your robot's frame

        # Set the position (x, y, z)
        initial_pose.pose.pose.position.x = -7.5094286874020275
        initial_pose.pose.pose.position.y = 1.156125451705252
        initial_pose.pose.pose.position.z = 0.0  # Usually 0 for 2D navigation

        # Set the orientation (qx, qy, qz, qw)
        # Original orientation
        z_original = 0.09096499073853476
        w_original = 0.9958540909490398

        # Convert original quaternion to yaw angle
        yaw_original = 2 * math.atan2(z_original, w_original)

        # Rotate by -1 degree (clockwise)
        yaw_new = yaw_original - math.radians(0.23)  # Convert degrees to radians

        # Convert back to quaternion
        z_new = math.sin(yaw_new / 2.0)
        w_new = math.cos(yaw_new / 2.0)

        # Set the new orientation
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = z_new
        initial_pose.pose.pose.orientation.w = w_new

        # Set the covariance matrix
        initial_pose.pose.covariance = [
            0.19399532238309547, -0.016307481898881093, 0.0, 0.0, 0.0, 0.0,
            -0.016307481898881093, 0.24571536869717292, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05931886022456813
        ]

        # Publish the initial pose
        self.initial_pose_publisher.publish(initial_pose)
        self.get_logger().info('Published initial pose: x={}, y={}'.format(
            initial_pose.pose.pose.position.x,
            initial_pose.pose.pose.position.y))

        # if self.initial_pose_count >= 2:
        #     self.destroy_timer(self.initial_pose_timer)

    def check_obstacle(self, scan: LaserScan):
        """
        Checks the /scan topic to determine if there is an obstacle in front of the robot.
        """

        # If the delay hasn't passed yet, don't process scan data
        # if not self.delay_finished:
        #     self.get_logger().info("Waiting for the initial pose delay to complete.")
        #     return
        # Ensure ranges are valid
        if not scan.ranges:
            self.get_logger().warn("No valid data in LaserScan ranges.")
            return

        # Assume the front of the robot is at the middle index of the ranges array
        front_index = len(scan.ranges) // 2

        # Range values closer than this threshold indicate an obstacle
        obstacle_threshold = 0.7  # meters

        if scan.ranges[front_index] < obstacle_threshold:
            
            self.get_logger().info("FBI, OPEN UP!!!")
            
            # self.delete_door()
        else:
            if self.BeginMoveRequest:
                self.get_logger().info("No obstacle detected. Door is clear. Charging in!")
                #self.move_and_rotate(3.5,1.5,-30.0,0.5)
                self.publish_goal_pose()

            
            #self.scan_subscriber.destroy()  # Unsubscribe from the topic

    def publish_goal_pose(self):
        """
        Publishes a goal pose for the robot to move to a specified position.
        """
        #self.float_arm()
        self.get_logger().info("pose goal starts")
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'  # Or 'base_link' depending on your setup
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # Set target position (adjust these values as needed)
        goal_pose.pose.position.x =  -0.55  # Target X coordinate
        goal_pose.pose.position.y =  5.5  # Target Y coordinate
        goal_pose.pose.position.z =  0.0

        # # Orientation (no rotation, facing forward)
        # goal_pose.pose.orientation.z = 0.0 #0.13
        # goal_pose.pose.orientation.w = 1.0   #0.77    #0.99

        # Original orientation
        z_original = 0.09096499073853476
        w_original = 0.9958540909490398

        # Convert original quaternion to yaw angle
        yaw_original = 2 * math.atan2(z_original, w_original)

        # Rotate by -1 degree (clockwise)
        yaw_new = yaw_original - math.radians(30)  # Convert degrees to radians

        # Convert back to quaternion
        z_new = math.sin(yaw_new / 2.0)
        w_new = math.cos(yaw_new / 2.0)

        # Set the new orientation
        # goal_pose.pose.pose.orientation.x = 0.0
        # goal_pose.pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = z_new
        goal_pose.pose.orientation.w = w_new
 
        self.get_logger().info("Publishing goal pose for robot to move to (-0.419 , 5.25)...")
        self.goal_pose_pub.publish(goal_pose)
        self.BeginMoveRequest = False
        

    def float_arm(self):
        self.__open_gripper()
        # position = [0.6961498416740417, 0.14734648839816558, 0.9614743897492457]
        position = [0.5, 0.14734648839816558, 0.9614743897492457]
        orientation = [0,0,0,1]
        self.__move_arm_to_pose(position=position, orientation=orientation)

    def __open_gripper(self):
        """
        Open gripper of tiago
        """

        self.get_logger().info(f"open_grippper is triggered")
        self.__gripper_interface.open()
        self.__gripper_interface.wait_until_executed()

    def __partial_close_gripper(self):
        """
        Close gripper of tiago
        """
        self.get_logger().info('Partially closing Gripper')
        partial_close_position = 0.0195
        #old value = 0.0195
        #partial_close_positions = 0.3  # Example values for partial closure
        self.__gripper_interface.move_to_position(partial_close_position)
        self.__gripper_interface.wait_until_executed()

    
    def move_and_rotate(self, distance: float, speed: float, angle_degrees: float, angular_speed: float):
        """
        Moves the robot forward for a given distance, then rotates it by a given angle.
        :param distance: Distance to move forward (meters)
        :param speed: Forward speed (m/s)
        :param angle_degrees: Angle to rotate after moving (degrees)
        :param angular_speed: Rotation speed (radians/sec)
        """
        
        move_cmd = Twist()

        # Step 1: Move Forward
        move_cmd.linear.x = speed
        move_cmd.angular.z = 0.0

        start_time = time.time()
        distance_travelled = 0.0
    
        while abs(distance_travelled) < abs(distance):
            self.cmd_vel_pub.publish(move_cmd)
            current_time = time.time()
            distance_travelled = abs((current_time - start_time) * speed)
       

        # Stop moving
        move_cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        self.get_logger().info(f"Moved {distance} meters forward.")

        # Step 2: Rotate
        angle_radians = angle_degrees * (3.1415926535 / 180.0)
        move_cmd.angular.z = angular_speed 

        start_time = time.time()
        angle_travelled = 0.0

        while abs(angle_travelled) < abs(angle_radians):  # <-- FIXED
            self.cmd_vel_pub.publish(move_cmd)
            current_time = time.time()
            angle_travelled = (current_time - start_time) * angular_speed

        # Stop rotating
        move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        self.get_logger().info(f"Rotated {angle_degrees} degrees.")

    def move_robot(self, distance: float, speed: float):
        """
        Moves the robot forward or backward for a given distance.
        :param distance: Distance to move (positive = forward, negative = backward) [meters]
        :param speed: Speed (positive = forward, negative = backward) [m/s]
        """
        while self.robot_halted == False:
            self.get_logger().info(f"waiting for robot to halt")
            if self.robot_halted:
                break

            
        move_cmd = Twist()
        move_cmd.linear.x = speed  # Speed can be positive or negative
        move_cmd.angular.z = 0.0

        start_time = time.time()
        distance_travelled = 0.0

        while abs(distance_travelled) < abs(distance):
            self.cmd_vel_pub.publish(move_cmd)
            current_time = time.time()
            elapsed_time = current_time - start_time
            distance_travelled = elapsed_time * speed  # Accounts for direction

            time.sleep(0.05)  # Optional: Smooth control loop

        # Stop the robot
        move_cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        self.get_logger().info(f"Moved {distance} meters.")

    def hand_brake(self):
        """
        Immediately stops the robot by publishing zero velocity.
        """
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(move_cmd)  # Send stop command
        self.get_logger().info("Emergency stop activated!")

    

    
    def rotate_robot(self, angle_degrees: float, angular_speed: float):
        """
        Rotates the robot by a given angle.
        :param angle_degrees: Angle to rotate (positive = counterclockwise, negative = clockwise) [degrees]
        :param angular_speed: Rotation speed [radians/sec]

        """
        while self.robot_halted == False:
            self.get_logger().info(f"waiting for robot to halt")
            if self.robot_halted:
                break

        move_cmd = Twist()
        angular_speed = -abs(angular_speed) if angle_degrees < 0 else abs(angular_speed)  # Ensure correct direction
        move_cmd.angular.z = angular_speed

        angle_radians = angle_degrees * (3.1415926535 / 180.0)
        start_time = time.time()
        angle_travelled = 0.0

        while abs(angle_travelled) < abs(angle_radians):
            self.cmd_vel_pub.publish(move_cmd)
            current_time = time.time()
            angle_travelled = abs((current_time - start_time) * angular_speed)
            time.sleep(0.05)  # Ensures smooth control loop

        # Stop rotation
        move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        self.get_logger().info(f"Rotated {angle_degrees} degrees.")


    def move_to_goal(self, target_x: float, target_y: float, speed: float = 1.5, angular_speed: float = 0.5):

        """
        Moves the robot to the given (x, y) coordinate.
        :param target_x: X coordinate of the goal
        :param target_y: Y coordinate of the goal
        :param speed: Speed of movement
        """
        self.get_logger().info(f"Moving to target: ({target_x}, {target_y})")

        while rclpy.ok():
            # Step 1: Calculate distance and angle to the target
            current_x, current_y = self.robot_position
            dx = target_x - current_x
            dy = target_y - current_y
            target_angle = math.atan2(dy, dx)  # Angle to target
            
            distance_to_target = math.sqrt(dx**2 + dy**2)
            angle_diff = target_angle - self.robot_orientation
            
            # Normalize angle difference to range [-pi, pi]
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            # Step 2: Rotate towards the target
            if abs(angle_diff) > 0.1:  # Tolerance for rotation
                self.rotate_robot(math.degrees(angle_diff), angular_speed=0.5)

            # Step 3: Move forward to the target
            #self.move_robot(distance_to_target, speed)
            while abs(target_x - self.robot_position) > 0.2:
                self.move_robot(0.1, 0.1)

            # Step 4: Stop if close to the target
            if distance_to_target < 0.05:  # 5cm tolerance
                self.hand_brake()
                self.get_logger().info("Arrived at destination!")
                break



    
       

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
    def __remove_collision_object(self, object_id:str):
        """
        Remove collision object based on the object id
        
        Args:
          object_id : Name of the object which has to be removed
        """
        self.__moveit2.remove_collision_object(id=object_id)
    def removeAllCollisionObjects(self):
        try:
            self.__remove_collision_object(object_id="table")
        except Exception as e:
            print(f"Failed to remove 'table': {e}")
    
        try:
            self.__remove_collision_object(object_id="box1")
        except Exception as e:
            print(f"Failed to remove 'box1': {e}")
    
        try:
            self.__remove_collision_object(object_id="box2")
        except Exception as e:
            print(f"Failed to remove 'box2': {e}")
    
        try:
            self.__remove_collision_object(object_id="box3")
        except Exception as e:
            print(f"Failed to remove 'box3': {e}")
    
        try:
            self.__remove_collision_object(object_id="box4")
        except Exception as e:
            print(f"Failed to remove 'box4': {e}")
    
        try:
            self.__remove_collision_object(object_id="box5")
        except Exception as e:
            print(f"Failed to remove 'box5': {e}")

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = RockAndRoll()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
