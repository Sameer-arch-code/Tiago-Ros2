import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time

from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import tiago as robot


#for markerDetection
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener





class CollisionObjectExample(Node):
    """
    Example how to add and remove collision objects

    There are different objects types which can be added, the most important here are box and cylinder.

    There are two possibilities to remove collision objects (examples are given below):
      1. Remove by object id
      2. Remove the alle collision objects
    """

    def __init__(self,TagPose):
        super().__init__("collision_object_example_node")
        self.__callback_group = ReentrantCallbackGroup()

        self.__moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.__callback_group,
        )

        self.tagPose = TagPose
        
        self.timer_callback()

    def timer_callback(self):
        self.get_logger().info('Adding Collisionobject')


        #Orientation data collection
        tagQuat = self.tagPose.transform.rotation.x,self.tagPose.transform.rotation.y,self.tagPose.transform.rotation.z,self.tagPose.transform.rotation.w
        #position data collection and manipulation
        position = [self.tagPose.transform.translation.x+0.20,self.tagPose.transform.translation.y,self.tagPose.transform.translation.z-self.tagPose.transform.translation.z/2]
        
        self.__add_collision_cylinder(object_id="table", 
                                        position=position,
                                        orientation=tagQuat, height=self.tagPose.transform.translation.z, radius=0.26)

      



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

   

    

class FrameListener(Node):

    def __init__(self):
        super().__init__('test_tf2_frame_listener')

        self.source_frame = "tag36h11:1"
        self.target_frame = "base_footprint"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.poseOfTag = None

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)
        #self.on_timer
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

    #getter method to collect tagPose
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
            rclpy.spin_once(detector)  # Spin once
            tagPose = detector.getPoseOfTag()
            if tagPose:
                break  # Exit loop if tagPose is found

        if tagPose:
            #collisionobj is only created when tagPose could be fetched
            collisionobj = CollisionObjectExample(tagPose)
            executor.add_node(collisionobj)
        else:
            print('tagPose ist null, value:',tagPose)



        try:
            executor.spin()

        finally:
            executor.shutdown()
        
            collisionobj.destroy_node()
            detector.destroy_node()
            
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()