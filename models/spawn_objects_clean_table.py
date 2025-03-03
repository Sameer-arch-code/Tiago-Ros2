#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

class SpawnObjectNode(Node):
    def __init__(self):
        super().__init__('spawn_object_node')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        
        self.get_logger().info("Ready to spawn objects")
        #self.spawn_object()

    def spawn_marker(self,  name, marker, pose):
        april_tag_path = "april_tags"

        with open(april_tag_path+ "/" + marker + '/model.sdf') as f:
            xml = f.read()
        
        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml
        request.initial_pose = pose

        future = self.cli.call_async(request)
        future.add_done_callback(self.callback)

    def spawn_object(self, name, model_name, width, depth, height, pose, mass=0.1):
        request = SpawnEntity.Request()

        request.name = name
        request.xml = self.__get_box_sdf(model_name, width, depth, height, mass)
        request.initial_pose = pose

#         request.name = "my_object2"
#         request.xml = """<?xml version='1.0'?>
# <sdf version='1.6'>
#   <model name='box'>
#     <pose>0 0 0.5 0 0 0</pose>
#     <link name='link'>
#       <pose>0 0 0.5 0 0 0</pose>
#       <collision name='collision'>
#         <geometry>
#           <box>
#             <size>1 1 1</size>
#           </box>
#         </geometry>
#       </collision>
#       <visual name='visual'>
#         <geometry>
#           <box>
#             <size>1 1 1</size>
#           </box>
#         </geometry>
#       </visual>
#     </link>
#   </model>
# </sdf>"""
#         request.robot_namespace = ""
#         request.initial_pose.position.x = 1.0
#         request.initial_pose.position.y = 0.0
#         request.initial_pose.position.z = 0.5

        future = self.cli.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Successfully spawned object!')
            else:
                self.get_logger().error(f'Failed to spawn object: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Exception: {str(e)}')

    def __get_box_sdf(self, model_name, x, y, z, mass, static=0):
        # Calculate inertia for a box
        inertia_xx = (mass / 12.0) * (y**2 + z**2)
        inertia_yy = (mass / 12.0) * (x**2 + z**2)
        inertia_zz = (mass / 12.0) * (x**2 + y**2)

        friction_coefficient = 2.0

        return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name="{model_name}">
    <pose>0 0 0.0 0 0 0</pose>
    <link name="{model_name}_link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{inertia_xx}</ixx>
          <iyy>{inertia_yy}</iyy>
          <izz>{inertia_zz}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>{x} {y} {z}</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>{friction_coefficient}</mu>
              <mu2>{friction_coefficient}</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="{model_name}_visual">
        <geometry>
          <box>
            <size>{x} {y} {z}</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""

def main(args=None):
    rclpy.init(args=args)

    import time

    current_timestamp = int(time.time())

    node = SpawnObjectNode()

    pose = Pose()
    pose.orientation.w = 1.0
    
    id = 1
    pose.position.x = -4.348470
    pose.position.y = 3.1
    pose.position.z = 0.6
    node.spawn_object("box"+str(id),"box", 0.04, 0.06, 0.15 ,pose)
    pose.position.y -= 0.07
    pose.position.z -= 0.09
    node.spawn_marker("tag"+str(id),"Apriltag36_11_0000"+str(id),pose)

    id = 2
    pose.position.x = -4.291854
    pose.position.y = 3.204903
    pose.position.z = 0.6
    node.spawn_object("box"+str(id),"box", 0.04, 0.06, 0.15 ,pose)
    pose.position.y -= 0.07
    pose.position.z -= 0.09
    node.spawn_marker("tag"+str(id),"Apriltag36_11_0000"+str(id),pose)

    id = 3
    pose.position.x = -4.131414
    pose.position.y = 3.130812
    pose.position.z = 0.6
    node.spawn_object("box"+str(id),"box", 0.04, 0.06, 0.15 ,pose)
    pose.position.y -= 0.07
    pose.position.z -= 0.09
    node.spawn_marker("tag"+str(id),"Apriltag36_11_0000"+str(id),pose)

    id = 10
    pose.position.x = -2.761000
    pose.position.y = 4.128580
    pose.position.z = 0.510000
    node.spawn_marker("tag"+str(id),"Apriltag36_11_000"+str(id),pose)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
