'''
Author: Abdurrahman Yilmaz (AYilmaz@lincoln.ac.uk)
version: v02
Date: 14 Nov 2024
Purpose: Generation of optimal lattice points array for robot arm movements
Project: Agri-Open core
'''

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

import math

from scipy.spatial.transform import Rotation

from rcl_interfaces.msg import SetParametersResult

class PoseArrayPublisher(Node):

    #rate = 30
    moving = True

    header_lattice = Header()

    pose_array = PoseArray()

    dtype = PointField.FLOAT32
    point_step = 16
    fields = [PointField(name='x', offset=0, datatype=dtype, count=1),
              PointField(name='y', offset=4, datatype=dtype, count=1),
              PointField(name='z', offset=8, datatype=dtype, count=1),
              PointField(name='intensity', offset=12, datatype=dtype, count=1)]
    
    fields_lattice = [PointField(name='x', offset=0, datatype=dtype, count=1),
              PointField(name='y', offset=4, datatype=dtype, count=1),
              PointField(name='z', offset=8, datatype=dtype, count=1),
              PointField(name='intensity', offset=12, datatype=dtype, count=1)]
    
    def parameters_callback(self, params):
        # do some actions, validate parameters, update class attributes, etc.
        self.update_points = True
        for param in params:
            if param.name == 'surface' and param.type_ == rclpy.Parameter.Type.STRING:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.surface = param.value
            if param.name == 'towards' and param.type_ == rclpy.Parameter.Type.STRING:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.towards = param.value
            if param.name == 'order' and param.type_ == rclpy.Parameter.Type.STRING:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.order = param.value
            if param.name == 'frame_id' and param.type_ == rclpy.Parameter.Type.STRING:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.header_lattice.frame_id = param.value
            if param.name == 'dist2plant' and param.type_ == rclpy.Parameter.Type.DOUBLE:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.limits.update({"x_lb_ub": [param.value, param.value], "dist2plant": [param.value, param.value]})
            if param.name == 'x_lb_ub' and param.type_ == rclpy.Parameter.Type.DOUBLE_ARRAY and len(param.value) == 2:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.limits.update({"x_lb_ub": param.value})
            if param.name == 'y_lb_ub' and param.type_ == rclpy.Parameter.Type.DOUBLE_ARRAY and len(param.value) == 2:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.limits.update({"y_lb_ub": param.value})
            if param.name == 'z_lb_ub' and param.type_ == rclpy.Parameter.Type.DOUBLE_ARRAY and len(param.value) == 2:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.limits.update({"z_lb_ub": param.value})
            if param.name == 'theta_hor_lb_ub' and param.type_ == rclpy.Parameter.Type.DOUBLE_ARRAY and len(param.value) == 2:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.limits.update({"theta_hor_lb_ub": param.value})
            if param.name == 'theta_ver_lb_ub' and param.type_ == rclpy.Parameter.Type.DOUBLE_ARRAY and len(param.value) == 2:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.limits.update({"theta_ver_lb_ub": param.value})
            if param.name == 'x_stepSize' and param.type_ == rclpy.Parameter.Type.DOUBLE:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.limits.update({"x_stepSize": param.value})
            if param.name == 'y_stepSize' and param.type_ == rclpy.Parameter.Type.DOUBLE:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.limits.update({"y_stepSize": param.value})
            if param.name == 'z_stepSize' and param.type_ == rclpy.Parameter.Type.DOUBLE:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.limits.update({"z_stepSize": param.value})
            if param.name == 'theta_hor_stepSize' and param.type_ == rclpy.Parameter.Type.DOUBLE:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.limits.update({"theta_hor_stepSize": param.value})
            if param.name == 'theta_ver_stepSize' and param.type_ == rclpy.Parameter.Type.DOUBLE:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.limits.update({"theta_ver_stepSize": param.value})
            if param.name == 'rate' and param.type_ == rclpy.Parameter.Type.INTEGER:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.rate = param.value
            if param.name == 'fixed_orientation' and param.type_ == rclpy.Parameter.Type.DOUBLE_ARRAY and len(param.value) == 3:
                self.get_logger().info("New value set to %s" % str(param.name) + " param: " + str(param.value))
                self.fixed_orientation = param.value
        return SetParametersResult(successful=True)

    def __init__(self):
        node_name = 'lattice_publisher'
        super().__init__(node_name)
        rclpy.logging._root_logger.info('%s node started to work!!!' % node_name)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('surface', "planar"),
                ('towards', "origin"),
                ('order', "horizontal"),
                ('frame_id', 'plant'),
                ('dist2plant', 1.0),
                ('x_lb_ub', [1.0, 1.0]),
                ('y_lb_ub', [-1.0, 1.0]),
                ('z_lb_ub', [-1.0, 1.0]),
                ('theta_hor_lb_ub', [-30.0, 30.0]),
                ('theta_ver_lb_ub', [-30.0, 30.0]),
                ('x_stepSize', 1.0),
                ('y_stepSize', 0.01),
                ('z_stepSize', 0.01),
                ('theta_hor_stepSize', 1.0),
                ('theta_ver_stepSize', 1.0),
                ('rate', 30),
                ('fixed_orientation', [0.0, 0.0, 0.0])
            ]
        )

        self.header_lattice.frame_id = self.get_parameter('frame_id').value # 'plant'

        self.rate = self.get_parameter('rate').value # Hz

        self.publisher_lattice = self.create_publisher(PointCloud2, 'lattice_cloud', 10)
        self.publisher_lattice_pose = self.create_publisher(PoseArray, 'lattice_pose_array', 10)
        timer_period = 1 / self.rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
        self.cnt = 0

        dist2plant = self.get_parameter('dist2plant').value #meter
        self.surface = self.get_parameter('surface').value # "planar" or "cylindrical" or "spherical"
        self.towards = self.get_parameter('towards').value # "origin" or "axisZ"
        self.order = self.get_parameter('order').value # "horizontal" or "vertical"

        self.fixed_orientation = self.get_parameter('fixed_orientation').value # [roll, pitch, yaw]
        print(self.fixed_orientation)

        self.limits = {
            "x_lb_ub": [dist2plant, dist2plant],
            "y_lb_ub": self.get_parameter('y_lb_ub').value,
            "z_lb_ub": self.get_parameter('z_lb_ub').value,
            "theta_hor_lb_ub": self.get_parameter('theta_hor_lb_ub').value,
            "theta_ver_lb_ub": self.get_parameter('theta_ver_lb_ub').value,
            "dist2plant": [dist2plant, dist2plant],
            "x_stepSize": self.get_parameter('x_stepSize').value,
            "y_stepSize": self.get_parameter('y_stepSize').value,
            "z_stepSize": self.get_parameter('z_stepSize').value,
            "theta_hor_stepSize": self.get_parameter('theta_hor_stepSize').value,
            "theta_ver_stepSize": self.get_parameter('theta_ver_stepSize').value
        }

        # Run the Sampling function
        self.poses = self.sampling_mesh(self.limits, self.surface, self.towards, self.order, self.fixed_orientation)  # Grid average sampling throughout the field
        self.update_points = False

        #print("poses (x, y, z)", poses)
        #print("size of points (numOfPts, numOfAxes)", poses.shape)

        self.add_on_set_parameters_callback(self.parameters_callback)

    def timer_callback(self):

        # update points by running the Sampling function
        if self.update_points:
            self.poses = self.sampling_mesh(self.limits, self.surface, self.towards, self.order, self.fixed_orientation)  # Grid average sampling throughout the field
            self.update_points = False

        self.header_lattice.stamp = self.get_clock().now().to_msg()
        cols = range(0, 3)
        poses = self.poses[:, cols]
        intensity = np.ones((poses.shape[0],1))

        pc2_lattice_msg = point_cloud2.create_cloud(self.header_lattice, self.fields_lattice, np.concatenate((poses,intensity), axis=1))
        self.publisher_lattice.publish(pc2_lattice_msg)

        self.pose_array.poses.clear()
        
        '''for i in range(self.poses.shape[0]):
            # Convert Euler to quaternions and print
            cols = range(3, 6)
            rot = Rotation.from_euler('XYZ', self.poses[i,cols])
            rot_quat = rot.as_quat()
            #print(rot_quat)

            cols = range(0, 3)
            pose = Pose() # create a new Pose message
            pose.position.x, pose.position.y, pose.position.z = self.poses[i,cols]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rot_quat
            self.pose_array.poses.append(pose) # add the new Pose object to the PoseArray list

        self.pose_array.header.stamp = self.get_clock().now().to_msg()
        self.pose_array.header.frame_id = self.header_lattice.frame_id
        self.publisher_lattice_pose.publish(self.pose_array)'''

        if self.cnt < self.poses.shape[0]:
            # Select the current pose by index
            cols = range(0, 3)
            position = self.poses[self.cnt, cols]
            cols = range(3, 6)
            euler_angles = self.poses[self.cnt, cols]
            
            # Create a new Pose message
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = position
            rot = Rotation.from_euler('XYZ', euler_angles)
            rot_quat = rot.as_quat()
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rot_quat

            # Publish this pose in a PoseArray with a single element
            self.pose_array.poses.clear()
            self.pose_array.poses.append(pose)
            self.pose_array.header.stamp = self.get_clock().now().to_msg()
            self.pose_array.header.frame_id = self.header_lattice.frame_id
            self.publisher_lattice_pose.publish(self.pose_array)
            
            # Move to the next pose
            self.cnt += 1
        else:
            # Reset the counter after all poses have been published
            self.cnt = 0

        #self.get_logger().info("Length of pose array %s" % str(len(self.pose_array.poses)))

        if self.moving:
            self.counter += 1
    
    def sampling_mesh(self, limits, surface="planar", towards="origin", order="horizontal", fixed_orientation=None):
        """
        Generate poses for a 3D grid with the specified scanning order.
        
        Parameters:
        - limits: Dictionary with grid boundaries and step sizes.
        - surface: Type of surface for generating points.
        - towards: Orientation direction.
        - fixed_orientation: Fixed orientation if specified.
        - order: The scanning order ("horizontal" or "vertical")
        """
        
        if surface == "planar":
            x_lim = limits.get("x_lb_ub")
            y_lim = limits.get("y_lb_ub")
            z_lim = limits.get("z_lb_ub")

            N_x = int((x_lim[1] - x_lim[0]) / limits.get("x_stepSize")) + 1
            N_y = int((y_lim[1] - y_lim[0]) / limits.get("y_stepSize")) + 1
            N_z = int((z_lim[1] - z_lim[0]) / limits.get("z_stepSize")) + 1

        elif surface == "cylindrical":
            x_lim = limits.get("dist2plant")
            y_lim = limits.get("theta_hor_lb_ub")
            z_lim = limits.get("z_lb_ub")

            N_x = int((x_lim[1] - x_lim[0]) / limits.get("x_stepSize")) + 1
            N_y = int((y_lim[1] - y_lim[0]) / limits.get("theta_hor_stepSize")) + 1
            N_z = int((z_lim[1] - z_lim[0]) / limits.get("z_stepSize")) + 1

        elif surface == "spherical":
            x_lim = limits.get("dist2plant")
            y_lim = limits.get("theta_hor_lb_ub")
            z_lim = limits.get("theta_ver_lb_ub")

            N_x = int((x_lim[1] - x_lim[0]) / limits.get("x_stepSize")) + 1
            N_y = int((y_lim[1] - y_lim[0]) / limits.get("theta_hor_stepSize")) + 1
            N_z = int((z_lim[1] - z_lim[0]) / limits.get("theta_ver_stepSize")) + 1

        else:
            x_lim = limits.get("x_lb_ub")
            y_lim = limits.get("y_lb_ub")
            z_lim = limits.get("z_lb_ub")

            N_x = int((x_lim[1] - x_lim[0]) / limits.get("x_stepSize")) + 1
            N_y = int((y_lim[1] - y_lim[0]) / limits.get("y_stepSize")) + 1
            N_z = int((z_lim[1] - z_lim[0]) / limits.get("z_stepSize")) + 1

        if order == "horizontal":
            X, Z, Y = np.mgrid[x_lim[0]:x_lim[1]:N_x*1j, z_lim[0]:z_lim[1]:N_z*1j, y_lim[0]:y_lim[1]:N_y*1j]
            # Horizontal scanning: stack points along the y-axis with zigzag across x and z
            poses = np.hstack([X.reshape(-1, 1), Y.reshape(-1, 1), Z.reshape(-1, 1)])
            for k in range(N_z):  # Loop over the z-axis
                if k % 2 == 1:  # Reverse every other z layer
                    # Reverse the 'y' values for this slice of 'z' before reshaping
                    start_idx = k * N_x * N_y
                    end_idx = (k + 1) * N_x * N_y
                    poses[start_idx:end_idx, 1] = poses[start_idx:end_idx, 1][::-1]
            poses = poses.reshape(N_x, N_y, N_z, 3)  # Arrange in a 3D grid
                            
            poses = poses.reshape(-1, 3)  # Flatten back to list of points

        elif order == "vertical":
            X, Y, Z = np.mgrid[x_lim[0]:x_lim[1]:N_x*1j, y_lim[0]:y_lim[1]:N_y*1j, z_lim[0]:z_lim[1]:N_z*1j]
            # Vertical scanning: stack points along the z-axis with zigzag across x and y
            poses = np.hstack([X.reshape(-1, 1), Y.reshape(-1, 1), Z.reshape(-1, 1)])
            poses = poses.reshape(N_x, N_y, N_z, 3)  # Arrange in a 3D grid
            
            # Apply zigzag pattern across z-axis (vertical)
            for i in range(N_x):  # Loop over the x-axis
                for j in range(N_y):  # Loop over the y-axis
                    if (i + j) % 2 == 1:  # Reverse every other column along the z-axis
                        poses[i, j, :, :] = poses[i, j, ::-1, :]
            
            poses = poses.reshape(-1, 3)  # Flatten back to list of points
        
        if surface == "cylindrical":
            for i in range(poses.shape[0]):
                dist = poses[i, 0]
                angle = poses[i, 1]
                poses[i, 0] = dist * math.cos(angle * math.pi / 180)
                poses[i, 1] = dist * math.sin(angle * math.pi / 180)
        elif surface == "spherical": 
            for i in range(poses.shape[0]):
                dist = poses[i, 0]
                angle_hor = poses[i, 1]
                angle_ver = poses[i, 2]
                poses[i, 0] = dist * math.cos(angle_ver * math.pi / 180) * math.cos(angle_hor * math.pi / 180)
                poses[i, 1] = dist * math.cos(angle_ver * math.pi / 180) * math.sin(angle_hor * math.pi / 180)
                poses[i, 2] = dist * math.sin(angle_ver * math.pi / 180)

        rpy_ = np.zeros((poses.shape[0], 3))

        if towards == "origin":
            for i in range(rpy_.shape[0]):
                pointA = poses[i,:]
                pointB = [0,0,0]
                vec_ = np.array(pointB - pointA)/np.linalg.norm(pointB - pointA)
                #z = -np.array(poses[i,:])/np.linalg.norm(poses[i,:])
                rx = math.atan2(-vec_[1], vec_[2])
                py = math.asin(vec_[0])
                yz = 0 
                rpy_[i, :] = [rx, py, yz]
        elif towards == "axisZ":
            for i in range(rpy_.shape[0]):
                pointA = poses[i,:]
                pointB = [0,0,poses[i,2]]
                vec_ = np.array(pointB - pointA)/np.linalg.norm(pointB - pointA)
                #z = -np.array(poses[i, :]) / np.linalg.norm(poses[i, :])
                #z[2] = 0
                rx = math.atan2(-vec_[1], vec_[2])
                py = math.asin(vec_[0])
                yz = 0
                rpy_[i, :] = [rx, py, yz]
        elif towards == "fixed":
            for i in range(rpy_.shape[0]):
                rpy_[i, :] = fixed_orientation
        else:
            for i in range(rpy_.shape[0]):
                pointA = poses[i,:]
                pointB = [0,0,0]
                vec_ = np.array(pointB - pointA)/np.linalg.norm(pointB - pointA)
                #z = -np.array(poses[i,:])/np.linalg.norm(poses[i,:])
                rx = math.atan2(-vec_[1], vec_[2])
                py = math.asin(vec_[0])
                yz = 0 
                rpy_[i, :] = [rx, py, yz]

        poses = np.concatenate((poses, rpy_), axis=1)

        return poses

def main(args=None):
    rclpy.init(args=args)
    lattice_publisher = PoseArrayPublisher()
    rclpy.spin(lattice_publisher)
    lattice_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
