import numpy as np
from visual_kinematics.RobotSerial import RobotSerial
from math import pi
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

# Define the DH parameters
# Each row corresponds to a joint/link with [d, a, alpha, theta]
# Note: theta is the joint variable for revolute joints and should be set to 0 in DH parameters
# since it will be replaced by the actual joint angles during forward kinematics.

dh_params2 = np.array([[0.077,  0.,     -pi*0.5,  0.],
                    	[0.,     0.13,  0.,     -1.3852],
                  	    [0.,    0.124,  0.,     1.3852],   
                    	[0., 0.126,     0.,      0.]])

# Initialize the robot with DH parameters
robot = RobotSerial(dh_params2)

# Define a static method to construct a frame using ZYX Euler angles and translation vector
# Assuming the Frame and Rotation classes are correctly imported and used within visual_kinematics
class Frame:
    @staticmethod
    def from_euler_3(euler_3, t_3_1):
        r_3_3 = Rotation.from_euler("ZYX", euler_3, degrees=False).as_matrix()
        return Frame.from_r_3_3(r_3_3, t_3_1)
    
    @staticmethod
    def from_r_3_3(r, t):
        # This method should construct a Frame object from rotation matrix and translation vector
        # Implementation depends on the visual_kinematics library
        # Placeholder implementation:
        return Frame(r, t)
    
    def __init__(self, rotation_matrix, translation_vector):
        self.rotation = rotation_matrix
        self.translation = translation_vector

# Define the joint positions from the header (in radians)
joint_positions = np.array([
    0.0,
    -1.052310824394226,
    0.3574175238609314,
    0.7025632262229919       # joint4
    #0.009986215531826019               # gripper (not used in kinematics)
])
# string : [0.0 , -1.3 ,12. ,3. ]

# Extract the first four joint values for forward kinematics
joint_values = joint_positions[:4]

# Perform Forward Kinematics
# Assuming the RobotSerial class has a forward method that takes joint angles as input
# and computes the end-effector frame.

# Set the joint angles
robot.forward(joint_values)

# Check if forward kinematics was successful
# (Assuming RobotSerial sets an attribute or returns a status)
# Here, we'll assume it always succeeds for simplicity
# You may need to adjust based on the actual library implementation
print("Forward kinematics computation completed.")

# Retrieve the end-effector frame
# Assuming RobotSerial has a method or attribute to get the end-effector frame
# This is a placeholder and should be replaced with actual methods from visual_kinematics


# Optional: Visualize the robot using the provided joint positions
# This assumes that the show() method visualizes the robot's current configuration
robot.show()

# Optional: Plot the robot's configuration using matplotlib for better visualization
# This requires that the visual_kinematics library provides joint positions or a way to extract them
# Here's a basic example assuming robot.joints is a list of joint positions

# Uncomment and modify the following lines based on the actual implementation of visual_kinematics
# joints = robot.joints  # Replace with actual attribute or method to get joint positions

# if hasattr(robot, 'joints'):
#     joints = robot.joints
#     xs = [joint[0] for joint in joints]
#     ys = [joint[1] for joint in joints]
#     zs = [joint[2] for joint in joints]
#     
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     ax.plot(xs, ys, zs, '-o')
#     ax.set_xlabel('X (m)')
#     ax.set_ylabel('Y (m)')
#     ax.set_zlabel('Z (m)')
#     ax.set_title('Robot Configuration')
#     plt.show()
# else:
#     print("Robot joints data not available for plotting.")
