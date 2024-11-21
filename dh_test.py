import  numpy
from visual_kinematics.RobotSerial import *
from math import pi

#                        d,     a,  alpha,  theta
dh_params2 = np.array([[0.077,  0.,     -pi*0.5,  0.],
                    	[0.,     0.13,  0.,     -1.3852],
                  	    [0.,    0.124,  0.,     1.3852],   
                    	[0., 0.126,     0.,      0.]])
robot = RobotSerial(dh_params2)

#####
#  construct a frame using ZYX euler angle and translation vector
@staticmethod
def from_euler_3(euler_3, t_3_1):
  r_3_3 = Rotation.from_euler("ZYX", euler_3, degrees=False).as_matrix()
  return Frame.from_r_3_3(r_3_3, t_3_1)
#####

# xyz = np.array([[0.28127], [0.], [0.5]])
# abc = np.array([0. , 0., 0.])
# end = Frame.from_euler_3(abc, xyz)
# robot.inverse(end)
# print("inverse is successful: {0}".format(robot.is_reachable_inverse))
# print("axis values: \n{0}".format(robot.axis_values))
# robot.show()


#####

# xyz = np.array([[0.28127], [0.], [0.5]])
# abc = np.array([0.25 * pi, 0., 0.])
# end = Frame.from_euler_3(abc, xyz)
# robot.inverse(end)
# print("inverse is successful: {0}".format(robot.is_reachable_inverse))
# print("axis values: \n{0}".format(robot.axis_values))
# robot.show()

### 3    

xyz = np.array([[0.0], [-0.16], [0.05]])
# zxy
abc = np.array([0., 0., 0.])
end = Frame.from_euler_3(abc, xyz)
robot.inverse(end)
print("inverse is successful: {0}".format(robot.is_reachable_inverse))
print("axis values: \n{0}".format(robot.axis_values))
robot.show()

