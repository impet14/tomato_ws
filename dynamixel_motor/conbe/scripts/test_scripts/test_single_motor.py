import dxl_move as DXL 
import rospy
from math import fabs
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

if __name__ == '__main__':
    print('start motor test....')
    
    joint = DXL.DXL_CONTROL(node_name='test',control_joint='joint1-2_controller',max_load=0.01)
    joint.open()
    joint.close()
    # joint.close()
    # rospy.sleep(2)
    # joint.move_to_goal(-0.7)
