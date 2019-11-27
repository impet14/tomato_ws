#!/usr/bin/env python
import rospy
from trac_ik_python.trac_ik import IK

class ConbeIK():
    def __init__(self,urdf_param,LorR,pos_bound,ori_bound):
        # Get URDF from parameter server
        # self._urdf_str = rospy.get_param('/LArm/robot_description')
        self._urdf_str = rospy.get_param(urdf_param)
        # Create IK instance
        self._ik_solver = IK(LorR+"link0",LorR+"EEFlink",urdf_string=self._urdf_str)
        # set the Dof
        self._seed_state = [0.0] * self._ik_solver.number_of_joints
        #set the lower&upper bound
        self._lower_bound, self._upper_bound = self._ik_solver.get_joint_limits()
        #can modify the joint limits
        # self._ik_solver.set_joint_limits([0.0]* self._ik_solver.number_of_joints, upper_bound)
        self._pos_bound = pos_bound
        self._ori_bound = ori_bound
        
    def check_setting(self):
        # check the info of model
        print('ik_solver.base_link  : ',self._ik_solver.base_link)
        print('ik_solver.tip_link   : ',self._ik_solver.tip_link)
        print('ik_solver.joint_names: ',self._ik_solver.joint_names)
        print('lower_bound          : ',self._lower_bound)
        print('upper_bound          : ',self._upper_bound)

    def calculate(self,pos,ori):
        #calculate IK
        result = self._ik_solver.get_ik(self._seed_state,
                        pos[0], pos[1], pos[2],
                        ori[0], ori[1], ori[2], ori[3],
                        self._pos_bound[0], self._pos_bound[1], self._pos_bound[2],  # X, Y, Z bounds
                        self._ori_bound[0], self._ori_bound[1], self._ori_bound[2])  # Rotation X, Y, Z bounds
        return result

if __name__ == '__main__':
    conbe_ik   = ConbeIK('/LArm/robot_description',[0.01,0.01,0.01],[0.1,0.1,0.1])
    target_pos = [0.15,0,0.40]
    target_ori = [0.0, 0.0, 0.0, 1.0]
    result     = conbe_ik.calculate(target_pos,target_ori)
    conbe_ik.check_setting()
    print(result)