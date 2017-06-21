#!/usr/bin/env python
"""
Baxter RSDK IKin & FKin hybrid force/position PID control
"""
import argparse
import struct
import sys
import copy
import ipdb


import rospy
import rospkg
import geometry_msgs

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)   
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
#-----F kin & I kin-----
from baxter_pykdl import baxter_kinematics
import baxter_interface
#-----import pid--------
import PID
import time
import matplotlib.pyplot as plt
import numpy 
from scipy.interpolate import spline
#--------------------
#---------------------
class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.0, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService" # maybe this is 
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable() # enable a robot , qin kan 48 hang.Next jump to line 261 

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):# open gripper
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        '''
        pose is position.x    =  0.6
                position.y    = -0.2
                position.z    = -0.12
                orientation.x =  0.0
                orientation.y =  1.0
                orientation.z =  0.0
                orientation.w =  6.12323399574e-17
        '''
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)
        #rospy.sleep(0.5)
        '''
        approach.position.z = approach.position.z - 0.05
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)
        rospy.sleep(0.5)
        
        approach.position.z = approach.position.z - 0.05
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)
        rospy.sleep(0.5)
        
        approach.position.z = approach.position.z - 0.03
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)
        rospy.sleep(0.5)
        
        approach.position.z = approach.position.z - 0.01
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)
        rospy.sleep(0.5)
        '''
#---my class function---    
    def _dragging(self, pose, delta_xx = 0.0, delta_yy = 0.0, delta_zz = 0.0):
        '''
        pose is position.x    =  0.6
                position.y    = -0.2
                position.z    = -0.12
                orientation.x =  0.0
                orientation.y =  1.0
                orientation.z =  0.0
                orientation.w =  6.12323399574e-17
        '''
        dragging = copy.deepcopy(pose)
        dragging.position.x = dragging.position.x + delta_xx
        dragging.position.y = dragging.position.y + delta_yy
        dragging.position.z = dragging.position.z + delta_zz + self._hover_distance
        joint_angles = self.ik_request(dragging)
        self._guarded_move_to_joint_position(joint_angles)
        #rospy.sleep(0.5)        
        #print pose      
#-----------------------    
    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x 
        ik_pose.position.y = current_pose['position'].y 
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y 
        ik_pose.orientation.z = current_pose['orientation'].z 
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def _init_pose(self, pose):
        '''
        pose is position.x    =  0.6
                position.y    = -0.2
                position.z    = -0.12
                orientation.x =  0.0
                orientation.y =  1.0
                orientation.z =  0.0
                orientation.w =  6.12323399574e-17
        '''
        # open the gripper
        #self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        #self._servo_to_pose(pose)
        # close gripper
        #self.gripper_close()
        #rospy.sleep(0.5)
        # retract to clear object
        #self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        rospy.sleep(1)
        self._retract()
        
#----------functions----------
#---define a pid function---
def test_pid(P = 0.2,  I = 0.0, D= 0.0, L=100):
    
    pid = PID.PID(P, I, D)

    pid.SetPoint=0.0
    pid.setSampleTime(0.01)

    END = L
    feedback = 0

    feedback_list = []
    time_list = []
    setpoint_list = []

    for i in range(1, END):  # i is time
        pid.update(feedback) # this feedback is the last feedback
        output = pid.output
        if pid.SetPoint > 0:
            feedback = feedback + (output - (1/i))
        if i>9:
            pid.SetPoint = 1
        time.sleep(0.02)

        feedback_list.append(feedback)
        setpoint_list.append(pid.SetPoint)
        time_list.append(i)
        time_sm = numpy.array(time_list)
        time_smooth = numpy.linspace(time_sm.min(), time_sm.max(), 300)
        feedback_smooth = spline(time_list, feedback_list, time_smooth)

        plt.plot(time_smooth, feedback_smooth)
        plt.plot(time_list, setpoint_list)
        plt.xlim((0, L))
        plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('TEST PID')

        plt.ylim((1-0.5, 1+0.5))

        plt.grid(True)
        plt.show()
        
def change_order(pose):
    pose_changeOrder = Pose()
    # position      
    pose_changeOrder.position.x = pose[0]
    pose_changeOrder.position.y = pose[1]
    pose_changeOrder.position.z = pose[2]
    # orientation
    pose_changeOrder.orientation.x = pose[3]
    pose_changeOrder.orientation.y = pose[4]
    pose_changeOrder.orientation.z = pose[5]
    pose_changeOrder.orientation.w = pose[6]  
    return pose_changeOrder
#----callback function---------------
t = 1   # t is time   
P = [0.89, 0.89, 0.89]#0.89, 0.1, 0.0001
I = [0.1, 0.1, 0.1]
D = [0.0001, 0.0001, 0.0001]
force_PIDoutput = numpy.array([0.0, 0.0, 0.0])
P_array = numpy.array(P)
I_array = numpy.array(I)
D_array = numpy.array(D)
pid = PID.PID(P, I, D)
pid.SetPoint = numpy.array([0.0, 0.0, 0.0])
pid.setSampleTime(0.01)
def getWrench(data):
    #force = copy.deepcopy(data)
    #this is output of the whole system
    force = numpy.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
    '''
    if((force.all() > -0.5) and (force.all() < 0.5)):
        force[0] = 0.0
        force[1] = 0.0
        force[2] = 0.0
    '''               
    pid.update(force) # this force is the last feedback
    output = pid.output
    force[0] = force[0] + (output[0] - (1/t))
    force[1] = force[1] + (output[1] - (1/t))
    force[2] = force[2] + (output[2] - (1/t))   
    global t
    t = t + 1
    time.sleep(0.01) 
    # give a flag of delta_x
    if (pid.output[0] > 0):        
        force_PIDoutput[0] = 1        
    elif (pid.output[0] < 0):
        force_PIDoutput[0] = -1       
    elif (pid.output[0] == 0):
        force_PIDoutput[0] = 0       
        # give a flag of delta_y
    elif (pid.output[1] > 0):           
        force_PIDoutput[1] = 1
    elif (pid.output[1] < 0):
        force_PIDoutput[1] = -1
    elif (pid.output[1] == 0):
        force_PIDoutput[1] = 0
        # give a flag of delta_z
    elif (pid.output[2] > 0):              
        force_PIDoutput[2] = 1
    elif (pid.output[2] < 0):
        force_PIDoutput[2] = -1
    elif (pid.output[2] == 0):
        force_PIDoutput[2] = 0
    #rospy.sleep(0.5) 
#-----main-------
#----------------
def main():
    """RSDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    ipdb.set_trace()
    rospy.init_node("pa_box")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    # Remove models from the scene on shutdown
    # rospy.on_shutdown(delete_gazebo_models)
    
    # Wait for the All Clear from emulator startup
    #rospy.wait_for_message("/robot/sim/started", Empty)
    #load_gazebo_models()
    limb = 'right'
    rKin = baxter_kinematics('right')
    hover_distance = 0.0# meters
    # Starting Joint angles for right arm
    starting_joint_angles = {'right_w0': -0.6699952259595108,
                             'right_w1': 1.030009435085784,
                             'right_w2': 0.4999997247485215,
                             'right_e0': -0.189968899785275,
                             'right_e1': 1.9400238130755056,
                             'right_s0': 0.08000397926829805,
                             'right_s1': -0.9999781166910306}
    pnp = PickAndPlace(limb, hover_distance)  #(right, 0.15) 
    # An orientation for gripper fingers to be overhead and parallel to the obj
    '''
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
    '''                        
    #block_pose is the pose of the cube on the table, which is a list 
    block_poses = list()
    
    #object position with a tranform offset between gripper and male part.
    object_pose = Pose()
    
    #the position of female box
    object_pose.position.x = 0.6 
    object_pose.position.y = 0 
    object_pose.position.z = -0.115 - 0.005 + 0.15

    #RPY = 0 pi 0
    object_orientation = Quaternion(
                                     x=0.0,
                                     y=1.0,
                                     z=0.0,
                                     w=6.123233995736766e-17
                                   )    
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses.append(
                       Pose(
                            position    = Point(x =object_pose.position.x, y =object_pose.position.y, z =object_pose.position.z),
                            orientation = object_orientation
                           )
                      )    
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(
                       Pose(
                            position    = Point(x=0.6, y=-0.2, z=-0.115-0.005),
                            orientation = object_orientation
                           )
                       )
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    idx = 0 #You can add idx
    #ipdb.set_trace()
    print("\ngo to init pose...")
    pnp._init_pose(block_poses[idx])
    
    print("\nTerning on the FT senser...")
    rospy.Subscriber('/wrench/filtered/right', geometry_msgs.msg.WrenchStamped, getWrench)
    # tranlate force to position
    delta_x = 0
    delta_y = 0
    delta_z = 0
    while not rospy.is_shutdown():     
        #rospy.wait_for_message('/wrench/filtered/right', geometry_msgs.msg.WrenchStamped)
        pose_current = rKin.forward_position_kinematics() # get current pose
        pose_changeOrder = change_order(pose_current)
        print("\ntranslating force data...")
        delta_x = 0.001 * force_PIDoutput[0]
        delta_y = 0.01 * force_PIDoutput[1]
        delta_z = 0.001 * force_PIDoutput[2]
        pnp._dragging(pose_changeOrder ,delta_x, delta_y, delta_z)
        if((delta_x == 0) and (delta_y == 0) and (delta_z == 0)):
            # if all xyz force is 0 , go to the init pose. 
            pnp._init_pose(block_poses[idx])                                   
    return 0

if __name__ == '__main__':
    sys.exit(main())
