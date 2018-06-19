#!/usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import JointState
import modern_robotics as mr
import sawyer_MR_description as s_des
import io_util


class MainVelCtrl:
    def __init__(self):
        rospy.init_node('main_vel_ctrl')
        self.tf_lis = tf.TransformListener()
        self.pub = rospy.Publisher('/main_sawyer/joint_states', JointState, queue_size=10)
        self.Kp = 1*np.eye(6)
        self.Ki = 1*np.eye(6)
        self.B_list = s_des.Blist
        self.M = s_des.M
        self.main_js = JointState()
        self.main_js.name = ['right_j0', 'head_pan', 'right_j1', 'right_j2', \
                            'right_j3', 'right_j4', 'right_j5', 'right_j6']
        self.main_js.position = np.ndarray.tolist(np.zeros(8)) #init main_js
        # rospy.sleep(1)
        catch = 0
        tf_received = False
        while not tf_received and not rospy.is_shutdown():
            try:
                (self.p_d, self.Q_d) = self.tf_lis.lookupTransform('/ref_tf/base', '/ref_tf/right_hand', rospy.Time(0))
                # print self.p_d
                tf_received = True
                rospy.sleep(0.1)
            except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException) as e:
                # print "ERROR", e
                continue




    def ctrl_r(self):
        self.cur_theta_list = np.delete(self.main_js.position, 1)
        self.X_d = self.tf_lis.fromTranslationRotation(self.p_d, self.Q_d)
        self.X = mr.FKinBody(self.M, self.B_list, self.cur_theta_list)
        self.X_e = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(self.X), self.X_d)))
        self.V_b = np.dot(self.Kp, self.X_e)
        self.J_b = mr.JacobianBody(self.B_list, self.cur_theta_list)
        self.theta_dot = np.dot(np.linalg.pinv(self.J_b), self.V_b)
        self.delt_theta = self.theta_dot*(1.0/20)
        self.delt_theta = np.insert(self.delt_theta, 1, 0)
        self.main_js.position += self.delt_theta

        # print(self.theta_dot)

        # THETA DOT NEXT


    def pub_main_js(self):
        self.main_js.header.stamp = rospy.Time.now()
        self.pub.publish(self.main_js)
        rate.sleep()



if __name__=='__main__':
    try:
        out = MainVelCtrl()
        rate = rospy.Rate(20)
        out.pub_main_js()
        print "Press any key to begin"
        io_util.wait_for_press()
        while not rospy.is_shutdown():
            out.ctrl_r()
            out.pub_main_js()
    except rospy.ROSInterruptException:
        raise e
