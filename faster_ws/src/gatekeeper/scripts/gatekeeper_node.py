#!/usr/bin/env python

import time
import math
import numpy as np
from numpy import linalg as LA
from copy import deepcopy

import tf
from snapstack_msgs.msg import Goal, GoalTrajectory, State
from geometry_msgs.msg import Pose, Vector3, Quaternion
from gatekeeper.srv import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply, quaternion_matrix, quaternion_from_matrix

import roslib
import rospy

from utils import getYaw, vector3_to_np
from dynamics import Quadrotor


class Gatekeeper:

    def __init__(self):

        # dt
        self.dt = 0.1 # runs every 50 milliseconds
        
        # construct quadrotor
        self.quad = Quadrotor()

        # initialize plan_committed
        self.plan_committed = GoalTrajectory()

        # construct goal
        self.goal = Goal()
        
        # construct state
        self.state=State()
        self.state.header.frame_id="world"

        # Publishers
        self.pubGoal = rospy.Publisher('goal', Goal, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.run)

        name = rospy.get_namespace()
        self.name = name[1:-1]
        
        rospy.wait_for_service('minDist')
        try:
            self.minDist_srv = rospy.ServiceProxy('minDist', MinDist)
            path = [Vector3(0,0,0)]
            resp = minDist_srv(path)
            print(resp)
        except rospy.ServiceException as e:
            print("minDist service failed: %s" % e)

        # now the timer should kick in

    def state_cb(self, state_msg):
        self.state = state_msg

    def track_nominal(self, plan):
        
        local_state = deepcopy(self.state) # copy over the last state of the quadrotor

        states = [local_state]
        dists = []
        # forward integrate a tracking controller for last_ind 
        for i in range(len(plan.goals)):
            dists.append(LA.norm(vector3_to_np(plan.goals[i].p) - vector3_to_np(local_state.pos)))
            local_state = self.quad.dynamics(local_state, plan.goals[i], plan.dt)

            states.append(local_state)

        print(dists)

        return states

    def append_backup(self, states_nominal, index, dt):

        states = states_nominal[0:(index+1)]
        TB = 10
        hover_goal = Goal()
        hover_goal.p = states[-1].pos
        hover_goal.yaw = getYaw(states[-1])

        for i in range(TB):
            local_state = self.quad.dynamics(states[-1], hover_goal, dt)
            states.append(local_state)

        return states
        


    def create_committed(self, plan_nominal):

        # first perform an ode tracking this nominal
        states_nominal = self.track_nominal(plan_nominal)

        N = len(plan_nominal.goals)

        print(N)

        # step backwards
        for ind in range(N-1, -1, -1):
            print("trying to use %d" % ind)
            states_candidate = self.append_backup(states_nominal, ind, plan_nominal.dt)
            if self.validate(states_candidate):
                # found a validate candidate!
                print("FOUND VALID")
                self.plan_committed.header = plan_nominal.header
                self.plan_committed.dt     = plan_nominal.dt
                self.plan_committed.goals  = plan_nominal.goals[0:(ind+1)]
                return
        


    # def create_committed(self, plan_nominal):

    #     # initialize with the last committed plan
    #     last_candidate = self.plan_committed
    # 
    #     # step through and determine the maximum horizon that is safe
    #     for t in range(len(plan_nominal.goals)):

    #         states, plan_candidate = self.create_candidate(plan_nominal, t)

    #         if not self.validate(states):
    #             # oops. no longer safe - break
    #             break

    #         last_candidate = plan_candidate

    #     # make the last_candidate the committed
    #     self.plan_committed = last_candidate
    #     return

    def validate(self, plan):
        return True

    def plan_whole_cb(self, plan_msg):

        start_time = time.time()

        self.plan_whole = plan_msg

        print("whole")
        if len(self.plan_whole.goals) == 0:
            return

        self.create_committed(self.plan_whole)

        print("plan has %d goals" % len(self.plan_committed.goals))

        end_time = time.time()

        print("whole_cb took: %f seconds" % (end_time - start_time))


    def run(self, timer_event):

        print("timer")

        # check that we have something to track
        if len(self.plan_committed.goals) == 0:
            return

        # determine which index we are in
        now = rospy.Time.now()
        elapsed_s = (now - self.plan_committed.header.stamp).to_sec() 

        ind = int(elapsed_s // self.plan_committed.dt) 
        ind = max(ind, 1) # force it go to atleast the next step
        ind = min(ind, len(self.plan_committed.goals)-1)
        # ind = len(self.plan_committed.goals)-1

        # publish based on the last element of the committed plan
        goal = self.plan_committed.goals[ind]
        goal.header.stamp = self.plan_committed.header.stamp + rospy.Duration(elapsed_s)
        goal.header.frame_id = self.plan_committed.header.frame_id

        print("publishing goal : (%f, %f, %f) %f" % (goal.p.x, goal.p.y, goal.p.z, goal.yaw))

        self.pubGoal.publish(goal)

        return


             

def startNode():
    c = Gatekeeper()


    return


    
    # callbacks
    rospy.Subscriber("faster/plan_whole", GoalTrajectory, c.plan_whole_cb)
    rospy.Subscriber("state", State, c.state_cb)
    # rospy.Subscriber("faster/plan_safe", GoalTrajectory, c.plan_safe_cb)

    rospy.spin()

if __name__ == '__main__':

    ns = rospy.get_namespace()
    try:
        rospy.init_node('relay')
        if str(ns) == '/':
            rospy.logfatal("Need to specify namespace as vehicle name.")
            rospy.logfatal("This is tyipcally accomplished in a launch file.")
            rospy.logfatal("Command line: ROS_NAMESPACE=mQ01 $ rosrun quad_control joy.py")
        else:
            print "Starting gatekeeper node for: " + ns
            startNode()

    except rospy.ROSInterruptException:
        pass
