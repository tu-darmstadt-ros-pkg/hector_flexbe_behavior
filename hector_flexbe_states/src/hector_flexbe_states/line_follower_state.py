#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from line_follower_msgs.msg import FollowLineAction, FollowLineGoal, FollowLineFeedback, ErrorCodes

from actionlib_msgs.msg import GoalID, GoalStatus
from rospy import Time
from dynamic_reconfigure.client import Client

'''
Created on 14.04.2019

@author: Katrin Becker
'''


class LineFollowerState(EventState):
    '''
    The robot follows a line on the ground.

    -- timeout_sec      float64         Timeout for LineFollower
    

    ># camera_topic      string         Camera topic used for line following
    ># drive_backwards   bool           True if robot drives backwards
	># speed            float64         Speed of the robot

    #< drive_backwards   bool           If robot droves backwards

    <= reached 					Robot has reached the end of line
    <= failed                   Robot has lost line or an error occurred
    '''

    def __init__(self, timeout_sec=10):
        '''
        Constructor
        '''
        super(LineFollowerState, self).__init__(outcomes=['reached','failed'],
                                                  input_keys=['camera_topic', 'drive_backwards', 'speed'])


        self._action_topic = '/line_detector_node/follow_line'
        self._action_client = ProxyActionClient({self._action_topic: FollowLineAction})

        #self._dynrec = Client("/line_follower_image_proc", timeout=10) # TODO correctly set up dynamic reconfigure
        #self._defaultspeed = 0.1

        self._timeout_sec = timeout_sec

        self._reached = False
        self._failed  = False


    def execute(self, userdata):
        '''
        Execute this state
        '''

        if self._failed:
            return 'failed'
        if self._reached:
            return 'reached'

        if self._action_client.has_result(self._action_topic):
            result = self._action_client.get_result(self._action_topic)

            #self._dynrec.update_configuration({'speed': self._defaultspeed}) # TODO correctly set up dynamic reconfigure

            if result.result.val == ErrorCodes.SUCCESS or result.result.val == ErrorCodes.LINE_ENDED:
                self._reached = True
                return 'reached'
            else:
                self._failed = True

                if result.result.val == 1:
                    Logger.logwarn('Line lost!')
                elif result.result.val == 3:
                    Logger.logwarn('Control failure!')
                elif result.result.val == 4:
                    Logger.logwarn('Preempted!')
                else:
                    Logger.logwarn('Timed out!')

                Logger.logwarn('LineFollowing failed!')
                return 'failed'


    def on_enter(self, userdata):

        # TODO correctly set up dynamic reconfigure
        #speedValue = self._dynrec.get_configuration(timeout=0.5)
        #if speedValue is not None:
        #    self._defaultspeed = speedValue['speed']

        #self._dynrec.update_configuration({'speed': userdata.max_speed})

        self._startTime = Time.now()

        self._failed = False
        self._reached = False


        action_goal = FollowLineGoal()
        action_goal.options.desired_max_speed = userdata.speed
        action_goal.options.desired_max_yaw_rate = 0.9
        action_goal.options.timeout_sec = self._timeout_sec

        action_goal.options.camera_topic = userdata.camera_topic
        action_goal.options.drive_backwards = userdata.drive_backwards


        try:
            self._action_client.send_goal(self._action_topic, action_goal)
        # resp = self.set_tolerance(goal_id, 0.2, 1.55)
        except Exception as e:
            Logger.logwarn('Failed to options:\n%(err)s' % {
                'err': str(e)
            })
            self._failed = True


    def on_stop(self):
        try:
            if self._action_client.is_available(self._action_topic) \
                    and not self._action_client.has_result(self._action_topic):
                self._action_client.cancel(self._action_topic)
        except:
            # client already closed
            pass

    def on_exit(self, userdata):
        try:
            if self._action_client.is_available(self._action_topic) \
                    and not self._action_client.has_result(self._action_topic):
                self._action_client.cancel(self._action_topic)
        except:
            # client already closed
            pass

    def on_pause(self):
        self._action_client.cancel(self._action_topic)

    def on_resume(self, userdata):
        self.on_enter(userdata)
