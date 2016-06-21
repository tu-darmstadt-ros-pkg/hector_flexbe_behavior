#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import PoseStamped
from hector_worldmodel_msgs.srv import GetObjectModel

'''

Created on 28.10.2015

@author: Markus Sigg

'''

class GetObjectPoseState(EventState):

        '''
        Retrieves the pose of an object given by its ID.


        ># object_id                string                        ID of the object of interest.

        #> object_pose                PoseStamped         Pose of the given object with z-axis aligned to objects normal.

        <= done                                                 Desired pose is available.
        <= unknown                                                 The specified object ID is not known.
        <= not_available                                 The preferred option is not available.


        '''

        def __init__(self):

                '''
                Constructor
                '''

                super(GetObjectPoseState, self).__init__(outcomes=['done', 'unknown', 'not_available'],input_keys=['object_id'], output_keys=['object_pose'])

                self._pose = None
                self._failed = False
                self._not_available = False

        def execute(self, userdata):

                '''
                Execute this state
                '''

                if self._failed:
			Logger.loginfo('pose unknown')
                        return 'unknown'

                if self._not_available:
			Logger.loginfo('pose not_available')
                        return 'not_available'

                userdata.object_pose = self._pose
		
		Logger.loginfo('update victim pose')
                return 'done'

        def on_enter(self, userdata):
		

                
		self._failed = False
                self._not_available = True
                _object_name = userdata.object_id

                try :
                        rospy.wait_for_service('/worldmodel/get_object_model', timeout=5.0)
                        _getObjects = rospy.ServiceProxy('/worldmodel/get_object_model', GetObjectModel)
                        _objects = _getObjects().model.objects

                except rospy.ServiceException, e:

                        Logger.logerror(str(e))
                        Logger.logerror("Could not call service '/worldmodel/get_object_model'")
                        self._failed = True

                except rospy.ROSException, te:

                        Logger.logerror(str(te))
                        Logger.logerror("Service not available (timeout)")
                        self._not_available = True

                for _o in _objects:

                        if _o.info.object_id == _object_name:
                                self._not_available = False
                                self._pose = PoseStamped()
                                self._pose.header = _o.header
                                self._pose.pose = _o.pose.pose

                                break

