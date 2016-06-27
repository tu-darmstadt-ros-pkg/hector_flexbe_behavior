#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyServiceCaller
from ethzasl_icp_mapper.srv import SetMode, GetMode


from smach import CBState



class SetMappingState(EventState):
	'''
	Activate or deactivate mapping.

	-- active 	bool 	Mapping changed to active or inactive.

	<= succeeded 		Mapping changed.

	'''

	def __init__(self, active):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(SetMappingState, self).__init__(outcomes = ['succeeded'])

		#self._mappingTopicSet = '/mapper/set_mode'
		#self._mappingTopicGet = '/mapper/get_mode'
		#self._srvSet = ProxyServiceCaller({self._mappingTopicSet: SetMode})
		#self._srvGet = ProxyServiceCaller({self._mappingTopicGet: GetMode})
        	self.set_mapper_mode = rospy.ServiceProxy('/mapper/set_mode', SetMode)
        	#self.get_mapper_mode = rospy.ServiceProxy('/mapper/get_mode', GetMode)

	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		##if rospy.Time.now() - self._start_time < self._target_time:
		return 'succeeded' # One of the outcomes declared above.
		

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		# The following code is just for illustrating how the behavior logger works.
		# Text logged by the behavior logger is sent to the operator and displayed in the GUI.
		
	
		map_state = userdata.switch
     		#response = ''
     		#try:
		#response = self.get_mapper_mode()
	  	#map_state = response.map	  
	  #print(str(response))
      		#except rospy.ServiceException as exc:
	  		#print("get_mapper_mode Service did not process request: " + str(exc))
	  
      		#try:
	  	resp = self.set_mapper_mode(True, map_state, True)
      		#except rospy.ServiceException as exc:
	  		#print("set_mapper_mode Service did not process request: " + str(exc))
	  
      		#try:
	  	#response = self.get_mapper_mode()
	  	#map_state = response.map
	  	#self._widget.qc_mapping_status_lineEdit.setText('active' if response.map else 'inactive')
	  #print(str(response))
      		#except rospy.ServiceException as exc:
	  		#print("get_mapper_mode Service did not process request: " + str(exc))
	  

        			


	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do in this example.


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		# In this example, we use this event to set the correct start time.
		##self._start_time = rospy.Time.now()

		pass

	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.
		
