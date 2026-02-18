#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from rof_flexbe_states.stop_state import StopState
from rof_flexbe_states.twist_state import TwistState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Nov 24 2022
@author: Julian
'''
class move_jetbotSM(Behavior):
	'''
	Lets the simulated jetbot drive around.
	'''


	def __init__(self, node):
		super(move_jetbotSM, self).__init__()
		self.name = 'move_jetbot'

		# parameters of this behavior

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		StopState.initialize_ros(node)
		TwistState.initialize_ros(node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:51 y:79
			OperatableStateMachine.add('Drive',
										TwistState(target_time=5.0, velocity=1.0, rotation_rate=-0.5, cmd_topic='jetbot/cmd_vel'),
										transitions={'done': 'stop'},
										autonomy={'done': Autonomy.Off})

			# x:222 y:143
			OperatableStateMachine.add('stop',
										StopState(timeout=0.5, cmd_topic='jetbot/cmd_vel', odom_topic='jetbot/odom'),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
