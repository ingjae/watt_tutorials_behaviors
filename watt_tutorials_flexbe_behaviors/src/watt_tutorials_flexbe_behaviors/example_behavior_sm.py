#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from watt_tutorials_flexbe_states.key_topic_sum_to_topic_state import KeyTopicSumToTopicState
from watt_tutorials_flexbe_states.logging_key_times_state import LoggingKeyTimesState
from watt_tutorials_flexbe_states.userdata_sum_to_key_state import UserdataSumToKeyState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Mar 12 2024
@author: Myungsang Park
'''
class ExampleBehaviorSM(Behavior):
	'''
	This is a simple example for a behavior.
	'''


	def __init__(self):
		super(ExampleBehaviorSM, self).__init__()
		self.name = 'Example Behavior'

		# parameters of this behavior
		self.add_parameter('param_a', 3)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		param_b = 4
		# x:783 y:40, x:383 y:190
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:74 y:24
			OperatableStateMachine.add('example_state_1',
										UserdataSumToKeyState(num_1=self.param_a, num_2=param_b),
										transitions={'continue': 'example_state_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'sum_result': 'sum_result'})

			# x:324 y:24
			OperatableStateMachine.add('example_state_2',
										KeyTopicSumToTopicState(),
										transitions={'continue': 'example_state_3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_num': 'sum_result', 'sum_result': 'log_print'})

			# x:574 y:24
			OperatableStateMachine.add('example_state_3',
										LoggingKeyTimesState(log_msg="HELLow FlexBE"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'log_print': 'log_print'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
