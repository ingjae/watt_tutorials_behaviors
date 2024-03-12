#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger


class UserdataSumToKeyState(EventState):
    '''
    Example for a state to show how to use userdata and access the behavior parameters.

    -- num_1		int 	Numeric value to be added.
    -- num_2		int 	Numeric value to be added.
    #> sum_result	int 	Result of the addition.

    <= continue 			No failure occurred, go on with the next state.
    <= failed 				Example for a failure outcome.

    '''

    def __init__(self, num_1, num_2):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(UserdataSumToKeyState, self).__init__(outcomes = ['continue', 'failed'],
                                                    output_keys = ['sum_result'])

        # Store state parameter for later use.
        self._num_1 = num_1
        self._num_2 = num_2

        self._start_time = None


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        # In this example, log the time of entering the state.
        time_until_state_on_enter = (rospy.Time.now() - self._start_time).to_sec()
        Logger.loginfo('Time taken from beginning of behavior to entering this state : %.1f seconds.' % time_until_state_on_enter)


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        # In this example, we add the two numbers and store the result in the output key.
        sum_result = self._num_1 + self._num_2
        Logger.loginfo('Sum result : %d' % sum_result)
        userdata.sum_result = sum_result

        return "continue"


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        pass # Nothing to do in this example.


    def on_start(self):
        # This method is called when the behavior is started.
        # If possible, it is generally better to initialize used resources in the constructor
        # because if anything failed, the behavior would not even be started.

        # In this example, we use this event to set the correct start time.
        self._start_time = rospy.Time.now()


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        pass # Nothing to do in this example.
        
