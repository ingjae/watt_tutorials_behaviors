#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger


class LoggingKeyTimesState(EventState):
    '''
    Example for a state to show how to works execute method.

    -- log_msg		string 	Message to be logged.
    ># log_print	int 	Number of times to output log.

    <= continue 			No failure occurred, go on with the next state.
    <= failed 				Example for a failure outcome.

    '''

    def __init__(self, log_msg):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(LoggingKeyTimesState, self).__init__(outcomes = ['continue', 'failed'],
                                                    input_keys = ['log_print'])
        
        self.msg = log_msg
        self.print_count = None


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.
        
        self.print_count = userdata.log_print


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self.print_count is None == False or type(self.print_count) is not int:
            return 'failed'
        elif self.print_count <= 0:
            return 'continue'
        else:
            Logger.logwarn('%s' % self.msg)
            self.print_count -= 1


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        pass # Nothing to do in this example.


    def on_start(self):
        # This method is called when the behavior is started.
        # If possible, it is generally better to initialize used resources in the constructor
        # because if anything failed, the behavior would not even be started.

        pass # Nothing to do in this example.


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        pass # Nothing to do in this example.
        
