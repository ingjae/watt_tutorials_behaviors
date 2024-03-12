#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from std_msgs.msg import Int8

class KeyTopicSumToTopicState(EventState):
    '''
    Example for a state to show how to use keys and access a topic.

    ># input_num	int 	Numeric value to be added.
    #> sum_result	int 	Result of the addition.

    <= continue 			No failure occurred, go on with the next state.
    <= failed 				Example for a failure outcome.

    '''

    def __init__(self):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(KeyTopicSumToTopicState, self).__init__(outcomes = ['continue', 'failed'],
                                                      input_keys = ['input_num'],
                                                      output_keys = ['sum_result'])

        self.number_topic_name = '/number'
        self.sub_number = ProxySubscriberCached({self.number_topic_name: Int8})
        self.number = None

        self.result_topic_name = '/result'
        self.pub_result = ProxyPublisher({self.result_topic_name: Int8})
        self.result = Int8()


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        pass # Nothing to do in this example.


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        # There is no callback for the subscriber, so we need to check if a message has been received.
        if self.sub_number.has_msg(self.number_topic_name):
            self.number = self.sub_number.get_last_msg(self.number_topic_name)

        # If subscriber has not received a message, "self.number" will be "None".
        # else, "self.number" will be the last received message.
        if self.number is not None:
            self.result.data = userdata.input_num + self.number.data
            Logger.loginfo('Sum result : %d' % self.result.data)
            self.pub_result.publish(self.result_topic_name, self.result)
            userdata.sum_result = self.result.data
            return 'continue'
        

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
        
