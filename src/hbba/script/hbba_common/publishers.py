#!/usr/bin/env python3
# Code taken from https://github.com/introlab/hbba_lite
import rospy

from hbba_common.filter_states import OnOffHbbaFilterState, ThrottlingHbbaFilterState

"""
This file provides the functions to create a publisher that can be used to filter the topic data 
(used for behavior/perception activation). The activation can be triggered by calling a service /TOPICNAME/filter_state
"""

class _HbbaPublisher:
    def __init__(self, topic_name, data_class, filter_state_class, state_service_name, queue_size):
        if state_service_name is None:
            state_service_name = topic_name + '/filter_state'

        self._filter_state = filter_state_class(state_service_name)
        self._publisher = rospy.Publisher(topic_name, data_class, queue_size=queue_size)

    def publish(self, msg):
        if self._filter_state.check():
            self._publisher.publish(msg)

    @property
    def is_filtering_all_messages(self):
        return self._filter_state._is_filtering_all_messages

    def on_filter_state_changed(self, callback):
        self._filter_state.on_changed(callback)


class OnOffHbbaPublisher(_HbbaPublisher):
    def __init__(self, topic_name, data_class, state_service_name=None, queue_size=None):
        super(OnOffHbbaPublisher, self).__init__(topic_name, data_class, OnOffHbbaFilterState,
                                                 state_service_name, queue_size)


class ThrottlingHbbaPublisher(_HbbaPublisher):
    def __init__(self, topic_name, data_class, callback=None, state_service_name=None, queue_size=None):
        super(ThrottlingHbbaPublisher, self).__init__(topic_name, data_class, ThrottlingHbbaFilterState,
                                                      state_service_name, queue_size)
