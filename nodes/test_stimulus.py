#!/usr/bin/env python
"""
Can use the following ROS parameters:
- olf/odor_pin (required)
- olf/pid_duration (default=60)
"""

import rospy
from std_msgs.msg import Header

from stimuli.msg import Sequence, Transition, State, DefaultState
from stimuli.srv import LoadDefaultStates, LoadSequence, LoadSequenceRequest
import stimuli.util as u


class TestStimLoader:
    def __init__(self):
        rospy.init_node('test_stimulus_loader')

        rospy.loginfo('test_stimulus_loader waiting for services')
        defaults_service_name = 'load_defaults'
        sequence_service_name = 'load_seq'
        rospy.wait_for_service(defaults_service_name)
        rospy.wait_for_service(sequence_service_name)
        load_defaults = rospy.ServiceProxy(defaults_service_name,
            LoadDefaultStates
        )
        load_next_sequence = rospy.ServiceProxy(sequence_service_name,
            LoadSequence
        )

        # to allow arduino to get parameters before services are called
        # (so that debug flag can be in effect during services)
        rospy.sleep(8.0)

        duration_s = rospy.get_param('olf/pid_duration', 60)

        odor = rospy.get_param('olf/odor_pin')
        pins = [odor]

        separate_balance = rospy.get_param('olf/separate_balances', True)
        if separate_balance:
            balance = rospy.get_param('olf/balance')
            assert balance != odor, 'balance and odor pin must be different'
            pins.append(balance)

            # all the valves should have their control pins LOW by default (0v)
            balance_normally_flowing = \
                rospy.get_param('olf/balance_normally_flowing', True)

        default_states = [DefaultState(p, False) for p in pins]

        # TODO factor
        rospy.loginfo('sending default states')
        try:
            h = Header()
            h.stamp = rospy.Time.now()
            resp = load_defaults(h, default_states)

        except rospy.ServiceException as exc:
            rospy.logerr("Service load_defaults failed: " + str(exc))
        rospy.loginfo('finished sending default states')
        #

        h = Header()
        now = rospy.Time.now()
        h.stamp = now
        start = now # + rospy.Duration.from_sec(0.5)
        end = now + rospy.Duration.from_sec(duration_s)

        ms_on = rospy.get_param('olf/odor_pulse_ms', 5000)
        ms_off = rospy.get_param('olf/post_pulse_ms', 5000)

        high = Transition(t=start, s=State(ms_on=ms_on, ms_off=ms_off))
        transitions = [high]

        if separate_balance:
            if balance_normally_flowing:
                balance_transition = high
            else:
                low = State(ms_on=ms_off, ms_off=ms_on)
                # TODO check / test
                balance_transition = [Transition(
                    t=start + rospy.Duration.from_sec(ms_on / 1000), s=low
                )]

            transitions.append(balance_transition)
            rospy.loginfo('balance on pin {}'.format(u.pin_label(balance)))

        seq = Sequence(start=start, end=end, pins=pins, seq=transitions)
        this_pin_high = LoadSequenceRequest(header=h, seq=seq,
            pins_to_signal=[]
        )
        rospy.loginfo('odor on pin {}'.format(u.pin_label(odor)))

        try:
            resp = load_next_sequence(this_pin_high)
        except rospy.ServiceException as exc:
            rospy.logerr('Service load_next_sequence failed: ' + str(exc))

        rate = rospy.Rate(0.2)
        while (not rospy.is_shutdown()) and rospy.Time.now() < end:
            rate.sleep()


if __name__ == '__main__':
    s =  TestStimLoader()

