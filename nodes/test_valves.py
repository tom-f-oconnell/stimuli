#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from stimuli.msg import Sequence, Transition, State, DefaultState
from stimuli.srv import LoadDefaultStates, LoadSequence, LoadSequenceRequest

class ValveTester:
    def __init__(self):
        rospy.init_node('valve_tester')

        self.pin2name = dict(zip(range(54, 70), \
            ['A' + str(i) for i in range(16)]))
        rospy.loginfo('valve_tester waiting for services')
        defaults_service_name = 'load_defaults'
        sequence_service_name = 'load_seq'
        rospy.wait_for_service(defaults_service_name)
        rospy.wait_for_service(sequence_service_name)
        load_defaults = rospy.ServiceProxy(defaults_service_name, \
            LoadDefaultStates)
        load_next_sequence = rospy.ServiceProxy(sequence_service_name, \
            LoadSequence)

        # to allow arduino to get parameters before services are called
        # (so that debug flag can be in effect during services)
        rospy.sleep(5.0)

        rospy.loginfo('valve_tester sending default states')
        left_pins = rospy.get_param('olf/left_pins', [])
        right_pins = rospy.get_param('olf/right_pins', [])

        if rospy.get_param('olf/separate_balances', True):
            lb = rospy.get_param('olf/left_balance')
            rb = rospy.get_param('olf/right_balance')
            left_pins.append(lb)
            right_pins.append(rb)
            # TODO also check param that controls balance direction?

        pins_to_test = sorted(left_pins + right_pins)
        if len(pins_to_test) == 0:
            raise ValueError('need some pins to test. specify olf/right_pins '+\
                'and olf/left_pins as ROS parameters that are lists of ' + \
                'integers.')

        # all the valves should have their control pins LOW by default (0v)
        default_states = [DefaultState(p, False) for p in \
            set(pins_to_test)]

        try:
            h = Header()
            h.stamp = rospy.Time.now()
            resp = load_defaults(h, default_states)

        except rospy.ServiceException as exc:
            rospy.logerr("Service load_defaults failed: " + str(exc))
        rospy.loginfo('valve_tester finished sending default states')
        
        block_num = 0
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            for p in pins_to_test:
                h = Header()
                now = rospy.Time.now()
                h.stamp = now
                # TODO why these? why not set via parameters? (diff file though?
                # in package?)
                start = now + rospy.Duration.from_sec(0.75)
                end = now + rospy.Duration.from_sec(2.0)
                high = Transition(t=start, s=State(ms_on=1, ms_off=0))
                seq = Sequence(start=start, end=end, pins=[p], seq=[high])
                this_pin_high = LoadSequenceRequest(header=h, seq=seq, \
                    pins_to_signal=[])

                print('pin ' + str(p))
                try:
                    resp = load_next_sequence(this_pin_high)
                except rospy.ServiceException as exc:
                    rospy.logerr("Service load_next_sequence failed: " + \
                        str(exc))

                # TODO does it always sleep the period, or try to maintain
                # period across all calls?
                rate.sleep()


if __name__ == '__main__':
    s = ValveTester()
