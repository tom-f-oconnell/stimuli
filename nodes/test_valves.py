#!/usr/bin/env python

# TODO TODO document how test_valves.py and test_stimulus.py are different
# (or delete one if not meaningfully diff...)

import rospy
from std_msgs.msg import Header

from stimuli.msg import Sequence, Transition, State, DefaultState
from stimuli.srv import LoadDefaultStates, LoadSequence, LoadSequenceRequest
import stimuli.util as u


class ValveTester:
    def __init__(self):
        rospy.init_node('valve_tester')

        rospy.loginfo('valve_tester waiting for services')
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
        rospy.sleep(5.0)

        left_pins_param = 'olf/left_pins'
        if rospy.has_param(left_pins_param):
            left_pins = rospy.get_param(left_pins_param, [])
            right_pins = rospy.get_param('olf/right_pins', [])

            if rospy.get_param('olf/separate_balances', True):
                lb = rospy.get_param('olf/left_balance')
                rb = rospy.get_param('olf/right_balance')
                left_pins.append(lb)
                right_pins.append(rb)
                # TODO also check param that controls balance direction?

            pins_to_test = sorted(left_pins + right_pins)

        else:
            # Assuming we either have these or the parameters above
            odor_pins = rospy.get_param('olf/odor_pins')
            balance_pins = rospy.get_param('olf/balance_pins', [])

            pins_to_test = sorted(odor_pins + balance_pins)

        if len(pins_to_test) == 0:
            raise ValueError('need some pins to test. specify olf/right_pins '
                'and olf/left_pins as ROS parameters that are lists of '
                'integers.'
            )

        # all the valves should have their control pins LOW by default (0v)
        default_states = [DefaultState(p, False) for p in set(pins_to_test)]

        try:
            h = Header()
            h.stamp = rospy.Time.now()
            rospy.loginfo('valve_tester sending default states')
            resp = load_defaults(h, default_states)

        except rospy.ServiceException as exc:
            rospy.logerr('Service load_defaults failed: ' + str(exc))
            return

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
                this_pin_high = LoadSequenceRequest(header=h, seq=seq,
                    pins_to_signal=[]
                )
                print('pin ' + str(u.pin_label(p)))
                try:
                    resp = load_next_sequence(this_pin_high)
                except rospy.ServiceException as exc:
                    rospy.logerr('Service load_next_sequence failed: ' +
                        str(exc)
                    )
                    return

                # TODO does it always sleep the period, or try to maintain
                # period across all calls?
                rate.sleep()


if __name__ == '__main__':
    s = ValveTester()

