#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from stimuli.msg import PulseSeq, Pulse, Transition, State, DefaultState
from stimuli.srv import LoadDefaultStates, LoadPulseSeq

# TODO use odor randomizer as input somehow?

def get_test_pulseseq():
    high = State(ms_on=1, ms_off=0)
    transitions = [Transition(rospy.Time.now(), high)]

def test_python_roundtrip_serialization():
    pass
