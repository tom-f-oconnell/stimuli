#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import time
import os
import math
import pickle

import rospy
from std_msgs.msg import Header

from stimuli.msg import Sequence, Transition, State, DefaultState
from stimuli.srv import LoadSequenceRequest
from stimuli.stimuli_loader import StimuliLoader
import stimuli.util as u


rospy.init_node('nagel_stimuli')

required_params = {
    'olf/odor_pins',
    'olf/balance_pins',

    'olf/prestimulus_delay_s',

    # TODO TODO switch to these after figuring out [necessity of/] rounding
    #'olf/pre_pulse_s',
    #'olf/odor_pulse_s',
    #'olf/post_pulse_s',
    'olf/pre_pulse_ms',
    'olf/odor_pulse_ms',
    'olf/post_pulse_ms',

    'olf/poststimulus_delay_s',

    # TODO TODO maybe just use something like 'n_trials' instead, especially if
    # trials are long and only contain single pulses
    # (otherwise probably just round [up?] duration so an even number of trials
    # fit in?)
    'olf/duration_s',

    # TODO need params for blocks / inter block intervals too?
    # (using this for now, to mirror trial structure gaps in two_choice.py, as
    # well as copy the similar ~6s in the Nagel methods)
    'olf/intertrial_delay_s',
}

optional_params = {
    'olf/phasic_odor_high_ms': None,
    # TODO test this default value gets passed through correctly
    'olf/phasic_odor_low_ms': 0
}


params = dict()
u.get_params(params, required_params, optional_params)

# TODO assert these are non-overlapping and of equal length
odor_pins = params['olf/odor_pins']
balance_pins = params['olf/balance_pins']

prestimulus_delay_s = params['olf/prestimulus_delay_s']

pre_pulse_ms = params['olf/pre_pulse_ms']
odor_pulse_ms = params['olf/odor_pulse_ms']
post_pulse_ms = params['olf/post_pulse_ms']

# TODO test both None (default) and passed
high_ms = params['olf/phasic_odor_high_ms']
low_ms = params['olf/phasic_odor_low_ms']

poststimulus_delay_s = params['olf/poststimulus_delay_s']

duration_s = params['olf/duration_s']

intertrial_delay_s = params['olf/intertrial_delay_s']

# To ensure that the stimulus program doesn't start until after the tracking is
# already up and running, as there is currently no other mechanism implemented
# to sequence them (The ROS launch file starts both at the same time. The
# tracking can take a few seconds to get going, and the user may need to
# manually enter / confirm ROIs.).
#'''
wait_for_keypress = rospy.get_param('olf/wait_for_keypress', True)
if wait_for_keypress:
    u.ros_friendly_raw_input('Press Enter to start stimulus program.\n')
#'''
#u.ros_friendly_raw_input('Press Enter to start stimulus program.\n')

# TODO TODO TODO actually check / use this.
balance_valves_inverted = True

# low_pins = pins that default to low (0v)
# high_pins = pins that default to high (5v)
# pins should be in the default state during the intertrial interval
#low_pins = odor_pins
low_pins = odor_pins + balance_pins

# Assuming the same NC outputs of the same 3-way valves are used for both the
# odors and the balances. This means that the balance valves will be driven
# whenever the odor valves are not, which could lead to them heating up /
# wearing out differently if the duty cycle is much more or less than 50%.
# However, doing it this way probably leads to more symmetric flow dynamics
# (between the odor and balance valves, so the ON/OFF switches are more likely
# to have similar transients).
# TODO TODO check whether default_states also end up determining valve states
# before / after sequences. if so, maybe still include balance_pins as part of
# low_pins, and just explicitly set them to states opposite the odor_pins
# (+ check behavior between loaded sequences)
# (to prevent overheating if left powered on after, mainly)
#high_pins = balance_pins

'''
default_states = [DefaultState(p, True) for p in set(high_pins)] + \
                 [DefaultState(p, False) for p in set(low_pins)]
'''
# TODO TODO TODO fix hack (make conditional on invert var at least)
default_states = [DefaultState(p, False) for p in set(low_pins)]


t0 = rospy.Time.now()
t_curr = u.copy_rostime(t0)

trial_duration_s = (pre_pulse_ms + odor_pulse_ms + post_pulse_ms) / 1000.0
pre_pulse_s = pre_pulse_ms / 1000.0
odor_pulse_s = odor_pulse_ms / 1000.0
post_pulse_s = post_pulse_ms / 1000.0

# True = hack
lump_pre_pulse_into_delays = True
if lump_pre_pulse_into_delays:
    t_curr += rospy.Duration(prestimulus_delay_s + pre_pulse_s)
else:
    t_curr += rospy.Duration(prestimulus_delay_s)

trial_structure = [u.copy_rostime(t_curr)]

# TODO calculate given there are n trials and n - 1 intertrial delays
# (shouldn't make a big difference though...)
n_trials = int(math.ceil(duration_s / (trial_duration_s + intertrial_delay_s)))
#print('n_trials:', n_trials)
#print('actual time:', prestimulus_delay_s +
#    trial_duration_s * n_trials + intertrial_delay_s * (n_trials - 1) +
#    poststimulus_delay_s
#)

if lump_pre_pulse_into_delays:
    intertrial_delay = rospy.Duration(post_pulse_s + intertrial_delay_s +
        pre_pulse_s
    )
else:
    intertrial_delay = rospy.Duration(post_pulse_s + intertrial_delay_s)

# TODO TODO fix hack / at least make conditional on invert bool flag
odor_pins = odor_pins + balance_pins

for i in range(n_trials):
    start = u.copy_rostime(t_curr)

    # TODO TODO actually handle balances (in not invert case. invert case ok.)!
    # need to fix firmware!

    if lump_pre_pulse_into_delays:
        t_start = start
    else:
        # (the firmware might not currently do anything with this time
        # anyway...)
        # TODO TODO TODO design tests to see if start times are used!!
        # (stagger a few pins and listen?)
        t_start = start + rospy.Duration(pre_pulse_s)

    # TODO test both cases
    if high_ms is None:
        odor_pulse = State(ms_on=odor_pulse_ms, ms_off=low_ms)
    else:
        odor_pulse = State(ms_on=high_ms, ms_off=low_ms)

    transitions = [Transition(t_start, odor_pulse)] * len(odor_pins)

    # In both cases post_pulse_s is now in one of the delays, to try to make it
    # easier to optionally have the odor pulses pulsed, using ms_on+ms_off <
    # total odor presentation length per trial.
    if lump_pre_pulse_into_delays:
        t_curr += rospy.Duration(odor_pulse_s)
    else:
        t_curr += rospy.Duration(pre_pulse_s + odor_pulse_s)

    end = u.copy_rostime(t_curr)
    # 'transitions' = seq.seq
    seq = Sequence(start, end, odor_pins, transitions)

    pins_to_signal = []
    trial_structure += [LoadSequenceRequest(Header(), seq, pins_to_signal)]

    if i == n_trials - 1:
        break

    t_curr += intertrial_delay
    trial_structure += [u.copy_rostime(t_curr)]


# TODO check whether i had this >0 in any of my / kristina's experiments
# (i did in mine, but maybe just to check behavior... check if i ever had it 0 /
# if there are problems w/ it being 0)
# Adding post_pulse_s now since I moved this from the ms_off in the loop above
# to the rospy.Duration object created at the end of each iteration above (via 
# change to intertrial_delay, so it also includes post_pulse_s), but I believe
# the loop exits before the last one would be appended.
t_curr += rospy.Duration(post_pulse_s + poststimulus_delay_s)
trial_structure += [u.copy_rostime(t_curr)]

# So this won't publish to /rosout unless this is called from a node that has
# been "set up properly" with init_node. Is that required for it to be displayed
# on the screen? Or saved in log files elsewhere?
rospy.loginfo('Stimuli should finish at ' + u.readable_rostime(t_curr))
rospy.loginfo(str(t_curr.to_sec() - t0.to_sec()) + ' seconds')

###############################################################################

# TODO get rid of multi_tracker prefix? implement way of managing experiment
# ids / data output in metatools (though the latter might need to be done in
# each package generating data?)?
# TODO TODO or maybe factor multi_tracker into module w/ a util function for
# generatign these, then just import that here? way to do that that doesn't make
# multi_tracker as much of a hard dependency of this stimulus code?
experiment_basename = \
    rospy.get_param('multi_tracker/experiment_basename', None)

if experiment_basename is None:
    # might need this for stimuli_only:=True case though...
    experiment_basename = \
        time.strftime('%Y%m%d_%H%M%S', time.localtime())
    rospy.set_param('multi_tracker/experiment_basename',
        experiment_basename
    )

output_base_dir = '.'
output_dir = os.path.join(output_base_dir, experiment_basename)

if not os.path.isdir(output_dir):
    os.mkdir(output_dir)

pickle_fname = os.path.join(output_dir, experiment_basename + '_stimuli.p')
with open(pickle_fname, 'wb') as f:
    pickle.dump((default_states, trial_structure), f)

###############################################################################

# TODO maybe *do* use the epoch_labels kwarg if randomly interleaving "blank"
# trials, as mentioned in Nagel paper
StimuliLoader(default_states, trial_structure)

