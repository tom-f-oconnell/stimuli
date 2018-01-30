#!/usr/bin/env python

from __future__ import print_function

import random
import datetime
import time
import os
import pickle

import rospy
from std_msgs.msg import Header
from stimuli.msg import Sequence, Transition, State, DefaultState
from stimuli.srv import LoadSequenceRequest
from stimuli_loader import StimuliLoader

import numpy as np

import sys

# TODO do i ever want to train the same flies on different pairs of odors
# sequentially?  or maybe expose them to some odors / some sequence of odors
# first (/ after?)?

# TODO provide a function in this file that stimuli_loader can call and pass a
# parameter indicating where to find the containing file to stimuli_loader?
# then move the node maintenance stuff to there?
rospy.init_node('stimuli')

# TODO make parameter for this?
save_stimulus_info = True

# TODO store as effective dilution given flow conditions / mixing ratios?
'''
odor_panel = {'paraffin (mock)': (0,),
              '4-methylcyclohexanol': (-2,),
              '3-octanol': (-2,)}
'''
'''
odor_panel = {'4-methylcyclohexanol': (-2,),
              '3-octanol': (-2,)}
'''
# TODO way to load parameter yaml directly? (for testing without ROS running)

reinforced_odor_side_order = rospy.get_param('olf/reinforced_odor_side_order')
train_one_odor_at_a_time = \
    rospy.get_param('olf/train_one_odor_at_a_time', False)

training_blocks = rospy.get_param('olf/training_blocks')
prestimulus_delay_s = rospy.get_param('olf/prestimulus_delay_s')
test_duration_s = rospy.get_param('olf/test_duration_s')
pretest_to_train_s = rospy.get_param('olf/pretest_to_train_s')
train_duration_s = rospy.get_param('olf/train_duration_s')
inter_train_interval_s = rospy.get_param('olf/inter_train_interval_s')
train_to_posttest_s = rospy.get_param('olf/train_to_posttest_s')
beyond_posttest_s = rospy.get_param('olf/beyond_posttest_s')

left_pins = rospy.get_param('olf/left_pins')
right_pins = rospy.get_param('olf/right_pins')
separate_balances = rospy.get_param('olf/separate_balances', False)
left_balance = rospy.get_param('olf/left_balance', None)
right_balance = rospy.get_param('olf/right_balance', None)

if separate_balances and (left_balance is None or right_balance is None):
    raise ValueError('need to specify balance pins')

balance_normally_flowing = rospy.get_param('olf/balance_normally_flowing', True)

shock_ms_on = rospy.get_param('zap/shock_ms_on', 0)
shock_ms_off = rospy.get_param('zap/shock_ms_off', 1)

# TODO handle both?
#left_shock = rospy.get_param('zap/left')
#right_shock = rospy.get_param('zap/right')
all_shock = rospy.get_param('zap/all_pin')

# TODO TODO how to deal w/ symmetry re: sides? (blocks pick a random side to
# start on?)

# TODO for now, just save sides to a separate file to be loaded by that ROS node

###############################################################################

daily_connections_filename = '.' + time.strftime('%Y%m%d', time.localtime()) + \
    '_mappings.p'
if os.path.isfile(daily_connections_filename):
    c = raw_input('Found saved mappings from today. Load them? ' + \
        '([y]es/[n]o/[d]elete them.')
    if c.lower() == 'y':
        rospy.loginfo('loading odors and odor->pin mappings from ' + \
            daily_connections_filename)
        with open(daily_connections_filename, 'rb') as f:
            odors, left_pins, right_pins = pickle.load(f)
        generate = False

    elif c.lower() == 'n':
        generate = True

    elif c.lower() == 'd':
        rospy.logwarn('Deleting ' + daily_connections_filename)
        os.remove(daily_connections_filename)
        generate = True

    else:
        raise ValueError('invalid choice')

else:
    generate = True

if generate:
    rospy.loginfo('did not find saved odors and odor->pin mappings to load')
    #odors = list(odor_panel)
    # TODO put in config file
    mock = ('paraffin (mock)', 0)
    #odors = [('4-methylcyclohexanol', -2), ('3-octanol', -2)]
    odors = rospy.get_param('olf/odors', ['UNSPECIFIED_ODOR'])
    # TODO fix
    odors = list(map(lambda x: (x, np.nan), odors))
    #odors.append(mock)

    print(odors, len(odors), left_pins)

    left_pins = random.sample(left_pins, len(odors))
    right_pins = random.sample(right_pins, len(odors))

    # TODO improve
    if len(odors) > 1:
        with open(daily_connections_filename, 'wb') as f:
            rospy.loginfo('temporarily saving odors and odor->pin mappings ' + \
                'to ' + daily_connections_filename + ', for reuse in other ' + \
                'experiments today.')
            pickle.dump([odors, left_pins, right_pins], f)
            # TODO verify it saved correctly?

# TODO
# randomly break stimuli into groups fitting into the number of 
# valves we have on the rig
# ***if odors are ever to be mixed, need to be connected simultaneously***

# assign them to random pins / ports
# needs |pins| >= |odors|
# (samples without replacement)
# TODO load pickle if it is there / if same day? prompt?

rospy.loginfo('Left pins:')
for pin, odor_pair in sorted(zip(left_pins, odors), key=lambda x: x[0]):
    rospy.loginfo(str(pin) + ' -> ' + str(odor_pair))

rospy.loginfo('Right pins:')
for pin, odor_pair in sorted(zip(right_pins, odors), key=lambda x: x[0]):
    rospy.loginfo(str(pin) + ' -> ' + str(odor_pair))

# TODO how to support no mock? (potentially just 1 odor in total)
# TODO fix hack
if len(odors) == 1:
    reinforced = odors[0]
    unreinforced = None
    rospy.logwarn('not pulsing mock on opposite side')
else:
    # TODO rename
    reinforced, unreinforced = random.sample(odors, 2)

if len(odors) > 1 and train_blocks != 0:
    rospy.loginfo('pairing shock with '  + str(reinforced))
    rospy.loginfo('unpaired ' + str(unreinforced))

# TODO also only start recording when this is done?
# TODO when should i wait / not wait? need the param?
wait_for_keypress = rospy.get_param('olf/wait_for_keypress', False)
if wait_for_keypress:
    raw_input('Press Enter when the odor vials are connected.')

odors2left_pins = dict(zip(odors, left_pins))
odors2right_pins = dict(zip(odors, right_pins))

# TODO pause until person has connected stuff?

###############################################################################

class StimuliGenerator:
    # TODO maybe just add all the parameters that are scoped in to the init?
    # not sure it would make sense to put this class in a fn otherwise, but
    # i'd like to have a function here, so i could call it both in a __main__
    # and from my testing functions
    def __init__(self):
        # TODO switch everything over to milliseconds relative to start? provide
        # the option?  or Durations? or zero time here somehow?
        self.current_t0 = rospy.Time.now()

        # only do this for 'alternating' mode?
        self.current_side_is_left = random.choice([True, False])

    # currently just on all the time. maybe i want something else?
    def odor_transitions(self, train=False):
        """
        TODO
        """
        # setting ms_on to 1 to emphasize duration is determined by end time set
        # of the Sequence and the pin will go low if that is the DefaultState
        # for the pin
        high = State(ms_on=1, ms_off=0)
        # don't need explicit low transition because default state is low
        # but end time in Sequence message must be set correctly
        transition = [Transition(self.current_t0, high)]

        expanded_pins = []
        seq = []
        if separate_balances:
            balance_pins = [left_balance, right_balance]

            if balance_normally_flowing:
                balance_transition = transition
            else:
                low = State(ms_on=0, ms_off=1)
                balance_transition = [Transition(self.current_t0, low)]

            for p in balance_pins:
                # TODO won't this len always be 1? is this a bug? why did i do
                # it this way?
                expanded_pins.extend(len(balance_transition) * [p])
                seq.extend(balance_transition)

        # TODO check / test this part
        if train and train_one_odor_at_a_time:
            # TODO probably rename this flag to indicate its use here as well?
            if self.current_side_is_left:
                pins = [odors2left_pins[reinforced], \
                    odors2right_pins[reinforced]]
            else:
                pins = [odors2left_pins[unreinforced], \
                    odors2right_pins[unreinforced]]

        else:
            if self.current_side_is_left:
                if not unreinforced is None:
                    pins = [odors2left_pins[reinforced], \
                        odors2right_pins[unreinforced]]
                else:
                    pins = [odors2left_pins[reinforced]]
            else:
                if not unreinforced is None:
                    pins = [odors2left_pins[unreinforced], \
                        odors2right_pins[reinforced]]
                else:
                    pins = [odors2right_pins[reinforced]]

        # TODO will need to make sure pins aren't turned into a set later
        for p in pins:
            expanded_pins.extend(len(transition) * [p])
            seq.extend(transition)

        return expanded_pins, seq


    # TODO how to handle shocking + presenting reinforced odor on both sides?
    def shock_transitions(self):
        """
        TODO
        """
        square_wave = State(ms_on=shock_ms_on, ms_off=shock_ms_off)
        transition = Transition(self.current_t0, square_wave)

        # TODO check this part
        if train_one_odor_at_a_time:
            if self.current_side_is_left:
                return [all_shock], [transition, transition]
            else:
                return [], []

        else:
            # TODO fail if using all_shock
            raise NotImplementedError('hardcoding zap/all_shock in config now')
            # TODO add support (w/ parameter?) for shocking both sides?
            if self.current_side_is_left:
                return [left_shock], [transition]
            else:
                return [right_shock], [transition]


    def test(self):
        start = self.current_t0
        end = start + rospy.Duration(test_duration_s)
        self.current_t0 = end

        pins, seq = self.odor_transitions()
        if reinforced_odor_side_order == 'alternating':
            self.current_side_is_left = not self.current_side_is_left 

        elif reinforced_odor_side_order == 'random':
            self.current_side_is_left = random.choice([True, False])

        pins_to_signal = []
        seq = Sequence(start, end, pins, seq)
        return LoadSequenceRequest(Header(), seq, pins_to_signal)


    # TODO share most of this function with test?
    def train(self):
        start = self.current_t0
        end = start + rospy.Duration(train_duration_s)
        self.current_t0 = end

        odor_pins, odor_seq = self.odor_transitions(train=True)
        shock_pins, shock_seq = self.shock_transitions()
        pins = odor_pins + shock_pins
        seq = odor_seq + shock_seq

        if reinforced_odor_side_order == 'alternating':
            self.current_side_is_left = not self.current_side_is_left 

        elif reinforced_odor_side_order == 'random':
            self.current_side_is_left = random.choice([True, False])

        pins_to_signal = []
        seq = Sequence(start, end, pins, seq)
        return LoadSequenceRequest(Header(), seq, pins_to_signal)


    def delay(self, delay_s):
        ros_delay = rospy.Duration(delay_s)
        self.current_t0 = self.current_t0 + ros_delay
        # do we have a copy constructor?
        return rospy.Time.from_sec(self.current_t0.to_sec())


flatten = lambda l: [item for sublist in l for item in sublist]

gen = StimuliGenerator()
t0_sec = gen.current_t0.to_sec()

# TODO repeat code to accommodate variable number of blocks for test
trial_structure = [gen.delay(prestimulus_delay_s), \
                   gen.test(), \
                   gen.delay(pretest_to_train_s)] + \
                  ((flatten([[f(), gen.delay(inter_train_interval_s)] for f \
                    in [gen.train] * (training_blocks - 1)]) + \
                  [gen.train()]) if training_blocks > 0 else []) + \
                  [gen.delay(train_to_posttest_s), \
                   gen.test(), \
                   gen.delay(beyond_posttest_s)]
epoch_labels = ['test'] + ['train'] * training_blocks + ['test']

# TODO even if not printed to /rosout, is this saved by default?
rospy.logdebug('trial_structure', trial_structure)

# low_pins = pins that default to low (0v)
# high_pins = pins that default to high (5v)
# pins should be in the default state during the intertrial interval
low_pins = left_pins + right_pins + [left_shock, right_shock]
high_pins = []
if separate_balances:
    if balance_normally_flowing:
        low_pins += [left_balance, right_balance]
    else:
        high_pins = [left_balance, right_balance]

default_states = [DefaultState(p, True) for p in set(high_pins)] + \
                 [DefaultState(p, False) for p in set(low_pins)]

# TODO make sure this node stays alive until the current_t0 after the trial
# structure has been evaluated

###############################################################################

# can i do this from outside of a node?
rospy.loginfo('Stimuli should finish at ' + \
    datetime.datetime.fromtimestamp(gen.current_t0.to_sec()).strftime(\
    '%Y-%m-%d %H:%M:%S'))
rospy.loginfo(str(gen.current_t0.to_sec() - t0_sec) + ' seconds')

output_base_dir = '.'

# TODO test this
if save_stimulus_info:
    # TODO get rid of multi_tracker prefix
    experiment_basename = \
        rospy.get_param('multi_tracker/experiment_basename', None)

    if experiment_basename is None:
        # TODO fix node number thing
        experiment_basename = \
            time.strftime('%Y%m%d_%H%M%S_N1', time.localtime())
        rospy.set_param('multi_tracker/experiment_basename', \
            experiment_basename)

    output_dir = os.path.join(output_base_dir, experiment_basename)
    filename = os.path.join(output_dir, experiment_basename + '_stimuli.p')
    rospy.loginfo('Trying to save save stimulus info to ' + filename)

    # TODO expand path?
    if not os.path.isdir(output_dir):
        os.mkdir(output_dir)

    # TODO check / test success
    with open(filename, 'wb') as f:
        pickle.dump((odors2left_pins, odors2right_pins, default_states, \
            trial_structure), f)
else:
    rospy.logwarn('Not saving generated trial structure ' + \
        '/ pin to odor mappings!')

stimuli_loader = StimuliLoader(default_states, trial_structure, \
    epoch_labels=epoch_labels)

