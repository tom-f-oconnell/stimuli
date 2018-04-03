#!/usr/bin/env python

from __future__ import print_function

import random
import datetime
import time
import os
import sys
import pickle

import numpy as np
import rospy
from std_msgs.msg import Header

from stimuli.msg import Sequence, Transition, State, DefaultState
from stimuli.srv import LoadSequenceRequest
from stimuli_loader import StimuliLoader


def get_params(param_dict, required_params, default_params):
    """Fills param_dict values with the values each key has on the ROS parameter
    server. Raises errors if these invalid combinations of parameters are
    found:
        - some, but not all, of required_params
        - any default_params, but not all required_params

    Args:
        param_dict (dict): 
        required_params (set): 
        default_params (dict):

    Returns whether any parameters were found.
    """
    def get_params_helper(params_to_get):
        found = set()
        not_found = set()

        if type(params_to_get) is dict:
            defaults = True
        else:
            defaults = False

        for k in params_to_get:
            try:
                v = rospy.get_param(k)
                assert not k in param_dict, '{} already in param_dict'.format(k)
                param_dict[k] = v
                found.add(k)
            except KeyError as e:
                not_found.add(k)
                if defaults:
                    param_dict[k] = params_to_get[k]

        return found, not_found

    def missing_params_err_msg():
        error_msg = ''
        for p in required_params:
            error_msg += '{}\n'.format(p)
        error_msg += '\nThe following were missing:\n'
        for p in required_not_found:
            error_msg += '{}\n'.format(p)
        return error_msg
        
    # TODO provide mechanism to validate types / ranges as well?

    required_found, required_not_found = get_params_helper(required_params)
    if len(required_found) > 0 and len(required_found) < len(required_params):
        error_msg = 'If any of the following parameters are specified, ' + \
            'they all must be:\n'
        error_msg += missing_params_err_msg()

        # TODO is the convention to have the error message describe the whole
        # problem, or to print stuff out before the error and then have a short
        # error message?
        raise ValueError(error_msg)

    defaults_found, _ = get_params_helper(default_params)
    # if len(required_found) > 0, there would already have been an error
    if len(defaults_found) > 0 and len(required_found) == 0:
        error_msg = 'Since {} were set, the '.format(defaults_found) + \
            'following are required:\n'
        error_msg += missing_params_err_msg()

        raise ValueError(error_msg)

    return len(defaults_found) > 0 or len(required_found) > 0


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
# TODO load parameter yaml directly? (for testing without ROS running)
# TODO add defaults for all and document this stuff externally
params = dict()
# TODO break out the validation checks here into another module, so that I can
# add another command to just validate the configuration?
###############################################################################
# Parameters common to experiments with and without reinforcement
###############################################################################
required_params = {
    'olf/left_pins',
    'olf/right_pins',
    'olf/odor_pulse_ms',
    'olf/post_pulse_ms',
    'olf/prestimulus_delay_s',
    'olf/test_duration_s',
    'olf/beyond_posttest_s'
}

optional_params = {
    'olf/balance_normally_flowing': None,
    'olf/left_balance': None,
    'olf/right_balance': None,
    'olf/odor_side_order': 'alternating'
}

get_params(params, required_params, optional_params)

left_pins = params['olf/left_pins']
right_pins = params['olf/right_pins']
odor_pulse_ms = params['olf/odor_pulse_ms']
post_pulse_ms = params['olf/post_pulse_ms']
prestimulus_delay_s = params['olf/prestimulus_delay_s']
test_duration_s = params['olf/test_duration_s']
beyond_posttest_s = params['olf/beyond_posttest_s']

balance_normally_flowing = params['olf/balance_normally_flowing']
left_balance = params['olf/left_balance']
right_balance = params['olf/right_balance']

odor_side_order = params['olf/odor_side_order']

if balance_normally_flowing is None:
    if left_balance is None or right_balance is None:
        raise ValueError('need to specify balance pins (olf/left_balance and ' +
            'olf/right_balance) when olf/balance_normally_flowing is set')
    else:
        balance_normally_flowing = True

if not (left_balance is None or right_balance is None):
    balances = True
else:
    balances = False

if not (odor_side_order == 'alternating' or odor_side_order == 'random'):
    raise ValueError("olf/odor_side_order must be either 'random' or " + 
        "'alternating'")

###############################################################################
# Parameters unique to experiments with reinforcement (training)
###############################################################################
required_training_params = {
    'olf/training_blocks',
    'olf/pretest_to_train_s',
    'olf/train_duration_s',
    'olf/inter_train_interval_s',
    'olf/train_to_posttest_s',
    # TODO document what means always on or always off, among other things
    'zap/shock_ms_on', 
    'zap/shock_ms_off'
}
# If any of these are specified, fail if not all of the required parameters are
# set.
optional_training_params = {
    # TODO maybe just set this conditional on some operant/classical option?
    # TODO should also be an error if this is set when none of the other
    # training
    'olf/train_one_odor_at_a_time': True,

    # TODO update definitions other places to use these parameter names
    'zap/left_control_pin': None,
    'zap/right_control_pin': None,
    # TODO still support left / right
    'zap/control_pin': None
}

have_training_params = \
    get_params(params, required_training_params, optional_training_params)

if have_training_params:
    left_shock = params['zap/left_control_pin']
    right_shock = params['zap/right_control_pin']
    all_shock = params['zap/control_pin']
    # TODO delete me
    rospy.logwarn('left_shock: {}'.format(left_shock))
    rospy.logwarn('right_shock: {}'.format(right_shock))
    rospy.logwarn('all_shock: {}'.format(all_shock))

    have_left = not (left_shock is None)
    have_right = not (right_shock is None)
    have_one_pin = not (all_shock is None)

    if ((not (have_left or have_right or have_one_pin)) or
        (have_left and not have_right) or (have_right and not have_left) or 
        have_one_pin and (have_left or have_right)):

        raise ValueError('only set either the parameters zap/control_pin or' + 
            ' both zap/left_control_pin and zap/right_control_pin')
    elif have_one_pin:
        one_pin_shock = True
    else:
        one_pin_shock = False

    training_blocks = params['olf/training_blocks']
    pretest_to_train_s = params['olf/pretest_to_train_s']
    train_duration_s = params['olf/train_duration_s']
    inter_train_interval_s = params['olf/inter_train_interval_s']
    train_to_posttest_s = params['olf/train_to_posttest_s']
    shock_ms_on = params['zap/shock_ms_on']
    shock_ms_off = params['zap/shock_ms_off']

    train_one_odor_at_a_time = params['olf/train_one_odor_at_a_time']

###############################################################################
# Parameters unique to experiments with NO reinforcement
# (only repeating the same test, over and over)
###############################################################################
required_testonly_params = {
    'olf/testing_blocks',
    'olf/inter_test_interval_s'
}

have_testonly_params = get_params(params, required_testonly_params, dict())

if have_testonly_params:
    testing_blocks = params['olf/testing_blocks']
    inter_test_interval_s = params['olf/inter_test_interval_s']

if ((have_testonly_params and have_training_params) or 
    not (have_testonly_params or have_training_params)):

    raise ValueError(
        'must set either test-only or training parameters, and not both.')

###############################################################################

# TODO maybe just reset at like 5am or check for experiments in last few hours, 
# to use the same mappings after midnight, but in the same workday
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
        generate_odor_to_pin_connections = False

    elif c.lower() == 'n':
        generate_odor_to_pin_connections = True

    elif c.lower() == 'd':
        rospy.logwarn('Deleting ' + daily_connections_filename)
        os.remove(daily_connections_filename)
        generate_odor_to_pin_connections = True

    else:
        raise ValueError('invalid choice')

else:
    generate_odor_to_pin_connections = True

if generate_odor_to_pin_connections:
    rospy.loginfo('did not find saved odors and odor->pin mappings to load')
    #odors = list(odor_panel)
    # TODO put in config file
    mock = ('paraffin (mock)', 0)
    odors = [('4-methylcyclohexanol', -2), ('3-octanol', -2)]
    #odors = rospy.get_param('olf/odors', ['UNSPECIFIED_ODOR'])
    # TODO fix
    #odors = list(map(lambda x: (x, np.nan), odors))
    #odors.append(mock)

    print(odors, len(odors), left_pins)

    left_pins = random.sample(left_pins, len(odors))
    right_pins = random.sample(right_pins, len(odors))

    # TODO improve (?)
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

# TODO open a new terminal for these / GUI, to make it not get hard to see among
# mess printed to terminal? (do I still care it is logged through ROS
# facilities?)
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
    # TODO rename (because sometimes we don't have any training trials?)
    reinforced, unreinforced = random.sample(odors, 2)

if len(odors) > 1 and training_blocks != 0:
    rospy.loginfo('pairing shock with '  + str(reinforced))
    rospy.loginfo('unpaired ' + str(unreinforced))

# include this in optional parameter dict above?
# TODO also only start recording when this is done? (when using this with
# multi_tracker) (maybe preferably, just make sure tracking is started before
# starting stimuli)
# TODO when should i wait / not wait? need the param?
wait_for_keypress = rospy.get_param('olf/wait_for_keypress', False)
if wait_for_keypress:
    raw_input('Press Enter when the odor vials are connected.')

odors2left_pins = dict(zip(odors, left_pins))
odors2right_pins = dict(zip(odors, right_pins))

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
        odor_pulse = State(ms_on=odor_pulse_ms, ms_off=post_pulse_ms)
        # don't need explicit low transition because default state is low
        # but end time in Sequence message must be set correctly
        transition = [Transition(self.current_t0, odor_pulse)]

        expanded_pins = []
        seq = []
        if balances:
            balance_pins = [left_balance, right_balance]

            if balance_normally_flowing:
                balance_transition = transition
            else:
                # TODO TODO fix. needs to be complement of odor_pulse, at all
                # times
                raise NotImplementedError
                #balance_transition = [Transition(self.current_t0 + \
                #    rospy.Duration(odor_pulse_ms / 1000.0), )]

            for p in balance_pins:
                # TODO won't this len always be 1? is this a bug? why did i do
                # it this way?
                expanded_pins.extend(len(balance_transition) * [p])
                seq.extend(balance_transition)

        # TODO check / test this part
        if train and train_one_odor_at_a_time:
            # TODO probably rename this flag to indicate its use here as well?
            # at least document
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

        for p in pins:
            expanded_pins.extend(len(transition) * [p])
            seq.extend(transition)

        return expanded_pins, seq


    def shock_transitions(self):
        """
        TODO
        """
        square_wave = State(ms_on=shock_ms_on, ms_off=shock_ms_off)
        transition = Transition(self.current_t0, square_wave)

        if train_one_odor_at_a_time:
            # when train_one_odor_at_a_time is True, current_side_is_left
            # indicates whether the current training epoch will use the
            # reinforced_odor or the unreinforced_odor
            if self.current_side_is_left:
                if one_pin_shock:
                    rospy.loginfo('ONE PIN SHOCK (in shock_transitions)')
                    # TODO what happens on the Arduino if it is given a
                    # duplicate of a transition? does it behave appropriately or
                    # fail?
                    return [all_shock], [transition]
                else:
                    rospy.loginfo('TWO PIN SHOCK')
                    return [left_shock, right_shock], [transition, transition]
            else:
                return [], []

        else:
            if one_pin_shock:
                raise ValueError('must specify zap/left_shock and ' +
                    'zap/right_shock, if train_one_odor_at_a_time is False')

            if self.current_side_is_left:
                return [left_shock], [transition]
            else:
                return [right_shock], [transition]


    def test(self):
        start = self.current_t0
        end = start + rospy.Duration(test_duration_s)
        self.current_t0 = end

        pins, seq = self.odor_transitions()
        if odor_side_order == 'alternating':
            # TODO TODO rename current_side_is_left to indicate that it also
            # controls whether the current training trial uses the reinforced
            # odor or the non-reinforced odor. it does, right?
            self.current_side_is_left = not self.current_side_is_left 

        elif odor_side_order == 'random':
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

        if odor_side_order == 'alternating':
            self.current_side_is_left = not self.current_side_is_left 

        elif odor_side_order == 'random':
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

# Get the (initial) time, before each call to one of the generator's functions
# internally increments this time.
t0_sec = gen.current_t0.to_sec()

if have_training_params:
    # TODO factor out code to accommodate variable number of training blocks for
    # testing blocks?
    trial_structure = [gen.delay(prestimulus_delay_s),
                       gen.test(),
                       gen.delay(pretest_to_train_s)] + \
                      ((flatten([[f(), gen.delay(inter_train_interval_s)] for f
                        in [gen.train] * (training_blocks - 1)]) + \
                      [gen.train()]) if training_blocks > 0 else []) + \
                      [gen.delay(train_to_posttest_s),
                       gen.test(),
                       gen.delay(beyond_posttest_s)]

    # TODO handle this concurrently w/ trial_structure generation, to make sure
    # they are always consistent?
    epoch_labels = ['test'] + ['train'] * training_blocks + ['test']

elif have_testonly_params:
    trial_structure = [gen.delay(prestimulus_delay_s)] + \
                      ((flatten([[f(), gen.delay(inter_test_interval_s)] for f
                        in [gen.test] * (testing_blocks - 1)]) + \
                      [gen.test()]) if training_blocks > 0 else []) + \
                      [gen.test(),
                       gen.delay(beyond_posttest_s)]
    epoch_labels = ['test']  * testing_blocks

# TODO even if not printed to /rosout, is this saved by default?
rospy.logdebug('trial_structure', trial_structure)

# low_pins = pins that default to low (0v)
# high_pins = pins that default to high (5v)
# pins should be in the default state during the intertrial interval

low_pins = left_pins + right_pins
if have_training_params:
    if one_pin_shock:
        low_pins += [all_shock]
    else:
        low_pins += [left_shock, right_shock]

high_pins = []
if balances:
    if balance_normally_flowing:
        low_pins += [left_balance, right_balance]
    else:
        high_pins = [left_balance, right_balance]

default_states = [DefaultState(p, True) for p in set(high_pins)] + \
                 [DefaultState(p, False) for p in set(low_pins)]

# TODO make sure this node stays alive until the current_t0 after the trial
# structure has been evaluated

###############################################################################

# So this won't publish to /rosout unless this is called from a node that has
# been "set up properly" with init_node. Is that required for it to be displayed
# on the screen? Or saved in log files elsewhere?
rospy.loginfo('Stimuli should finish at ' + \
    datetime.datetime.fromtimestamp(gen.current_t0.to_sec()).strftime(\
    '%Y-%m-%d %H:%M:%S'))
rospy.loginfo(str(gen.current_t0.to_sec() - t0_sec) + ' seconds')

output_base_dir = '.'

# TODO test this
if save_stimulus_info:
    # TODO get rid of multi_tracker prefix? implement way of managing experiment
    # ids / data output in metatools (though the latter might need to be done in
    # each package generating data?)?
    experiment_basename = \
        rospy.get_param('multi_tracker/experiment_basename', None)

    if experiment_basename is None:
        # TODO fix node number thing
        experiment_basename = \
            time.strftime('%Y%m%d_%H%M%S_N1', time.localtime())
        rospy.set_param('multi_tracker/experiment_basename', \
            experiment_basename)

    # TODO do i want to save this if this program is Ctrl-C'd? probably?
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

