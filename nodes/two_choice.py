#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import random
import time
import os
import pickle

import rospy
from std_msgs.msg import Header
import yaml

from stimuli.msg import Sequence, Transition, State, DefaultState
from stimuli.srv import LoadSequenceRequest
from stimuli.stimuli_loader import StimuliLoader
import stimuli.util as u


# TODO provide a function in this file that stimuli_loader can call and pass a
# parameter indicating where to find the containing file to stimuli_loader?
# then move the node maintenance stuff to there?
rospy.init_node('two_choice_stimuli')

# TODO make parameter for this?
save_stimulus_info = True

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
    'olf/beyond_posttest_s',
    'olf/odors'
}

# TODO option to have first side be (deterministically) either left or right?
optional_params = {
    'olf/random_valve_connections': True,
    'olf/save_yaml_stiminfo': True,

    # How to set this would seem to be a matter of whether valves and their
    # control or odor vials are more valuable to you.
    # If this is True, and you're using three port valves (common, NO, and NC)
    # then it implies that EITHER the NO OR the NC port of each valve should be
    # stopped. If balance_normally_flowing is also True, then the normally
    # closed (NC) port of each balance valve should be stopped.
    'olf/balance_normally_flowing': None,
    'olf/left_balance': None,
    'olf/right_balance': None,
    'olf/odor_side_order': 'alternating',
    'olf/initial_odor_side': None,
    # If False, always loads existing mappings, if they are present.
    'olf/prompt_to_regen_connections': False,
    # Relative to the last experiment run.
    'olf/valve_odor_connections_expiration_hr': 6,
    'olf/flanking_solvent_test_blocks': 0,
    'olf/solvent_test_to_first_test_s': None,
    'olf/last_test_to_solvent_test_s': None
}

params = dict()
u.get_params(params, required_params, optional_params)

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

random_valve_connections = params['olf/random_valve_connections']
odors_and_pins = params['olf/odors']

save_yaml_stiminfo = params['olf/save_yaml_stiminfo']

flanking_solvent_test_blocks = params['olf/flanking_solvent_test_blocks']
solvent_test_to_first_test_s = params['olf/solvent_test_to_first_test_s']
last_test_to_solvent_test_s = params['olf/last_test_to_solvent_test_s']

# TODO should one odor in list imply a copy of the balance to be used on the
# opposite side? (maybe if balance_normally_flowing is false, or something like
# that?) or just err?
# TODO test all of these conditions
# TODO document odors parameter format
assert len(odors_and_pins) > 0, \
    'List of odors can not be empty. See docs for format.'

solvent = None
for odor in odors_and_pins:
    if not (type(odor) is dict) or len(odor.keys()) == 0:
        raise ValueError("each entry of 'olf/odors' parameter should parse as"
            " non-empty dictionary"
        )

    if not 'name' in odor.keys():
        raise ValueError("Each 'olf/odors' entry needs a key 'name: X'")

    # TODO convert to effective dilution given flow conditions / mixing ratios?
    if not 'vial_log10_concentration' in odor.keys():
        raise ValueError("Each 'olf/odors' entry needs a key " +
            "'vial_log10_concentration: <negative integer or 0>'")

    for k in odor.keys():
        if k not in {'name', 'vial_log10_concentration',
            'left_pin', 'right_pin', 'against_itself', 'solvent'}:

            raise ValueError('Did not recognize key {}'.format(k) +
                "in an entry under 'olf/odors' parameter."
            )

    if 'solvent' in odor.keys() and odor['solvent']:
        if solvent is not None:
            raise NotImplementedError('Currently only support specifying one '
                'solvent in olf/odors list.'
            )
         
        solvent = (odor['name'], odor['vial_log10_concentration'])

if flanking_solvent_test_blocks > 0:
    if flanking_solvent_test_blocks > 1:
        raise NotImplementedError('Currently only support one solvent only '
            'test on either end.'
        )

    if solvent is None:
        raise ValueError('One entry in olf/odors must include "solvent: True"'
            ', if have flanking solvent tests.'
        )
        if solvent_test_to_first_test_s is None:
            raise ValueError('Specify olf/solvent_test_to_first_test_s')

        if last_test_to_solvent_test_s is None:
            raise ValueError('Specify olf/last_test_to_solvent_test_s')


# TODO TODO TODO infer that random_valve_connections should be false if it is
# not specified, and if left_pin + right_pin are specified for each odor in the
# list
if random_valve_connections:
    for odor in odors_and_pins:
        assert not (('left_pin' in odor) or ('right_pin' in odor)), \
            ('All odors must be randomized if '
            'olf/random_valve_connections is True, so do not specify '
            'left_pin / right_pin for any of the odors in the list under the'
            " 'olf/odors' parameter. Otherwise, set "
            "'olf/random_valve_connections' to False.")

else:
    # TODO TODO support assymetric setups (for Dhruv), where odor may have only
    # either a left_pin or a right_pin...
    for odor in odors_and_pins:
        assert (('left_pin' in odor) and ('right_pin' in odor)), \
            ('If not randomizing connections between odor vials and valves, '
             'you must specify left_pin and right_pin under each element in '
             "the parameter 'olf/odors'.")

if balance_normally_flowing is None:
    balance_normally_flowing = True
else:
    if left_balance is None or right_balance is None:
        raise ValueError('need to specify balance pins (olf/left_balance and '
            'olf/right_balance) when olf/balance_normally_flowing is set'
        )

if not (left_balance is None or right_balance is None):
    balances = True
else:
    balances = False

if not (odor_side_order == 'alternating' or odor_side_order == 'random'):
    raise ValueError("olf/odor_side_order must be either 'random' or "
        "'alternating'"
    )

initial_side = params['olf/initial_odor_side']
if initial_side is not None:
    if odor_side_order == 'random':
        raise ValueError('initial_odor_side only valid for alternating case')

    initial_side = initial_side.lower()

    if not (initial_side in ('left','right')):
        raise ValueError('initial_odor_side must be either left or right')

    # TODO delete after debugging
    print('USING FIXED INITIAL SIDE: {}'.format(initial_side))
    #


# TODO TODO TODO print out any unrecognized params in some namespace(s)
# maybe just olf and zap, maybe consolidate
###############################################################################
# Parameters unique to experiments with reinforcement (training)
###############################################################################
required_training_params = {
    'olf/training_blocks',
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
    'olf/train_with_cs_minus': True,
    # TODO implement support for negative delays?
    'zap/delay_from_odor_onset_ms': 0,

    'zap/power_supply_enable_pin': None,
    # TODO update definitions other places to use these parameter names
    'zap/left_control_pin': None,
    'zap/right_control_pin': None,
    # TODO still support left / right
    'zap/control_pin': None,

    # TODO TODO handle case where this is not present, or provide another means
    # of only having a post test
    'olf/pretest_to_train_s': None,
    'olf/shock_solvent_sometimes': True
}

have_training_params = u.get_params(params, required_training_params,
    optional_training_params
)

all_pins = left_pins + right_pins
if balances:
    all_pins.append(left_balance)
    all_pins.append(right_balance)

if have_training_params:
    left_shock = params['zap/left_control_pin']
    right_shock = params['zap/right_control_pin']
    all_shock = params['zap/control_pin']
    power_supply_enable_pin = params['zap/power_supply_enable_pin']
    if power_supply_enable_pin is not None:
        all_pins.append(power_supply_enable_pin)

    have_left = not (left_shock is None)
    have_right = not (right_shock is None)
    have_one_pin = not (all_shock is None)

    if ((not (have_left or have_right or have_one_pin)) or
        (have_left and not have_right) or (have_right and not have_left) or 
        have_one_pin and (have_left or have_right)):

        raise ValueError('only set either the parameters zap/control_pin or'
            ' both zap/left_control_pin and zap/right_control_pin'
        )
    elif have_one_pin:
        one_pin_shock = True
        all_pins.append(all_shock)
    else:
        one_pin_shock = False
        all_pins += [left_shock, right_shock]

    training_blocks = params['olf/training_blocks']
    
    train_duration_s = params['olf/train_duration_s']
    inter_train_interval_s = params['olf/inter_train_interval_s']
    train_to_posttest_s = params['olf/train_to_posttest_s']

    pretest_to_train_s = params['olf/pretest_to_train_s']

    shock_ms_on = params['zap/shock_ms_on']
    shock_ms_off = params['zap/shock_ms_off']

    train_one_odor_at_a_time = params['olf/train_one_odor_at_a_time']
    train_with_cs_minus = params['olf/train_with_cs_minus']

    if not train_with_cs_minus:
        assert train_one_odor_at_a_time

if params['olf/shock_solvent_sometimes'] and solvent is None:
    raise ValueError('Can not shock solvent sometimes if no solvent '
        'is specified. Add "solvent: True" to the solvent in olf/odors'
    )

if len(all_pins) > len(set(all_pins)):
    raise ValueError('Some pins specified as controlling more than one thing.'
        ' This is unsupported. Fix parameters.'
    )

###############################################################################
# Parameters unique to experiments with NO reinforcement
# (only repeating the same test, over and over)
###############################################################################

# TODO make 1 testing block case supported, and have interval optional there
required_testonly_params = {
    'olf/testing_blocks',
    'olf/inter_test_interval_s'
}

have_testonly_params = u.get_params(params, required_testonly_params, dict())

if have_testonly_params:
    testing_blocks = params['olf/testing_blocks']
    inter_test_interval_s = params['olf/inter_test_interval_s']

if ((have_testonly_params and have_training_params) or 
    not (have_testonly_params or have_training_params)):

    raise ValueError(
        'must set either test-only or training parameters, and not both.'
    )

###############################################################################

odors = [(x['name'], x['vial_log10_concentration']) for x in odors_and_pins]

if not random_valve_connections:
    # should lead to them being indexed as the odors in the list above
    # TODO just move this before / same place as other definitions, so
    # left/right_pins are optional in this case
    # TODO test
    left_pins = [x['left_pin'] for x in odors_and_pins]
    right_pins = [x['right_pin'] for x in odors_and_pins]
    # TODO TODO rename to indicate the order is important, or use a
    # different datatype
else:
    # ../ so it goes in directory that contains experiment group directories
    # (e.g. ~/data, rather than ~/data/<experiment_group_name>), so that it
    # does not need to be copied across experiment groups, and 
    valve_conns_filename = '../.tmp_valve_connections.p'
    abs_valve_conns_filename = os.path.abspath(valve_conns_filename)

    if os.path.isfile(valve_conns_filename):
        map_age_hrs = \
            (time.time() - os.path.getmtime(valve_conns_filename)) / 3600.0

        if map_age_hrs > params['olf/valve_odor_connections_expiration_hr']:

            rospy.loginfo('Cached odor->valve map has expired ' +
                '(age: {} hrs, lifetime: {})'.format(map_age_hrs,
                params['olf/valve_odor_connections_expiration_hr']
            ))
            rospy.logwarn('Deleting ' + abs_valve_conns_filename)
            generate_odor_to_pin_connections = True

        else:
            # Need to check it has valid odors / pins for our current
            # experiment, since this is now shared across experiment groups,
            # which could have different stimulus parameters.
            with open(valve_conns_filename, 'rb') as f:
                saved_odors, saved_left_pins, saved_right_pins = pickle.load(f)

            success = False
            saved_conns_valid = True
            for o in saved_odors:
                if not o in odors:
                    saved_conns_valid = False
                    break

            if saved_conns_valid:
                for p in saved_left_pins:
                    if not p in left_pins:
                        saved_conns_valid = False
                        break
            
            if saved_conns_valid:
                for p in saved_right_pins:
                    if not p in right_pins:
                        saved_conns_valid = False
                        break

            if not saved_conns_valid:
                success = True
                rospy.logwarn('Cached odor->valve map had pins or odors ' +
                    'not in current stimulus_parameters.yaml'
                )
                rospy.logwarn('Deleting ' + abs_valve_conns_filename)
                os.remove(valve_conns_filename)
                generate_odor_to_pin_connections = True

            while not success:
                if params['olf/prompt_to_regen_connections']:
                    c = u.ros_friendly_raw_input(
                        'Found saved valve->odor mapping. Use? [y/n]\n'
                    )
                else:
                    c = 'y'

                if c.lower() == 'y':
                    rospy.loginfo('Using odors and odor->pin mappings from ' +
                        abs_valve_conns_filename
                    )
                    odors = saved_odors
                    left_pins = saved_left_pins
                    right_pins = saved_right_pins

                    generate_odor_to_pin_connections = False

                elif c.lower() == 'n':
                    rospy.logwarn('Deleting ' + abs_valve_conns_filename)
                    os.remove(valve_conns_filename)
                    generate_odor_to_pin_connections = True

                else:
                    rospy.logerr('Invalid choice!')
                    continue

                success = True
    else:
        generate_odor_to_pin_connections = True

    if generate_odor_to_pin_connections:
        rospy.loginfo('did not find saved odors and odor->pin mappings to load')
        # TODO document this behavior (or explicitly save mappings) for people
        # who might want to load the pickle and make sense of it. actually... is
        # ultimately saved pickle better? check that too. (+ more important to
        # document that one)

        # without replacement
        left_pins = random.sample(left_pins, len(odors))
        right_pins = random.sample(right_pins, len(odors))

        # TODO need to also delete if gets shutdown prematurely? register on
        # exit that checks some success flag or something? or gets cancelled
        # during a successful exit?

        with open(valve_conns_filename, 'wb') as f:
            rospy.loginfo('temporarily saving odors and odor->pin mappings '
                'to ' + abs_valve_conns_filename +
                ', for reuse in other experiments today.'
            )
            pickle.dump([odors, left_pins, right_pins], f)
            # TODO verify it saved correctly?

    else:
        # Updates mtime of this file, so that expiration is relative to last
        # run.
        # "If times is None, then the file's access and modified times are
        #  set to the current time."
        os.utime(valve_conns_filename, None)

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
# TODO TODO TODO maybe only generate this from odors2<X>_pins, to minimize risk
# of divergence through code changes
rospy.loginfo('Left pins:')
for pin, odor_pair in sorted(zip(left_pins, odors), key=lambda x: x[0]):
    rospy.loginfo(str(pin) + ' -> ' + str(odor_pair))

rospy.loginfo('Right pins:')
for pin, odor_pair in sorted(zip(right_pins, odors), key=lambda x: x[0]):
    rospy.loginfo(str(pin) + ' -> ' + str(odor_pair))

if len(odors) == 1:
    # TODO if elsewhere i've decided that it is desirable to be able to specify
    # more odor pairings than can be tested in one experiment in config, i need
    # to make that handling consistent with this case.
    reinforced = odors[0]

    if not params['olf/shock_solvent_sometimes'] and reinforced == solvent:
        raise ValueError('Only specified one odor, which was solvent, but ' +
            'also said not to shock solvent. No training can happen.')

    if ('against_itself' in odors_and_pins[0] and
        odors_and_pins[0]['against_itself']):

        unreinforced = odors[0]

    else:
        unreinforced = None
        # TODO implement option to suppress warning?
        # TODO TODO test that training / testing blocks generated after this
        # path are reasoanble
        # TODO one way would be to have two vials off of the balance, and there
        # could be an option for that, but that would probably generate slightly
        # less noise on that side, considering it would only be one valve
        # actuated
        # TODO TODO TODO allow some configuration to allow one odor specified in
        # odor list to be presented on both sides, for no-learning, zero discrim
        # control. how best?
        rospy.logwarn('Only one odor. When presented on one side, two valves'
            ' will click, but on the opposite side, no valves will. Asymmetry.'
        )
else:
    for odor in odors_and_pins:
        if 'against_itself' in odor:
            raise NotImplementedError(
                "currently only support 'against_itself' option for one odor"
            )

    # TODO rename reinforced/unreinforced (because sometimes we don't have any
    # training trials)
    reinforced_odor_candidates = list(odors)
    if have_testonly_params or not params['olf/shock_solvent_sometimes']:
        # TODO test
        reinforced_odor_candidates.remove(solvent)

    assert len(reinforced_odor_candidates) > 0, 'Expected odors to shock'
    reinforced = random.choice(reinforced_odor_candidates)

    unreinforced_odor_candidates = list(odors)
    unreinforced_odor_candidates.remove(reinforced)
    assert len(unreinforced_odor_candidates) > 0, \
        'Expected unreinforced odor candidates.'
    unreinforced = random.choice(unreinforced_odor_candidates)

# TODO delete me
'''
print('unreinforced', unreinforced)
print('reinforced', reinforced)
import sys; sys.exit()
'''
#

if have_training_params:
        # TODO TODO fix to accomodate 'against_itself' case + 1 odor against
        # balance / nothing case
	if len(odors) > 1 and training_blocks != 0:
	    rospy.loginfo('pairing shock with '  + str(reinforced))
	    rospy.loginfo('unpaired ' + str(unreinforced))

# Must be True to ensure that the stimulus program doesn't start until after the
# tracking is already up and running, as there is currently no other mechanism
# implemented to sequence them (The ROS launch file starts both at the same
# time. The tracking can take a few seconds to get going, and the user may
# need to manually enter / confirm ROIs.).
# TODO include this in optional parameter dict above?
# TODO TODO also only start recording when this is done? (when using this with
# multi_tracker) (maybe preferably, just make sure tracking is started before
# starting stimuli)
wait_for_keypress = rospy.get_param('olf/wait_for_keypress', False)
if wait_for_keypress:
    u.ros_friendly_raw_input('Press Enter to start stimulus program.\n')


# TODO TODO TODO make sure randomness isn't broken here, and that the mappings
# instructing the user are those ultimately followed and recorded in the trial
# structure!
odors2left_pins = dict(zip(odors, left_pins))
odors2right_pins = dict(zip(odors, right_pins))

# TODO TODO turn into a test
'''
o1_pins = [p for o, p in odors2left_pins.items() if o == o1] + \
    [p for o, p in odors2right_pins.items() if o == o1]
o2_pins = [p for o, p in odors2left_pins.items() if o == o2] + \
    [p for o, p in odors2right_pins.items() if o == o2]
print('PINS THAT YOU SHOULD HAVE BEEN TOLD TO CONNECT TO {}: {}'.format(o1, o1_pins))
print('PINS THAT YOU SHOULD HAVE BEEN TOLD TO CONNECT TO {}: {}'.format(o2, o2_pins))
'''

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
        if initial_side is None:
            self.current_side_is_left = random.choice([True, False])
        else:
            # TODO TODO check this isn't flipped in first block!
            if initial_side == 'left':
                self.current_side_is_left = True
            elif initial_side == 'right':
                self.current_side_is_left = False


    # currently just on all the time. maybe i want something else?
    def odor_transitions(self, train=False, solvent_only=False):
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

        if solvent_only:
            pins = [odors2left_pins[solvent], odors2right_pins[solvent]]
        else:
            # TODO check / test this part
            if train and train_one_odor_at_a_time:
                if not train_with_cs_minus:
                    pins = [odors2left_pins[reinforced],
                            odors2right_pins[reinforced]]

                # TODO probably rename this flag to indicate its use here as
                # well?  at least document
                elif self.current_side_is_left:
                    pins = [odors2left_pins[reinforced],
                            odors2right_pins[reinforced]]
                else:
                    # TODO TODO consider erring if unreinforced is None
                    if not unreinforced is None:
                        pins = [odors2left_pins[unreinforced],
                                odors2right_pins[unreinforced]]
                    else:
                        # this work?
                        pins = []

            else:
                if self.current_side_is_left:
                    if not unreinforced is None:
                        pins = [odors2left_pins[reinforced],
                                odors2right_pins[unreinforced]]
                    else:
                        pins = [odors2left_pins[reinforced]]
                else:
                    if not unreinforced is None:
                        pins = [odors2left_pins[unreinforced],
                                odors2right_pins[reinforced]]
                    else:
                        pins = [odors2right_pins[reinforced]]
        # end solvent_only if

        # work for zero length pins?
        for p in pins:
            expanded_pins.extend(len(transition) * [p])
            seq.extend(transition)

        return expanded_pins, seq


    def shock_transitions(self):
        # TODO TODO TODO verify this use of Transition.t leads to expected
        # behavior
        # TODO handle as other params (make var)
        onset_delay_ms = params['zap/delay_from_odor_onset_ms']
        square_wave = State(ms_on=shock_ms_on, ms_off=shock_ms_off)
        transition = Transition(self.current_t0 + 
            rospy.Duration(onset_delay_ms / 1000.0), square_wave
        )

        pins = []
        pin_transitions = []

        if power_supply_enable_pin is not None:
            always_on = State(ms_on=1, ms_off=0)
            supply_enbl_transition = Transition(self.current_t0, always_on)
            pins.append(power_supply_enable_pin)
            pin_transitions.append(supply_enbl_transition)

        if train_one_odor_at_a_time:
            if not train_with_cs_minus:
                # TODO maybe refactor a little to avoid this duplication
                if one_pin_shock:
                    pins.append(all_shock)
                    pin_transitions.append(transition)
                else:
                    pins += [left_shock, right_shock]
                    pin_transitions += [transition, transition]

            # when train_one_odor_at_a_time is True, current_side_is_left
            # indicates whether the current training epoch will use the
            # reinforced_odor or the unreinforced_odor
            elif self.current_side_is_left:
                # TODO test
                if one_pin_shock:
                    # TODO what happens on the Arduino if it is given a
                    # duplicate of a transition? does it behave appropriately or
                    # fail?
                    pins.append(all_shock)
                    pin_transitions.append(transition)
                else:
                    pins += [left_shock, right_shock]
                    pin_transitions += [transition, transition]
            else:
                return [], []

        else:
            if one_pin_shock:
                raise ValueError('must specify zap/left_shock and '
                    'zap/right_shock, if train_one_odor_at_a_time is False'
                )
            if self.current_side_is_left:
                pins.append(left_shock)
                pin_transitions.append(transition)
            else:
                pins.append(right_shock)
                pin_transitions.append(transition)

        return pins, pin_transitions


    def test(self, solvent_only=False):
        start = self.current_t0

        pins, seq = self.odor_transitions(solvent_only=solvent_only)

        if not solvent_only:
            if odor_side_order == 'alternating':
                # TODO TODO rename current_side_is_left to indicate that it also
                # controls whether the current training trial uses the
                # reinforced odor or the non-reinforced odor. it does, right?
                self.current_side_is_left = not self.current_side_is_left 

            elif odor_side_order == 'random':
                self.current_side_is_left = random.choice([True, False])

        pins_to_signal = []

        end = start + rospy.Duration(test_duration_s)
        seq = Sequence(start, end, pins, seq)

        # Since odor_transitions uses this, the update has to happen after that
        # call.
        self.current_t0 = end
        return LoadSequenceRequest(Header(), seq, pins_to_signal)


    # TODO share most of this function with test?
    def train(self):
        start = self.current_t0

        odor_pins, odor_seq = self.odor_transitions(train=True)
        shock_pins, shock_seq = self.shock_transitions()
        pins = odor_pins + shock_pins
        seq = odor_seq + shock_seq

        if odor_side_order == 'alternating':
            self.current_side_is_left = not self.current_side_is_left 

        elif odor_side_order == 'random':
            self.current_side_is_left = random.choice([True, False])

        pins_to_signal = []

        end = start + rospy.Duration(train_duration_s)
        seq = Sequence(start, end, pins, seq)

        # Since odor_transitions and shock_transitions use this, the update has
        # to happen after those calls.
        self.current_t0 = end
        return LoadSequenceRequest(Header(), seq, pins_to_signal)


    def delay(self, delay_s):
        ros_delay = rospy.Duration(delay_s)
        self.current_t0 = self.current_t0 + ros_delay
        return u.copy_rostime(self.current_t0)


gen = StimuliGenerator()

# Get the (initial) time, before each call to one of the generator's functions
# internally increments this time.
t0_sec = gen.current_t0.to_sec()

trial_structure = [gen.delay(prestimulus_delay_s)]
epoch_labels = []

if flanking_solvent_test_blocks == 1:
    trial_structure += [gen.test(solvent_only=True)]
    epoch_labels += ['solvent vs. solvent test']
    if not solvent_test_to_first_test_s is None:
        trial_structure += [gen.delay(solvent_test_to_first_test_s)]

if have_training_params:
    # TODO factor out code to accommodate variable number of training blocks for
    # testing blocks?
    trial_structure += ([] if (pretest_to_train_s is None) else
                        [gen.test(),
                         gen.delay(pretest_to_train_s)]) + \
                       ((u.flatten([[f(), gen.delay(inter_train_interval_s)]
                         for f in [gen.train] * (training_blocks - 1)]) + \
                        [gen.train()]) if training_blocks > 0 else []) + \
                       [gen.delay(train_to_posttest_s),
                        gen.test()]

    # TODO handle this concurrently w/ trial_structure generation, to make sure
    # they are always consistent?
    epoch_labels += ['test'] + ['train'] * training_blocks + ['test']

elif have_testonly_params:
    # TODO looks like case where testing_blocks == 1 might have extra long
    # posttest period (=beyond_post_test_s + inter_test_interval_s). fix if so
    # TODO should i really support testing_blocks == 0 case?
    trial_structure += ((u.flatten([[f(), gen.delay(inter_test_interval_s)]
                         for f in [gen.test] * (testing_blocks - 1)]) +
                         [gen.test()]) if testing_blocks > 0 else [])
    epoch_labels += ['test']  * testing_blocks

if flanking_solvent_test_blocks == 1:
    if not last_test_to_solvent_test_s is None:
        trial_structure += [gen.delay(last_test_to_solvent_test_s)]
    trial_structure += [gen.test(solvent_only=True)]
    epoch_labels += ['solvent vs. solvent test']

trial_structure += [gen.delay(beyond_posttest_s)]

# TODO even if not printed to /rosout, is this saved by default?
# (seems like no)
rospy.logdebug('trial_structure', trial_structure)
rospy.logdebug('epoch_labels', epoch_labels)

# low_pins = pins that default to low (0v)
# high_pins = pins that default to high (5v)
# pins should be in the default state during the intertrial interval

low_pins = left_pins + right_pins
if have_training_params:
    if one_pin_shock:
        low_pins += [all_shock]
    else:
        low_pins += [left_shock, right_shock]

    if power_supply_enable_pin is not None:
        low_pins += [power_supply_enable_pin]

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
rospy.loginfo('Stimuli should finish at ' + u.readable_rostime(gen.current_t0))
rospy.loginfo(str(gen.current_t0.to_sec() - t0_sec) + ' seconds')
# TODO delete me
#'''
print('default_states:')
for x in default_states:
    print('type(x):', type(x))
    print('x:\n', x)
print()
print('trial_structure:')
for x in trial_structure:
    print('type(x):', type(x))
    print('x:\n', x)
print()
#'''
print('\ntypes of trial_structure elements:')
print([str(type(x)) for x in trial_structure])
print('len(trial_structure):', len(trial_structure))
print('# non-Time objects in trial_structure:',
    sum([type(x) is not rospy.Time for x in trial_structure])
)
print()
#

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
        rospy.set_param('multi_tracker/experiment_basename',
            experiment_basename
        )

    output_dir = os.path.join(output_base_dir, experiment_basename)
    filename = os.path.join(output_dir, experiment_basename + '_stimuli.p')
    rospy.loginfo('Trying to save save stimulus info to ' + filename)

    # TODO expand path?
    if not os.path.isdir(output_dir):
        os.mkdir(output_dir)

    # TODO check / test success
    with open(filename, 'wb') as f:
        # TODO maybe also save this as a dict?
        pickle.dump((odors2left_pins, odors2right_pins, default_states,
            trial_structure), f
        )

    # TODO allow testing this w/o running experiment
    if save_yaml_stiminfo:
        # make sure i don't mutate these, since the same variables are passed to
        # stimuli loader
        stiminfo = dict()

        left_pins2odors = dict()
        for o, p in odors2left_pins.items():
            left_pins2odors[p] = list(o)

        right_pins2odors = dict()
        for o, p in odors2right_pins.items():
            right_pins2odors[p] = list(o)

        stiminfo['left_pins2odors'] = left_pins2odors
        stiminfo['right_pins2odors'] = right_pins2odors
        stiminfo['high_by_default'] = u.represent_default_states(default_states)
        stiminfo['trial_structure'] = u.represent_trial_structure(
            trial_structure
        )

        yaml_stimfile = os.path.join(output_dir,
            experiment_basename + '_stimuli.yaml'
        )
        with open(yaml_stimfile, 'w') as f:
            yaml.dump(stiminfo, f)
else:
    rospy.logwarn('Not saving generated trial structure '
        '/ pin to odor mappings!'
    )

# TODO option to play this quickly for testing purposes
StimuliLoader(default_states, trial_structure, epoch_labels=epoch_labels)

