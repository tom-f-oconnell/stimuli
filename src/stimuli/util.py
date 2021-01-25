
import sys
import signal
import time
import datetime

import rospy

from stimuli.srv import LoadSequenceRequest


def copy_rostime(ros_time):
    """Returns a copy of a `rospy.Time` object.
    """
    return rospy.Time.from_sec(ros_time.to_sec())


def readable_rostime(ros_time, format_str='%Y-%m-%d %H:%M:%S'):
    return datetime.datetime.fromtimestamp(ros_time.to_sec()).strftime(
        format_str
    )


# TODO test. this seems to maybe be incorrectly returning True sometimes?
def get_params(param_dict, required_params, default_params=None):
    """Fills param_dict values with the values each key has on the ROS parameter
    server. Raises errors if these invalid combinations of parameters are
    found:
        - some, but not all, of required_params
        - any default_params, but not all required_params

    Args:
        required_params (set): set of required parameter names (str)
        default_params (dict): parameter name (str) -> default value
        param_dict (dict): dict to populate with parameter name (str) -> value

    Returns whether any parameters were found.
    """
    if param_dict is None:
        param_dict = dict()

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

    # TODO test this change doesn't break anything
    if default_params is not None:
        defaults_found, _ = get_params_helper(default_params)
        # if len(required_found) > 0, there would already have been an error
        if len(defaults_found) > 0 and len(required_found) == 0:
            error_msg = ('Since {} were set, the following are required:\n'
                ).format(defaults_found)
            error_msg += missing_params_err_msg()

            raise ValueError(error_msg)

        # TODO was this ever the correct return expression? test.
        return len(defaults_found) > 0 or len(required_found) > 0
    else:
        # TODO correct? test
        return len(required_found) > 0


# TODO maybe define this in a general utility library? (unix specific?)
def nonblocking_raw_input(prompt, timeout_s=1):
    """Returns None if timeout, otherwise behaves as raw_input.

    Unix specific.

    Need to use signal.setitimer if I want better than 1s resolution.

    See jer's answer here:
    https://stackoverflow.com/questions/2933399
    """
    class AlarmException(Exception):
        pass

    def alarm_handler(signum, frame):
        raise AlarmException

    signal.signal(signal.SIGALRM, alarm_handler)
    signal.alarm(timeout_s)
    try:
        text = raw_input(prompt)
        signal.alarm(0)
        return text

    except AlarmException:
        pass

    signal.signal(signal.SIGALRM, signal.SIG_IGN)
    return None


def ros_friendly_raw_input(prompt):
    """raw_input that periodically polls for input and checks for ROS shutdown.

    prompt is printed to stdout, not through the rospy logging.

    If ROS is shutdown while blocking, calls sys.exit()
    """
    not_shown_prompt = True
    while not rospy.is_shutdown():
        if not_shown_prompt:
            out = nonblocking_raw_input(prompt)
            not_shown_prompt = False
        else:
            out = nonblocking_raw_input('')

        if not (out is None):
            return out

        # to not take up as much resources
        # not sure if it'd make much of a difference
        time.sleep(0.1)

    sys.exit()


def flatten(l):
    return [item for sublist in l for item in sublist]


# str board identifier -> dict mapping pin #'s to labels
# Each such dict must only have entries for pins where their labels are not the
# same as their pin number.
_pin_labels = {
    'mega': dict(zip(range(54, 70), ['A' + str(i) for i in range(16)]))
}
def pin_label(pin_number, board='mega'):
    """Takes digital pin number to the label that pin has on the board.

    For many digital pins on Arduinos, the output is the same as the input,
    but for the analog pins, this will return labels like A0, A1, etc.
    """
    p2l = _pin_labels[board]
    return p2l[pin_number] if pin_number in p2l else pin_number


# TODO TODO TODO rename all these to indicate they are used for formatting YAML
# output, if that is their (only) intended use!
def represent_default_states(defaults):
    """Returns a representation of a list of stimuli/DefaultState for neat YAML.
    """
    high_by_default = dict()
    for default in defaults:
        high_by_default[default.pin] = default.high
    return high_by_default

def represent_rostime(t):
    """Returns a float of the same Unix epoch time as the input ROS Time.
    """
    return t.to_sec()

def represent_state(s):
    """Return a representation of stimuli/State type for neat YAML saving.
    """
    return {'ms_on': s.ms_on, 'ms_off': s.ms_off}

def represent_transition(t):
    """Return a representation of stimuli/Transition type for neat YAML saving.
    """
    return {'time': represent_rostime(t.t), 'new_state': represent_state(t.s)}

def represent_sequence(seq):
    """Return a representation of stimuli/Sequence type for neat YAML saving.
    """
    return {
        'start': represent_rostime(seq.start),
        'end': represent_rostime(seq.end),
        'pins': seq.pins,
        'transitions': [represent_transition(t) for t in seq.seq]
    }

def represent_trial_structure(ts):
    ts_representation = []
    for b in ts:
        if type(b) is rospy.rostime.Time:
            ts_representation.append(represent_rostime(b))

        # TODO can i do without the _LoadSequence part? need a diff import?
        #elif type(b) is stimuli.srv._LoadSequence.LoadSequenceRequest:
        elif type(b) is LoadSequenceRequest:
            ts_representation.append(represent_sequence(b.seq))
        else:
            raise ValueError('unexpected type in trial structure: ' +
                '{}'.format(type(b))
            )
    return ts_representation

