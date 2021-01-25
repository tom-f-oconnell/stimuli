#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import rospy
from std_msgs.msg import Header

from stimuli.msg import Sequence, Transition, State, DefaultState
from stimuli.srv import LoadDefaultStates, LoadSequence, LoadSequenceRequest
import stimuli.util as u


# I'm not sure how low this can go, but it was hardcoded to 10s before.
# TODO also actually test behavior if this is 0. delete this variable if it
# doesn't matter, despite the sleep somewhat based on this duration below
# TODO rename to intersequence or something?
# TODO TODO find lowest value of this that works (@ 115200 baud)
MIN_INTERTRIAL_INTERVAL_S = 5

# TODO TODO TODO function to validate input sequences:
# - check all times in future (by enough [~the 8s sleep below]), if not going to
#   provide a means to adjust all times to start from when we are actually able
#   to start in here
# - check all times in order (across [times + sequences] and within sequences)
# - check no pin duplication

# TODO TODO also check that seq[0] has min time (`t`) (firmware currently
# expects this...) (or fix firmware)
def _validate_trial_structure(trial_structure):
    """
    Checks some of my assumptions regarding trial structures passed by
    two_choice.py, so that I can copy the same pattern with more confidence
    in nagel.py (or drop the separate rospy.Time elements...).
    """
    assert isinstance(trial_structure[0], rospy.Time)
    for i, x in enumerate(trial_structure):
        if isinstance(x, LoadSequenceRequest):
            last_x = trial_structure[i - 1]
            assert isinstance(last_x, rospy.Time)
            assert last_x.to_sec() == x.seq.start.to_sec()
            if i + 1 < len(trial_structure):
                next_x = trial_structure[i + 1]
                assert isinstance(next_x, rospy.Time)
                # since i'm apparently not removing Time corresponding to
                # beyond_posttest_s when this param is 0 (like i thought i
                # was...), this isn't always true in the case where
                # i + 1 == len(trial_structure) - 1
                if i + 1 < len(trial_structure) - 1:
                    assert x.seq.end.to_sec() < next_x.to_sec()


class StimuliLoader:
    # TODO TODO assert that time to sleep before is less than all intertrial
    # intervals
    def __init__(self, default_states, trial_structure, epoch_labels=None):
        # TODO TODO TODO do i really still want to require rospy.Time in
        # trial_structure to manage delays? shouldn't i just compute the
        # appropriate times in here? that not possible for some reason?
        """
        Args:
        default_states: iterable of stimuli.msg.DefaultState. specifies whether
            the referenced pins should default to HIGH or LOW.

        trial_structure: iterable where elements are of type either rospy.Time
            or stimuli.srv.LoadSequenceRequest. 
        """
        _validate_trial_structure(trial_structure)

        rospy.loginfo('stimuli_loader waiting for services')
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

        # TODO is this to ensure the parameters are set or because the length of
        # time this [code on / communication with] the arduino takes? both?
        # maybe try to replace with some polling / callback / wait_for_* type
        # thing?
        # to allow arduino to get parameters before services are called
        # (so that debug flag can be in effect during services)
        # TODO TODO TODO break out as some MIN_... constant if i can't find some
        # other way to ensure time will be in the future by the time we are able
        # to execute it [/ if i don't explicitly fail those cases in some
        # validation]
        rospy.sleep(8.0)

        if not rospy.is_shutdown():
            rospy.loginfo('stimuli_loader sending default states')

        if not rospy.is_shutdown():
            try:
                h = Header()
                # This header timestamp is not used in the firmware OTHER than
                # to include it in the response, to enable us to detect problems
                # host-side (which I don't believe I currently really do
                # anywhere...)
                h.stamp = rospy.Time.now()
                resp = load_defaults(h, default_states)

            except rospy.ServiceException as exc:
                rospy.logerr('Service load_defaults failed: ' + str(exc))

        if not rospy.is_shutdown():
            rospy.loginfo('stimuli_loader finished sending default states')
        
        # TODO try to make this more interuptable when ros gets shut down
        # (though test the way i do this doesn't break runs that would be
        # started before the firmware thinks the previous sequence should have
        # finished...)
        block_num = 0
        end = None
        for i, block in enumerate(trial_structure):
            # TODO this doesn't really risk leaving the firmware in a bad state,
            # does it?
            # TODO some way to software-reset the firwmare?
            if rospy.is_shutdown():
                break

            # TODO err if would send block before arduino could start it in time
            if type(block) is LoadSequenceRequest:
                rospy.loginfo('stimuli_loader sending block info')
                try:
                    rospy.logdebug(block)
                    if not epoch_labels is None:
                        try:
                            rospy.loginfo(epoch_labels[block_num])
                        except IndexError:
                            rospy.logwarn('Not enough stimulus epoch labels.')

                    # See comment on header field above for information on how
                    # these headers are used.
                    block.header.stamp = rospy.Time.now()
                    # TODO TODO why does this seem to selectively block forever
                    # on the last block? (how to reproduce? doesn't seem to be
                    # happening now...)
                    # TODO doesn't seem to matter whether i pass wrap the
                    # Sequence in the request type... (why?) is something
                    # wrapping it for me? do they just happen to serialize the
                    # same? what do the tutorials do?
                    resp = load_next_sequence(block)
                    rospy.logdebug('sent block info!')
                    
                    rospy.logdebug('current time is ' +
                        u.readable_rostime(rospy.Time.now())
                    )
                    rospy.logdebug('should start at ' +
                        u.readable_rostime(block.seq.start)
                    )
                    rospy.logdebug('should end at ' +
                        u.readable_rostime(block.seq.end)
                    )
                    rospy.loginfo('duration of sequence ' +
                        str((block.seq.end - block.seq.start).to_sec())
                    )
                    # TODO probably factor into util fn that prints list of
                    # pins, given an object of whatever type block is
                    rospy.loginfo('using pins: ' +
                        str([u.pin_label(p) for p in block.seq.pins])
                    )

                except rospy.ServiceException as exc:
                    rospy.logerr('Service load_next_sequence failed: ' +
                        str(exc)
                    )

                end = block.seq.end
                block_num += 1

            elif type(block) is rospy.Time:
                intertrial_interval_end = block
                # TODO how to not write when arduino is in a busy state /
                # rewrite if necessary should i just switch the arduino to a
                # client?
                # sleep until N secs before next block we need to communicate
                wake_at = intertrial_interval_end - rospy.Duration(
                    MIN_INTERTRIAL_INTERVAL_S
                )
                rospy.loginfo('stimuli_loader sleeping until ' +
                    u.readable_rostime(wake_at)
                )
                until_wake = (wake_at - rospy.Time.now()).to_sec()
                rospy.sleep(max(0.0, until_wake))
                end = intertrial_interval_end

            else:
                # TODO was this error getting supressed? logerr?
                # seems a list was in here
                # TODO fix string so error is displayed correctly
                raise ValueError('unexpected type ' + str(type(block)) +
                    ' in trial structure'
                )

        # TODO maybe also only do this if (i == len(trial_structure) - 1)?
        if end is not None:
            rate = rospy.Rate(0.5)
            end = end + rospy.Duration.from_sec(3.0)
            while not rospy.is_shutdown():
                if rospy.Time.now() >= end:
                    break
                rate.sleep()

        if i == len(trial_structure) - 1:
            rospy.loginfo('Done sending stimuli!')

