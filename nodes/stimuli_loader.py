#!/usr/bin/env python

from __future__ import division
from __future__ import print_function
import rospy
import datetime
from std_msgs.msg import Header
from stimuli.msg import Sequence, Transition, State, DefaultState
from stimuli.srv import LoadDefaultStates, LoadSequence, LoadSequenceRequest

def readable_rostime(ros_time):
    return datetime.datetime.fromtimestamp(ros_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S')

class StimuliLoader:
    # TODO TODO assert that time to sleep before is less than all intertrial intervals
    def __init__(self, default_states, trial_structure, epoch_labels=None):
        self.pin2name = dict(zip(range(54, 70), ['A' + str(i) for i in range(16)]))
        rospy.loginfo('stimuli_loader waiting for services')
        defaults_service_name = 'load_defaults'
        sequence_service_name = 'load_seq'
        rospy.wait_for_service(defaults_service_name)
        rospy.wait_for_service(sequence_service_name)
        load_defaults = rospy.ServiceProxy(defaults_service_name, LoadDefaultStates)
        load_next_sequence = rospy.ServiceProxy(sequence_service_name, LoadSequence)

        # to allow arduino to get parameters before services are called
        # (so that debug flag can be in effect during services)
        rospy.sleep(8.0)

        rospy.loginfo('stimuli_loader sending default states')
        try:
            h = Header()
            h.stamp = rospy.Time.now()
            resp = load_defaults(h, default_states)

        except rospy.ServiceException as exc:
            rospy.logerr("Service load_defaults failed: " + str(exc))
        rospy.loginfo('stimuli_loader finished sending default states')
        
        block_num = 0
        for n, block in enumerate(trial_structure):
            # TODO err if would send block before arduino could start it in time
            if type(block) is LoadSequenceRequest:
                rospy.loginfo('stimuli_loader sending block info')
                try:
                    rospy.logdebug(block)
                    if not epoch_labels is None:
                        rospy.loginfo(epoch_labels[block_num])

                    block.header.stamp = rospy.Time.now()
                    # TODO TODO why does this seem to selectively block forever on the last block?
                    # TODO is this the correct type? should it be the request type?
                    # TODO doesn't seem to matter whether i pass wrap the Sequence in the
                    # request type... (why?) is something wrapping it for me? do they just
                    # happen to serialize the same? what do the tutorials do?
                    resp = load_next_sequence(block)
                    rospy.logdebug('sent block info!')
                    
                    rospy.logdebug('current time is ' + readable_rostime(rospy.Time.now()))
                    rospy.logdebug('should start at ' + readable_rostime(block.seq.start))
                    rospy.logdebug('should end at ' + readable_rostime(block.seq.end))
                    rospy.loginfo('duration of sequence ' + str((block.seq.end - block.seq.start).to_sec()))
                    # TODO also include integer pin names
                    #rospy.loginfo('using pins: ' + str([self.pin2name[p] if p in self.pin2name else p for p in block.seq.pins]))
                    # TODO print which odors are associated w/ them?
                    rospy.loginfo('using pins: ' + str([p if p in self.pin2name else p for p in block.seq.pins]))

                except rospy.ServiceException as exc:
                    rospy.logerr("Service load_next_sequence failed: " + str(exc))

                end = block.seq.end
                block_num += 1

            # TODO test
            elif type(block) is rospy.Time:
                intertrial_interval_end = block
                # TODO how to not write when arduino is in a busy state / rewrite if necessary
                # should i just switch the arduino to a client?
                wake_at = intertrial_interval_end - rospy.Duration(10)
                rospy.loginfo('stimuli_loader sleeping until ' + readable_rostime(wake_at))

                # sleep until N secs before next block we need to communicate
                until_wake = (wake_at - rospy.Time.now()).to_sec()
                rospy.sleep(max(0.0, until_wake))

                end = intertrial_interval_end

            else:
                # TODO was this error getting supressed? logerr?
                # seems a list was in here
                raise ValueError('unexpected type ' + str(type(block)) + ' in trial structure')

        rate = rospy.Rate(0.5)
        end = end + rospy.Duration.from_sec(3.0)
        while not rospy.is_shutdown():
            if rospy.Time.now() >= end:
                break
            rate.sleep()
        rospy.loginfo('Done sending stimuli!')


if __name__ == '__main__':
    s = StimuliLoader()
