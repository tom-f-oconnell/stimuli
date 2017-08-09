
// TODO include some preprocessor conditionals to build with ROS
// stuff if in a catkin package? how to automatically upload?
// or at least facilitate switching between ROS / potential non-
// ROS interface

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <ros.h>
// what happens if this isn't included?
#include <ros/time.h>
#include <stimuli/PulseSequence.h>

ros::NodeHandle nh;

boolean pulse_registered;

// signal to the data acquisition which olfactometer pin we will pulse for this trial
// ~2 ms period square wave. # pulses = pin #
void signal_odor(unsigned char pin) {
  digitalWrite(odor_signaling_pin, LOW);
  delay(1);
  while (pin > 0) {
    digitalWrite(odor_signaling_pin, HIGH);
    delay(1);
    digitalWrite(odor_signaling_pin, LOW);
    delay(1);
    pin--;
  }
  delay(10);
}

void pulse_seq_callback(const PulseSeq::Request &req, PulseSeq::Response &res) {
  // TODO
  // syntax?
  res.header.time_received = nh.now();
  if (pulse_registered) {
    // TODO fail
    return;
  }
  
  pulse_registered = true;
}

void setup() {
  nh.initNode();
  nh.advertiseService(pulse_roll);
  // need topic for error reporting? dead man switch in case
  // lockup?
  pulse_registered = false;
}

void loop() {
  nh.spinOnce();
  // TODO why do they always delay?
  // how to prevent this from limiting my timing precision?
  delay(10);
}
