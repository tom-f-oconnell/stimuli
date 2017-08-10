
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
// what does this do?
using stimuli::PulseSeq;

boolean pulse_registered;

// TODO is it any easier having one node initiate this interaction?
// make any more sense one way? (right now the python node will be
// pushing updates)
void callback(const PulseSeq::Request &req, PulseSeq::Response &res) {
  // TODO
  // syntax?
  res.header.time_received = nh.now();
  if (pulse_registered) {
    // TODO fail
    return;
  }
  
  pulse_registered = true;
}
ros::ServiceServer<PulseSeq::Request, PulseSeq::Response> server("load_next_stiminfo", &callback);

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

void setup() {
  // TODO name?
  nh.initNode();
  nh.advertiseService(server);
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
