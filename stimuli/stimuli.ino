
// TODO include some preprocessor conditionals to build with ROS
// stuff if in a catkin package? how to automatically upload?
// or at least facilitate switching between ROS / potential non-
// ROS interface

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <avr/wdt.h>
#include <ros.h>
// what happens if this isn't included?
#include <ros/time.h>
#include <stimuli/PulseSeq.h>
#include <stimuli/LoadPulseSeq.h>

#define MAX_PARAM_NAME_LENGTH 256
// TODO compare to builtin related to board to validate?
#define MAX_NUM_PINS 8

ros::NodeHandle nh;
// what does this do?
// TODO should i use something more general / do this for my other stuff?
using stimuli::PulseSeq;

boolean pulse_registered;

int odor_signaling_pin;

// TODO arduino fail w/ error is max_num_pins is > what arduino has?

// TODO is it any easier having one node initiate this interaction?
// make any more sense one way? (right now the python node will be
// pushing updates)
// TODO update docs to remove :: and just concatenate? or am i missing something?
void callback(const stimuli::LoadPulseSeqRequest &req, stimuli::LoadPulseSeqResponse &res) {
  // TODO
  // syntax?
  res.receive_time = nh.now();
  if (pulse_registered) {
    // TODO fail
    return;
  }
  
  pulse_registered = true;
}
ros::ServiceServer<stimuli::LoadPulseSeqRequest, stimuli::LoadPulseSeqResponse> server("load_next_stiminfo", &callback);

void fail() {
  noInterrupts();
  wdt_enable(WDTO_8S);
  interrupts();
  // TODO delay?
}

void fail(const char *s) {
  // not sure this even works?
  nh.logerror(s);
  fail();
}

// TODO way to specify OR / sum type for param, since implementation is identical for all 3?
// TODO auto required if no default?
bool get_param(const char* n, int* param, const int* def, bool required=false, int len=1, int timeout=1000) {
  bool success;
  success = nh.getParam(n, param);
  if (!success) {
    if (def != NULL) {
      // sufficient? TODO test
      *param = *def;
    } else if (required) {
      // should not return. board will reset.
      fail("could not get parameter");
    } else {
      return false;
    }
  }
  return true;
}

bool get_param(const char* n, float* param, const float* def, bool required=false, int len=1, int timeout=1000) {
  bool success;
  success = nh.getParam(n, param);
  if (!success) {
    if (def != NULL) {
      // sufficient? TODO test
      *param = *def;
    } else if (required) {
      // should not return. board will reset.
      fail("could not get parameter");
    } else {
      return false;
    }
  }
  return true;
}

// TODO how to define default w/ const, while allowing its value to be assigned to param?
bool get_param(const char *n, char **param, char** def, bool required=false, int len=1, int timeout=1000) {
  bool success;
  success = nh.getParam(n, param);
  if (!success) {
    if (def != NULL) {
      // sufficient? TODO test
      *param = *def;
    } else if (required) {
      // TODO concatenate w/ param name (will need to use a diff type though)
      // should not return. board will reset.
      fail("could not get parameter");
    } else {
      return false;
    }
  }
  return true;
}

// is this syntax with default args correct?
bool get_param(const char* n, int* param, bool required=false, int len=1, int timeout=1000) {
  return get_param(n, param, NULL, required=required, len=len, timeout=timeout);
}

bool get_param(const char* n, float* param, bool required=false, int len=1, int timeout=1000) {
  return get_param(n, param, NULL, required=required, len=len, timeout=timeout);
}

bool get_param(const char *n, char **param, bool required=false, int len=1, int timeout=1000) {
  return get_param(n, param, NULL, required=required, len=len, timeout=timeout);
}

// TODO modify these library functions so they can take string or any kind of const char input?
// what does Serial.print do?
// TODO check for republishing node or fail (will be missing important parameters)
void get_params() {
  bool s;
  int their_max;

  nh.loginfo("stimulus arduino loading parameters");
  
  get_param("olf/max_num_pins", &their_max);
  if (their_max >= MAX_NUM_PINS) {
    fail("Arduino can't dynamically allocate pins. Change MAX_NUM_PINS in Arduino code to a higher value");
  }

  // TODO way to load them into variable of same name somehow (or compile time macro for it?)?
  // (so i can operate on a list of param names)
  get_param("olf/odor_signaling_pin", &odor_signaling_pin);
  nh.loginfo("stimulus arduino done loading parameters");
}

// TODO maybe dont use delays?
// get parameters re: signalling from ROS?
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
  // disabling interrupts necessary here?
  noInterrupts();
  wdt_disable();
  interrupts();
  // ever a chance of these first two calls taking > WDT_RESET?
  // i could limit their timeouts to prevent that?
  // TODO name?
  nh.initNode();
  nh.advertiseService(server);
  get_params();
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
