
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
#include <stimuli/DefaultState.h>
#include <stimuli/PulseSeq.h>
#include <stimuli/LoadDefaultStates.h>
#include <stimuli/LoadPulseSeq.h>

#define MAX_PARAM_NAME_LENGTH 256
// TODO compare to builtin related to board to validate?
#define MAX_NUM_PINS 8
#define PIN_NOT_SET -1

ros::NodeHandle nh;

boolean defaults_registered;
boolean pulse_registered;

// TODO is this ever correctly set?? test!
stimuli::PulseSeq seq;

unsigned long start_ms;
unsigned long end_ms;
unsigned long rostime_millis_offset;
unsigned long soonest_ms;

char pins[MAX_NUM_PINS];
char pins_for_default[MAX_NUM_PINS];
unsigned char default_state[MAX_NUM_PINS];
unsigned int next_state_index[MAX_NUM_PINS];
unsigned char next_state[MAX_NUM_PINS];
unsigned long next_time_ms[MAX_NUM_PINS];
unsigned long ms_on[MAX_NUM_PINS];
unsigned long ms_off[MAX_NUM_PINS];
// replace w/ a boolean flag and signal all pins?
unsigned char pins_to_signal[MAX_NUM_PINS];
unsigned long signal_ms_before;
int odor_signaling_pin;

// TODO TODO warn if loading something that demands more memory than the arduino has

// not using params because it (seems) harder to get a parameter list of dynamic size...
// not sure why they didn't implement them simiarly, especially considering all param types seem to be contained in
// std_msgs...
void load_defaults(const stimuli::LoadDefaultStatesRequest &req, stimuli::LoadDefaultStatesResponse &res) {
  res.receive_time = nh.now();
  res.request_time = req.header.stamp;
  if (defaults_registered) {
    nh.logerror("received default pins states while already having defaults");
    return;
  }
  for (int i=0;i<req.defaults_length;i++) {
    pins_for_default[i] = req.defaults[i].pin;
    pinMode(pins_for_default[i], OUTPUT);
    if (req.defaults[i].high) {
      default_state[i] = HIGH;
    } else {
      default_state[i] = LOW;
    }
    digitalWrite(pins_for_default[i], default_state[i]);
  }
  defaults_registered = true;
}
// TODO i'm not limited in the number of ServiceServers I can operate at once on the arduino am i?
ros::ServiceServer<stimuli::LoadDefaultStatesRequest, stimuli::LoadDefaultStatesResponse> defaults_server("load_defaults", &load_defaults);

// TODO arduino fail w/ error is max_num_pins is > what arduino has?

// TODO is it any easier having one node initiate this interaction?
// make any more sense one way? (right now the python node will be
// pushing updates)
// TODO update docs to remove :: and just concatenate? or am i missing something?
// TODO what does it mean to have the ampersand here? not sure i've seen that...a
void load_next_sequences(const stimuli::LoadPulseSeqRequest &req, stimuli::LoadPulseSeqResponse &res) {
  res.receive_time = nh.now();
  res.request_time = req.seq.header.stamp;
  if (pulse_registered) {
    nh.logerror("was sent another set of stimulus info before first expired");
    // maybe make this fatal?
    return;
  }
  // TODO TODO TODO does this work or potentially fail (dangling pointer)? need to deep copy or is there easier workaround?
  // TODO test
  seq = req.seq;

  clear_pins();

  // TODO TODO maybe this should not be sent separately? (just a bool parameter)
  for (int i=0;i<req.seq.pins_to_signal_length;i++) {
    pins_to_signal[i] = req.seq.pins_to_signal[i];
  }
  
  // fail if difference of left two terms is negative?
  
  unsigned long ros_now_ms = to_millis(nh.now());
  unsigned long now_ms = millis();
  rostime_millis_offset = ros_now_ms - now_ms;
  start_ms = to_millis(req.seq.start) - rostime_millis_offset;
  // TODO this actually a keyword problem ("end")?
  end_ms = to_millis(req.seq.end) - rostime_millis_offset;

  soonest_ms = to_millis(req.seq.pulse_seq[0].states[0].t);
  stimuli::Transition curr;
  unsigned long curr_ms;

  // TODO error if pulse_seq_length > max_num_pins
  // assume that defaults and these arrive in same order? (and sort on PC side code?)
  for (int i=0;i<req.seq.pulse_seq_length;i++) {
    pins[i] = req.seq.pulse_seq[i].pin;
    next_state_index[i] = 0;
    curr = req.seq.pulse_seq[i].states[next_state_index[i]];
    // cast?
    ms_on[i] = curr.s.ms_on;
    ms_off[i] = curr.s.ms_off;

    // TODO TODO only if pwm?
    // TODO uniform start or set each to first transition time?
    // assume pins already in default states by start?
    next_time_ms[i] = start_ms;
    
    curr_ms = to_millis(curr.t);
    if (curr_ms < soonest_ms) {
      soonest_ms = curr_ms;
    }
  }
  soonest_ms = soonest_ms - rostime_millis_offset;

  // TODO dynamically allocate? or use some kind of copy constructor and just keep request obj around?
  pulse_registered = true;
}
ros::ServiceServer<stimuli::LoadPulseSeqRequest, stimuli::LoadPulseSeqResponse> server("load_next_sequences", &load_next_sequences);

void reset() {
  noInterrupts();
  wdt_enable(WDTO_8S);
  interrupts();
  // TODO delay?
}

void fail(const char *s) {
  // not sure this even works?
  nh.logerror(s);
  reset();
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
  int their_max;

  nh.loginfo("stimulus arduino loading parameters");
  
  get_param("olf/max_num_pins", &their_max);
  if (their_max >= MAX_NUM_PINS) {
    fail("Arduino can't dynamically allocate pins. Change MAX_NUM_PINS in Arduino code to a higher value");
  }

  // TODO way to load them into variable of same name somehow (or compile time macro for it?)?
  // (so i can operate on a list of param names)
  get_param("olf/odor_signaling_pin", &odor_signaling_pin);
  pinMode(odor_signaling_pin, OUTPUT);
  nh.loginfo("stimulus arduino done loading parameters");
}

void init_state() {
  for (int i=0;i<MAX_NUM_PINS;i++) {
    pins_for_default[i] = PIN_NOT_SET;
    pins[i] = PIN_NOT_SET;
    pins_to_signal[i] = PIN_NOT_SET;
  }
  defaults_registered = false;
  pulse_registered = false;
  signal_ms_before = 500;
}

void clear_pins() {
  for (int i=0;i<MAX_NUM_PINS;i++) {
    pins[i] = PIN_NOT_SET;
    pins_to_signal[i] = PIN_NOT_SET;
  }
}

// TODO maybe dont use delays?
// get parameters re: signalling from ROS?
// signal to the data acquisition which olfactometer pin we will pulse for this trial
// ~2 ms period square wave. # pulses = pin #
void signal_pin(unsigned char pin) {
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

void signal_pins() {
  for (int i=0;i<MAX_NUM_PINS;i++) {
    if (pins_to_signal[i] != PIN_NOT_SET) {
      signal_pin(pins_to_signal[i]);
    }
  }
}

// TODO test
unsigned long to_millis(ros::Time t) {
  // TODO is ull right? ul? casting necessary?
  // why do they cast to uint32_t in rosserial_client time code if sec and nsec are defined as
  // that type? because ull isn't guaranteed to be that?
  return (unsigned long) t.sec*1000ull + (unsigned long) t.nsec/1e6;
}

unsigned long to_micros(ros::Time t) {
  return (unsigned long) t.sec*1000000ull + (unsigned long) t.nsec/1e3;
}

void update_pwm_pinstates() {
  for (int i=0;i<MAX_NUM_PINS;i++) {
    if (pins[i] != PIN_NOT_SET) {
      if (next_time_ms[i] <= millis()) {
        digitalWrite(pins[i], next_state[i]);

        // should i increment from previous or from actual time?
        // currently incrementing from previous goal time
        if (next_state[i] == HIGH) {
          next_time_ms[i] += ms_on[i];
          next_state[i] = LOW;
        } else {
          next_time_ms[i] += ms_off[i];
          next_state[i] = HIGH;
        }
      }
    } else {
      return;
    }
  }
}

void update_pulses_ros() {
  // update pulses with transitions specified explicitly (slower changes)
  if (soonest_ms <= millis()) {
    for (int i=0;i<seq.pulse_seq_length;i++) {
      // TODO - or + rostime_millis_offset? (i think this is right)
      if ((to_millis(seq.pulse_seq[i].states[next_state_index[i]].t) - \
        rostime_millis_offset) <= millis()) {

        stimuli::State s = seq.pulse_seq[i].states[next_state_index[i]].s;
        if (s.ms_on == 0) {
          digitalWrite(seq.pulse_seq[i].pin, LOW);
        } else if (s.ms_off == 0) {
          digitalWrite(seq.pulse_seq[i].pin, HIGH);
        } else {
          // TODO 
          // merge w/ above?
          digitalWrite(seq.pulse_seq[i].pin, HIGH);
          ms_on[i] = s.ms_on;
          ms_off[i] = s.ms_off;
          next_state[i] = LOW;
          // use same time as that in if statement above?
          next_time_ms[i] = millis() + ms_on[i];
        }
        next_state_index[i]++;
      }
    }

    unsigned long curr_ms;
    soonest_ms = to_millis(seq.pulse_seq[0].states[next_state_index[0]].t);
    for (int i=0;i<seq.pulse_seq_length;i++) {
      // TODO check for / fix different wraparound than millis()
      curr_ms = to_millis(seq.pulse_seq[i].states[next_state_index[i]].t);
      if (curr_ms < soonest_ms) {
        soonest_ms = curr_ms;
      }
    }
    soonest_ms = soonest_ms - rostime_millis_offset;
  }

  // TODO store minimum time in loop above to avoid need to loop until we are there?
  // maybe call this multiple times for each of the above?
  // update pins locked in a specified squarewave until next explicit transition
  update_pwm_pinstates();
}

void end_sequence() {
  for (int i=0;i<MAX_NUM_PINS;i++) {
    if (pins[i] != PIN_NOT_SET) {
      digitalWrite(pins_for_default[i], default_state[i]);
    } else {
      break;
    }
  }
  pulse_registered = false;
}

void update_pulses_blocking() {
  while (millis() < end_ms) {
  // while (micros() < end_us) {
    update_pulses_ros();
  }
  end_sequence();
}

void setup() {
  // disabling interrupts necessary here?
  noInterrupts();
  wdt_disable();
  interrupts();
  init_state();
  // ever a chance of these first two calls taking > WDT_RESET?
  // i could limit their timeouts to prevent that?
  // TODO name?
  nh.initNode();
  nh.advertiseService(server);
  get_params();
}

// TODO change things named pulse_XXX to sequence_XXX

void loop() {
  // bounds on how long this will take?
  // TODO debug compile flags to track distribution of times this takes, w/ emphasis on extreme values?
  // for reporting...
  // i think for now i'm just going to block while executing the sequence
  nh.spinOnce();
  if (pulse_registered) {
    if (start_ms <= millis()) {
      update_pulses_blocking();
    } else if (start_ms - signal_ms_before <= millis()) {
      signal_pins();
    }
  }
  // TODO why do they always delay?
  // how to prevent this from limiting my timing precision?
  delay(10);
}
