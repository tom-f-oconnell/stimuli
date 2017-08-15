
// TODO include some preprocessor conditionals to build with ROS
// stuff if in a catkin package? how to automatically upload?
// or at least facilitate switching between ROS / potential non-
// ROS interface

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

// TODO debug flags with rosdebug prints

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
  nh.loginfo("stimulus arduino receiving defaults...");
  res.request_time = req.header.stamp;
  if (defaults_registered) {
    // TODO replace with warn so i can control state after that? does error actually exit / do something?
    nh.logerror("received default pins states while already having defaults");
    return;
  }
  for (int i = 0; i < req.defaults_length; i++) {
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
void load_next_sequence(const stimuli::LoadPulseSeqRequest &req, stimuli::LoadPulseSeqResponse &res) {
  res.receive_time = nh.now();
  nh.loginfo("stimulus arduino in load_next_sequence");
  res.request_time = req.seq.header.stamp;
  if (pulse_registered) {
    // TODO careful...
    nh.logerror("was sent another set of stimulus info before first expired");
    // maybe make this fatal?
    return;
  }
  // TODO TODO TODO does this work or potentially fail (dangling pointer)? need to deep copy or is there easier workaround?
  // TODO test
  seq = req.seq;

  clear_pins();

  // TODO TODO maybe this should not be sent separately? (just a bool parameter)
  for (int i = 0; i < req.seq.pins_to_signal_length; i++) {
    pins_to_signal[i] = req.seq.pins_to_signal[i];
  }

  // fail if difference of left two terms is negative?

  unsigned long ros_now_ms = to_millis(nh.now());
  unsigned long now_ms = millis();
  rostime_millis_offset = ros_now_ms - now_ms;
  // TODO TODO can i definitely assume start_ms is correct? test!
  start_ms = to_millis(req.seq.start) - rostime_millis_offset;
  // TODO this actually a keyword problem ("end")?
  end_ms = to_millis(req.seq.end) - rostime_millis_offset;
  
  // TODO delete
  char str[30];
  sprintf(str, "ros_now_ms %lu", ros_now_ms);
  nh.logwarn(str);
  sprintf(str, "now_ms %lu", now_ms);
  nh.logwarn(str);
  sprintf(str, "rostime_millis_offset %lu", rostime_millis_offset);
  nh.logwarn(str);
  // TODO TODO i need to get it s.t. start_ms is < millis() here, preferably by a little bit of a margin
  sprintf(str, "start_ms %lu", start_ms);
  nh.logwarn(str);
  sprintf(str, "end_ms %lu", end_ms);
  nh.logwarn(str);

  // TODO this is correct. why is it different from what i'm getting above?
  int diff = req.seq.end.sec - req.seq.start.sec;
  sprintf(str, "duration secs via ros %d", diff);
  nh.logwarn(str);

  soonest_ms = to_millis(req.seq.pulse_seq[0].states[0].t);
  stimuli::Transition curr;
  unsigned long curr_ms;

  // TODO error if pulse_seq_length > max_num_pins
  // assume that defaults and these arrive in same order? (and sort on PC side code?)
  for (int i = 0; i < req.seq.pulse_seq_length; i++) {
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
ros::ServiceServer<stimuli::LoadPulseSeqRequest, stimuli::LoadPulseSeqResponse> server("load_seq", &load_next_sequence);

void reset() {
  noInterrupts();
  wdt_enable(WDTO_8S);
  interrupts();
  // TODO delay?
}

void fail(const char *s) {
  // TODO careful... might never get to the next line
  nh.logerror(s);
  reset();
}

// TODO way to specify OR / sum type for param, since implementation is identical for all 3?
// TODO auto required if no default?
bool get_param(const char* n, int* param, const int* def, bool required = false, int len = 1, int timeout = 1000) {
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

bool get_param(const char* n, float* param, const float* def, bool required = false, int len = 1, int timeout = 1000) {
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
bool get_param(const char *n, char **param, char** def, bool required = false, int len = 1, int timeout = 1000) {
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
bool get_param(const char* n, int* param, bool required = false, int len = 1, int timeout = 1000) {
  return get_param(n, param, NULL, required = required, len = len, timeout = timeout);
}

bool get_param(const char* n, float* param, bool required = false, int len = 1, int timeout = 1000) {
  return get_param(n, param, NULL, required = required, len = len, timeout = timeout);
}

bool get_param(const char *n, char **param, bool required = false, int len = 1, int timeout = 1000) {
  return get_param(n, param, NULL, required = required, len = len, timeout = timeout);
}

// TODO TODO keep trying to get params if any requests are not successful?

// TODO modify these library functions so they can take string or any kind of const char input?
// what does Serial.print do?
// TODO check for republishing node or fail (will be missing important parameters)
bool get_params() {
  bool success;
  int their_max;

  nh.loginfo("stimulus arduino attempting to get parameters");

  success = get_param("olf/max_num_pins", &their_max);
  //nh.logwarn(success
  if (!success) {
    nh.logwarn("parameter olf/max_num_pins needs to be set");
    return false;
  }
  if (their_max > MAX_NUM_PINS) {
    fail("Arduino can't dynamically allocate pins. Change MAX_NUM_PINS in Arduino code to a higher value");
  }

  // TODO way to load them into variable of same name somehow (or compile time macro for it?)?
  // (so i can operate on a list of param names)
  /*
    int default_odor_signaling_pin = PIN_NOT_SET;
    get_param("olf/odor_signaling_pin", &odor_signaling_pin, &default_odor_signaling_pin);
    if (odor_signaling_pin != PIN_NOT_SET) {
    nh.logwarn("no odor signaling pin set. set the parameter olf/odor_signaling_pin if you'd like to signal to a DAQ");
    pinMode(odor_signaling_pin, OUTPUT);
    }
  */
  nh.loginfo("stimulus arduino done loading parameters");
  return success;
}

void init_state() {
  for (int i = 0; i < MAX_NUM_PINS; i++) {
    pins_for_default[i] = PIN_NOT_SET;
    pins[i] = PIN_NOT_SET;
    pins_to_signal[i] = PIN_NOT_SET;
  }
  defaults_registered = false;
  pulse_registered = false;
  signal_ms_before = 500;
}

void clear_pins() {
  for (int i = 0; i < MAX_NUM_PINS; i++) {
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
  for (int i = 0; i < MAX_NUM_PINS; i++) {
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

  // unsigned longs here are 4 bytes
  unsigned long high_bits_zeroed_secs = 0x00ff & t.sec;
  /*
  char str[30];
  // TODO TODO why is this intelligible on its own? i thought it had two fields...
  sprintf(str, "t.sec %lu", t.sec);
  nh.logwarn(str);
  sprintf(str, "t.nsec %lu", t.nsec);
  nh.logwarn(str);
  sprintf(str, "converted t.sec %lu", (unsigned long) t.sec * 1000);
  nh.logwarn(str);
  // TODO why isn't this ~3 digits? (it is many more)
  sprintf(str, "converted t.nsec %lu", (unsigned long) t.nsec / 1000000);
  nh.logwarn(str);
  sprintf(str, "total %lu", (unsigned long) t.sec * 1000 + (unsigned long) t.nsec / 1000000);
  nh.logwarn(str);
  */
  // TODO is the type of their sum not ul or something? why does returned value seem diferent from the "total" print?
  return (unsigned long) high_bits_zeroed_secs * 1000 + (unsigned long) t.nsec / 1000000;
}

unsigned long to_micros(ros::Time t) {
  // TODO fix
  return (unsigned long) t.sec * 1000000ull + (unsigned long) t.nsec / 1e3;
}

void update_pwm_pinstates() {
  for (int i = 0; i < MAX_NUM_PINS; i++) {
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
    for (int i = 0; i < seq.pulse_seq_length; i++) {
      // TODO - or + rostime_millis_offset? (i think this is right)
      if ((to_millis(seq.pulse_seq[i].states[next_state_index[i]].t) - \
           rostime_millis_offset) <= millis()) {

        // TODO TODO test / debug w/ prints
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
    for (int i = 0; i < seq.pulse_seq_length; i++) {
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
  for (int i = 0; i < MAX_NUM_PINS; i++) {
    if (pins[i] != PIN_NOT_SET) {
      digitalWrite(pins_for_default[i], default_state[i]);
    } else {
      break;
    }
  }
  pulse_registered = false;
}

// TODO it seems the pulses were a little jittery? why do they appear to jump low briefly?
// solution?
void update_pulses_blocking() {
  nh.loginfo("stimulus arduino beginning sequence");
  char str[30];
  digitalWrite(LED_BUILTIN, HIGH);
  unsigned long now_millis = millis();
  sprintf(str, "millis() %lu", now_millis);
  nh.logwarn(str);
  while (millis() < end_ms) {
    // while (micros() < end_us) {
    update_pulses_ros();
  }
  end_sequence();
  digitalWrite(LED_BUILTIN, LOW);
  nh.loginfo("stimulus arduino finished sequence");
  // TODO how to not permanently lose sync with rosserial_python?
}

void setup() {
  // disabling interrupts necessary here?
  noInterrupts();
  wdt_disable();
  interrupts();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  init_state();
  // ever a chance of these first two calls taking > WDT_RESET?
  // i could limit their timeouts to prevent that?
  // TODO name?
  nh.initNode();
  // to get time sync. may need to spin more times.
  // remove?
  nh.spinOnce();
  
  nh.advertiseService(defaults_server);
  nh.advertiseService(server);
  // keep trying to get params until succesful
  while (!get_params()) {
    delay(500);
  }
  nh.loginfo("stimulus arduino done setup.");
}

// TODO change things named pulse_XXX to sequence_XXX
//int mod = 0;

void loop() {
  /*
  char str[30];
  // TODO TODO why is this intelligible on its own? i thought it had two fields...
  sprintf(str, "nh.now() %lu", nh.now());
  nh.logwarn(str);
  sprintf(str, "to_millis(nh.now()) %lu", to_millis(nh.now()));
  nh.logwarn(str);
  */
  
  // do w/ certain low rate?
  //if (mod % 1) {
  //nh.loginfo("stimulus arduino in loop");
  //  mod = (mod + 1) % 1;
  //}

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

  //if (nh.now() != 0 && !services_advertised) {
    
  //}
  // TODO why do they always delay?
  // how to prevent this from limiting my timing precision?
  delay(10);
}
