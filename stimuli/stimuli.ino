
// TODO include some preprocessor conditionals to build with ROS
// stuff if in a catkin package? how to automatically upload?
// or at least facilitate switching between ROS / potential non-
// ROS interface
// TODO add (maybe target specific) preprocessor definition in catkin / cmake to support this

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <avr/wdt.h>
#include <ros.h>
// what happens if this isn't included?
#include <ros/time.h>
#include <stimuli/LoadDefaultStates.h>
#include <stimuli/LoadPulseSeq.h>
#include <stimuli/TestTransportLoadDefaultStatesReq.h>
#include <stimuli/TestTransportLoadPulseSeqReq.h>

#define MAX_PARAM_NAME_LENGTH 256
// TODO compare to builtin related to board to validate?
// TODO maybe just make arbitrarily large and err if used > # actually available on board?
// or invalid pin #s requested?
#define MAX_NUM_PINS 8
#define PIN_NOT_SET -1
#define NOT_PWM -1
#define DONE -1

// TODO test all time calculations for millis / micros rollover robustness (could use setMillis(-NNN))
// TODO valgrind the output of this + the rosserial code?

ros::NodeHandle nh;

boolean debug;

boolean defaults_registered;
boolean pulse_registered;

// TODO is this ever correctly set?? test!
//stimuli::PulseSeq seq;

unsigned long start_ms;
unsigned long end_ms;
unsigned long rostime_millis_offset;
unsigned long soonest_ms;

char pins[MAX_NUM_PINS];
char pins_for_default[MAX_NUM_PINS];
unsigned char default_state[MAX_NUM_PINS];
unsigned int next_state_index[MAX_NUM_PINS];
unsigned int last_state_index[MAX_NUM_PINS];
char next_state[MAX_NUM_PINS];
unsigned long next_time_ms[MAX_NUM_PINS];
unsigned long ms_on[MAX_NUM_PINS];
unsigned long ms_off[MAX_NUM_PINS];
// replace w/ a boolean flag and signal all pins?
unsigned char pins_to_signal[MAX_NUM_PINS];
unsigned long signal_ms_before;
int odor_signaling_pin;

// TODO TODO warn if loading something that demands more memory than the arduino has

void test_defaults_req(const stimuli::TestTransportLoadDefaultStatesReqRequest &req, 
                         stimuli::TestTransportLoadDefaultStatesReqResponse &res) {
  // TODO how to appropriately copy over?
}
ros::ServiceServer<stimuli::TestTransportLoadDefaultStatesReqRequest, 
  stimuli::TestTransportLoadDefaultStatesReqResponse> test_defaults_server("test_defaults_req", &test_defaults_req);

void test_loadseq_req(const stimuli::TestTransportLoadPulseSeqReqRequest &req, 
                         stimuli::TestTransportLoadPulseSeqReqResponse &res) {
  // TODO maybe just manually copy? (if possible...)
  unsigned char buff[256];
  req.serialize(buff);
  res.deserialize(buff);
}
ros::ServiceServer<stimuli::TestTransportLoadPulseSeqReqRequest, 
  stimuli::TestTransportLoadPulseSeqReqResponse> test_loadseq_server("test_loadseq_req", &test_loadseq_req);


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
  char str[30];
  // TODO how to set debug flag dynamically from computer? include in service type
  // since cant seem to reliably nh.getParams before services are negotiated...
  if (debug) {
    nh.loginfo("defaults:");
  }
  // TODO break and err if length > defaults. or don't start.
  if (req.defaults_length > MAX_NUM_PINS) {
    fail("Trying to set defaults for more pins than available.");
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
    if (debug) {
      sprintf(str, "%d=%d", pins_for_default[i], default_state[i]);
      nh.loginfo(str);
    }
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
  char str[30];
  for (int i=0;i<req.seq.pulse_seq_length;i++) {
    sprintf(str, "i %d pin %d", i, req.seq.pulse_seq[i].pin);
    nh.loginfo(str);
    sprintf(str, "first s on %d off %d", req.seq.pulse_seq[i].states[0].s.ms_on, req.seq.pulse_seq[i].states[0].s.ms_off);
    nh.loginfo(str);    
    sprintf(str, "states_length %d", req.seq.pulse_seq[i].states_length);
    nh.loginfo(str);
    sprintf(str, "address of transition list %p", (void *) &req.seq.pulse_seq[i].states[0]);
    nh.loginfo(str);
  }
  res.receive_time = nh.now();
  /*
  if (debug) {
    nh.loginfo("stimulus arduino entering load_next_sequence");
  }
  */
  res.request_time = req.seq.header.stamp;
  if (pulse_registered) {
    fail("was sent another set of stimulus info before first expired");
  }
  // TODO TODO TODO does this work or potentially fail (dangling pointer)? need to deep copy or is there easier workaround?
  // TODO test
  //seq = req.seq;

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

  if (debug) {
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
    // TODO TODO still a problem?
    int diff = req.seq.end.sec - req.seq.start.sec;
    sprintf(str, "duration secs via ros %d", diff);
    nh.logwarn(str);
  }

  soonest_ms = to_millis(req.seq.pulse_seq[0].states[0].t);
  // TODO might this be causing memory issues? not sure...
  stimuli::Transition curr;
  unsigned long curr_ms;

  for (int i=0;i<req.seq.pulse_seq_length;i++) {
    pins[i] = req.seq.pulse_seq[i].pin;
    
    sprintf(str, "pin %d", pins[i]);
    nh.loginfo(str);
    
    next_state_index[i] = 0;
    last_state_index[i] = req.seq.pulse_seq[i].states_length - 1;
    curr = req.seq.pulse_seq[i].states[0];

    // TODO why the hell are these all the values of the last one? is it not being transferred / (de)serialized properly?
    sprintf(str, "first s on %d off %d", req.seq.pulse_seq[i].states[0].s.ms_on, req.seq.pulse_seq[i].states[0].s.ms_off);
    nh.loginfo(str);
    
    // cast?
    ms_on[i] = curr.s.ms_on;
    ms_off[i] = curr.s.ms_off;
    if (ms_on[i] == 0 || ms_off[i] == 0) {
      next_state[i] = NOT_PWM;
      nh.loginfo("setting not pwm");
    } else {
      next_state[i] = HIGH;
      nh.loginfo("setting pwm");
    }

    // TODO TODO only if pwm?
    // TODO uniform start or set each to first transition time?
    // assume pins already in default states by start?
    next_time_ms[i] = start_ms;

    
    curr_ms = to_millis(curr.t);
    // TODO subtract before comparing
    if (curr_ms < soonest_ms) {
      soonest_ms = curr_ms;
    }
  }
  soonest_ms = soonest_ms - rostime_millis_offset;

  // TODO  but i supposed the problem with this is that if no states are counted as ~ start above
  // it is unlikely any will be set in first iteration
  //soonest_ms = start_ms;
  
  // TODO should i just set this to the start? and forgo the calculation?
  // or forgo start being passed in the future?
  sprintf(str, "soonest_ms %lu", soonest_ms);
  nh.loginfo(str);

  // TODO dynamically allocate? or use some kind of copy constructor and just keep request obj around?
  pulse_registered = true;

  /*
  if (debug) {
    nh.loginfo("stimulus arduino leaving load_next_sequence");
  }
  */
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
  int local_debug;
  int their_max;

  nh.loginfo("stimulus arduino attempting to get parameters");

  // TODO will this err if not set? any way to suppress that?
  success = get_param("stimulus_arduino/debug", &local_debug);
  
  // could result in debug being false when it was just not correctly sent
  // but is set true. maybe try to fix that? (as is to not fail if param is
  // not set. use required flag?)
  if (!success) {
    debug = false;
  } else {
    if (local_debug == 0) {
      debug = false;
    } else {
      debug = true;
    }
  }

  success = get_param("olf/max_num_pins", &their_max);
  if (!success) {
    return false;
  }
  /*
   * TODO uncomment
  if (their_max > MAX_NUM_PINS) {
    fail("Arduino can't dynamically allocate pins. Change MAX_NUM_PINS in Arduino code to a higher value");
  }
  */

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
    // next_time_ms elements are undefined unless next_state is not this value
    next_state[i] = NOT_PWM;
  }
  defaults_registered = false;
  pulse_registered = false;
  // TODO TODO set via parameter
  signal_ms_before = 500;
}

// TODO consolidate w/ init_state? move some of that here?
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


// TODO unit test
unsigned long to_millis(ros::Time t) {
  return (unsigned long) (t.sec * 1000ull + t.nsec / 1000000ull);
}

unsigned long to_micros(ros::Time t) {
  // TODO fix + test
  return (unsigned long) (t.sec * 1000000ull + t.nsec / 1000ull);
}

void update_pwm_pinstates() {
  for (int i = 0; i < MAX_NUM_PINS; i++) {
    if (pins[i] != PIN_NOT_SET && next_state[i] != NOT_PWM) {
      // TODO make robust to rollover
      if (next_time_ms[i] <= millis()) {
        digitalWrite(pins[i], next_state[i]);
        char str[30];
        sprintf(str, "%d to %d", pins[i], next_state[i]);
        nh.loginfo(str);

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

// TODO (fix this) soonest is broken, but as long as i only have one state per sequence, it should work for now
// TODO maybe spinOnce occasionally in here?
void update_pulses_ros() {
  // update pulses with transitions specified explicitly (slower changes)
  // TODO maybe only check millis periodically here, to update pwm better (or call that multiple times?)
  unsigned long now_ms = millis();
  char str[30];
  // TODO fix rollover
  //sprintf(str, "now %lu", now_ms);
  //nh.loginfo(str);
  
  // TODO wait, was initial value of soonest off? i thought this loop should be triggered once at beginning?
  /*
  if (soonest_ms <= now_ms) {
    // TODO maybe this seq storage can not be trusted?
    // TODO TODO print and check      
    for (int i = 0; i < seq.pulse_seq_length; i++) {
      sprintf(str, "p %d done %d", pins[i], next_state_index[i]);
      nh.loginfo(str);
      // TODO TODO fix. try to compare durations rather than times... rollover!
      if (next_state_index[i] != DONE && (to_millis(seq.pulse_seq[i].states[next_state_index[i]].t) - \
           rostime_millis_offset) <= now_ms) {

        // TODO not sure this can be trusted
        stimuli::State s = seq.pulse_seq[i].states[next_state_index[i]].s;
        sprintf(str, "new state on %d off %d", s.ms_on, s.ms_off);
        nh.loginfo(str);
        if (s.ms_on == 0) {
          digitalWrite(seq.pulse_seq[i].pin, LOW);
          next_state[i] = NOT_PWM;
          nh.loginfo("setting NOT_PWM");
        } else {
          digitalWrite(seq.pulse_seq[i].pin, HIGH);
          if (s.ms_off != 0) {
            sprintf(str, "s.ms_off = %d", s.ms_off);
            nh.loginfo(str);
            nh.loginfo("setting PWM!!!");
            next_state[i] = LOW;
            next_time_ms[i] = now_ms + ms_on[i];
          } else {
            nh.loginfo("setting NOT_PWM");
            next_state[i] = NOT_PWM;
          }
        }
        
        sprintf(str, "n %d l %d", next_state_index[i], last_state_index[i]);
        nh.loginfo(str);
        
        if (next_state_index[i] < last_state_index[i]) {
          next_state_index[i]++;
        } else if (next_state_index[i] == last_state_index[i]) {
          // TODO print when some are done to check length is set correctly
          // TODO TODO TODO why aren't all being set to done on the first iteration?
          next_state_index[i] = DONE;
          nh.loginfo("done with this pin");
        }
      }
    }
    sprintf(str, "prev soonest %lu", soonest_ms);
    nh.logwarn(str);

    unsigned long curr_ms;
    soonest_ms = to_millis(seq.pulse_seq[0].states[next_state_index[0]].t);
    sprintf(str, "init soonest %lu", soonest_ms);
    nh.logwarn(str);
    for (int i = 0; i < seq.pulse_seq_length; i++) {
      sprintf(str, "i=%d, nsi[i]=%d", i, next_state_index[i]);
      nh.logwarn(str);
      // TODO check for / fix different wraparound than millis() (? still an issue?)
      if (next_state_index[i] != DONE) {
        curr_ms = to_millis(seq.pulse_seq[i].states[next_state_index[i]].t);
        sprintf(str, "curr_ms %lu", curr_ms);
        nh.logwarn(str);
        // TODO fix rollover bug
        if (curr_ms < soonest_ms) {
          soonest_ms = curr_ms;
          nh.logwarn("new soonest");
        }
      }
    }
    // TODO TODO need to do the subtraction before comparison to avoid wraparound problems?
    soonest_ms = soonest_ms - rostime_millis_offset;
    
    sprintf(str, "soonest_ms %lu", soonest_ms);
    nh.loginfo(str);
  }
  */
  // TODO 2500 in state maybe meant i was running off edge of state array?

  // TODO store minimum time in loop above to avoid need to loop until we are there?
  // maybe call this multiple times for each of the above?
  // update pins locked in a specified squarewave until next explicit transition
  update_pwm_pinstates();
}


// TODO consolidate this code w/ similar in loops?
void set_non_pwm_pins() {
  // TODO need to do anything with soonest?
  for (int i = 0; i < MAX_NUM_PINS; i++) {
    if (pins[i] != PIN_NOT_SET && next_state[i] == NOT_PWM) {
      if (ms_on[i] == 0) {
        digitalWrite(pins[i], LOW);
      } else {
        digitalWrite(pins[i], HIGH);
      }
    }
  }
}

void end_sequence() {
  // TODO add debug prints
  for (int i = 0; i < MAX_NUM_PINS; i++) {
    if (pins_for_default[i] != PIN_NOT_SET) {
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
  digitalWrite(LED_BUILTIN, HIGH);
  // TODO delete me
  /*
  digitalWrite(61, HIGH);
  digitalWrite(60, HIGH);
  digitalWrite(6, HIGH);
  */
  //
  //shouldn't be used if if in update_pulses_ros is triggered once
  set_non_pwm_pins();
  while (millis() < end_ms) {
  // while (micros() < end_us) {
    // TODO uncomment me
    update_pulses_ros();
  }
  end_sequence();
  digitalWrite(LED_BUILTIN, LOW);
  nh.loginfo("stimulus arduino finished sequence");
  // TODO how to not lose sync with rosserial_python?
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
  nh.getHardware()->setBaud(9600);
  nh.initNode();

  nh.advertiseService(test_defaults_server);
  nh.advertiseService(test_loadseq_server);
  
  nh.spinOnce();
  // still need to spinOnce somewhere around here?
  nh.advertiseService(defaults_server);
  nh.advertiseService(server);
  
  // keep trying to get params until succesful
  while (!get_params()) {
    nh.spinOnce();
  }
  
  nh.loginfo("stimulus arduino done getting parameters.");
  // TODO maybe timeout here after a certain amount of time?
  while (!defaults_registered) {
    nh.spinOnce();
  }
  nh.loginfo("stimulus arduino got defaults.");
  nh.loginfo("stimulus arduino done setup.");
}

// TODO change things named pulse_XXX to sequence_XXX

void loop() {
  //char str[30];
  //sprintf(str, "before spin %lu", millis());
  //nh.logwarn(str);
  // bounds on how long this will take?
  // TODO debug compile flags to track distribution of times this takes, w/ emphasis on extreme values?
  // for reporting...
  // i think for now i'm just going to block while executing the sequence
  nh.spinOnce();

  if (pulse_registered) {
    if (start_ms <= millis()) {
      char str[30];
      sprintf(str, "starting at %lu", millis());
      nh.logwarn(str);
      update_pulses_blocking();
      sprintf(str, "ending at %lu", millis());
      nh.logwarn(str);
    // TODO also check we are supposed to signal pins
    }
    /* else if (start_ms - signal_ms_before <= millis()) {
      nh.logwarn("signalling pins");
      signal_pins();
      nh.logwarn("done with that");
    }*/
  }

  // TODO why do they always delay?
  // how to prevent this from limiting my timing precision?
  //delay(10);
}
