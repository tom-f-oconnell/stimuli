
// TODO include some preprocessor conditionals to build with ROS
// stuff if in a catkin package? how to automatically upload?
// or at least facilitate switching between ROS / potential non-
// ROS interface
// TODO add (maybe target specific) preprocessor definition in catkin / cmake to
// support this

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

// TODO include appropriate types header

#include <avr/wdt.h>
#include <ros.h>
// what happens if this isn't included?
#include <ros/time.h>
#include <stimuli/LoadDefaultStates.h>
#include <stimuli/LoadSequence.h>
#include <stimuli/TestTransportLoadDefaultStatesReq.h>
#include <stimuli/TestTransportLoadSequenceReq.h>

namespace stim {
    // TODO compare to builtin related to board to validate?
    // TODO maybe just make arbitrarily large and err if used > # actually
    // available on board?  or invalid pin #s requested?
    const static uint8_t max_num_pins = 12;
    const static uint8_t max_num_function_pairs = 1;
    // TODO put in init_state
    const static uint8_t no_function = 0;
    // TODO make sure 8 bits is big enough for all. pin_not_set? (125?)
    const static int8_t pin_not_set = -1;
    // TODO order of modifiers?
    const static int8_t not_pwm = -1;
    const static int8_t done = -1;

    // TODO test all time calculations for millis / micros rollover robustness
    // (could use setMillis(-NNN))
    // TODO valgrind the output of this + the rosserial code?

    ros::NodeHandle nh;

    boolean debug;

    boolean defaults_registered;
    boolean pulse_registered;

    // TODO is this ever correctly set?? test!
    //stimuli::Sequence seq;

    // TODO fixed width types
    unsigned long start_ms;
    unsigned long end_ms;
    unsigned long rostime_millis_offset;
    unsigned long soonest_ms;

    // TODO make sure all printf specifiers are appropriate
    char pins[max_num_pins];
    char pins_for_default[max_num_pins];
    // TODO maybe make char for consistency?
    // TODO are these all still used?
    unsigned char default_state[max_num_pins];
    unsigned int next_state_index[max_num_pins];
    unsigned int last_state_index[max_num_pins];
    char next_state[max_num_pins];
    unsigned long next_time_ms[max_num_pins];
    unsigned long ms_on[max_num_pins];
    unsigned long ms_off[max_num_pins];
    // replace w/ a boolean flag and signal all pins?
    unsigned char pins_to_signal[max_num_pins];
    unsigned long signal_ms_before;
    int odor_signaling_pin;

    char * function_pair_ids[max_num_function_pairs];
    func_pair function_pairs[max_num_function_pairs];

    // TODO TODO warn if loading something that demands more memory than the
    // arduino has

    // not using params because it (seems) harder to get a parameter list of
    // dynamic size...  not sure why they didn't implement them simiarly,
    // especially considering all param types seem to be contained in
    // std_msgs...
    void load_defaults(const stimuli::LoadDefaultStatesRequest &req, \
      stimuli::LoadDefaultStatesResponse &res) {

      res.receive_time = nh.now();
      nh.loginfo("stimulus arduino receiving defaults...");
      res.request_time = req.header.stamp;
      if (defaults_registered) {
        // TODO replace with warn so i can control state after that? does error
        // actually exit / do something?
        nh.logerror("received default pins states while already having "
            "defaults");
        return;
      }
      char str[30];
      // TODO how to set debug flag dynamically from computer? include in
      // service type since cant seem to reliably nh.getParams before services
      // are negotiated...
      if (debug) {
        nh.loginfo("defaults:");
      }
      // TODO break and err if length > defaults. or don't start.
      if (req.defaults_length > max_num_pins) {
        fail("Trying to set defaults for more pins than available.");
      }
      for (int i=0; i < req.defaults_length; i++) {
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
    // TODO i'm not limited in the number of ServiceServers I can operate at
    // once on the arduino am i?
    ros::ServiceServer<stimuli::LoadDefaultStatesRequest, \
      stimuli::LoadDefaultStatesResponse> \
      defaults_server("load_defaults", &load_defaults);

    // TODO arduino fail w/ error is max_num_pins is > what arduino has?

    // TODO is it any easier having one node initiate this interaction?
    // make any more sense one way? (right now the python node will be
    // pushing updates)
    // TODO update docs to remove :: and just concatenate? or am i missing
    // something?
    // TODO what does it mean to have the ampersand here?
    void load_next_sequence(const stimuli::LoadSequenceRequest &req, \
      stimuli::LoadSequenceResponse &res) {

      res.receive_time = nh.now();
      res.request_time = req.header.stamp;

      if (pulse_registered) {
        fail("was sent another set of stimulus info before first expired");
      }
      clear_pins();
     
      // TODO TODO TODO does this work or potentially fail (dangling pointer)?
      // need to deep copy or is there easier workaround?
      // TODO test
      //seq = req.seq;

      /*
      char str[30];
      sprintf(str, "seq_length %lu", req.seq.seq_length);
      nh.loginfo(str);
     
      for (int i=0;i<req.seq.seq_length;i++) {
        sprintf(str, "i %d pin %d", i, req.seq.pins[i]);
        nh.loginfo(str);
        sprintf(str, "s on %lu off %lu", req.seq.seq[i].s.ms_on, \
          req.seq.seq[i].s.ms_off);
        nh.loginfo(str);
      }
      */

      // TODO TODO maybe this should not be sent separately? (just a flag
      // parameter)
      for (int i=0; i < req.pins_to_signal_length; i++) {
        pins_to_signal[i] = req.pins_to_signal[i];
      }

      // fail if difference of left two terms is negative?

      unsigned long ros_now_ms = to_millis(nh.now());
      unsigned long now_ms = millis();
      rostime_millis_offset = ros_now_ms - now_ms;
      // TODO TODO can i definitely assume start_ms is correct? test!
      start_ms = to_millis(req.seq.start) - rostime_millis_offset;
      // TODO this actually a keyword problem ("end")?
      end_ms = to_millis(req.seq.end) - rostime_millis_offset;

      char str[30];
      /*
      if (debug) {
        sprintf(str, "ros_now_ms %lu", ros_now_ms);
        nh.logwarn(str);
        sprintf(str, "now_ms %lu", now_ms);
        nh.logwarn(str);
        sprintf(str, "rostime_millis_offset %lu", rostime_millis_offset);
        nh.logwarn(str);
        // TODO TODO i need to get it s.t. start_ms is < millis() here,
        // preferably by a little bit of a margin

        sprintf(str, "start_ms %lu", start_ms);
        nh.logwarn(str);
        sprintf(str, "end_ms %lu", end_ms);
        nh.logwarn(str);

        // TODO this is correct. why is it different from what i'm getting
        // above?
        // TODO TODO still a problem?
        int diff = req.seq.end.sec - req.seq.start.sec;
        sprintf(str, "duration secs via ros %d", diff);
        nh.logwarn(str);
      }
      */

      // TODO either guarantee this list is sorted before leaving python or find
      // minimum here
      soonest_ms = to_millis(req.seq.seq[0].t);
      
      stimuli::Transition curr;
      unsigned long curr_ms;

      // TODO maybe assert seq.seq and seq.pins have same length?
      unsigned char j = 0;
      for (int i=0;i<req.seq.seq_length;i++) {
        // TODO test
        // TODO plan to sort python side, but will assert that time was not less
        // than (?) last but what about wraparound? not a problem i suppose with
        // the 64 bit ros time?
        if (req.seq.pins[i] != pins[j]) {
          if (j != 0) {
            // will need to set last_state_index for last pin
            // at the end of the loop
            last_state_index[j-1] = i-1;
          }
          pins[j] = req.seq.pins[i];
          // assumes soonest is first (receiving list sorted within each pin)
          next_state_index[j] = i;
       
          curr = req.seq.seq[i];
          
          // cast?
          ms_on[j] = curr.s.ms_on;
          ms_off[j] = curr.s.ms_off;
          if (ms_on[j] == 0 || ms_off[j] == 0) {
            next_state[j] = not_pwm;
          } else {
            next_state[j] = HIGH;
          }

          // TODO only if pwm?
          // TODO uniform start or set each to first transition time?
          // assume pins already in default states by start?
          // TODO TODO fix. this makes first s.t meaningless (pins set to next
          // state here)
          next_time_ms[j] = start_ms;

          // TODO despite sorting in python, this might be nice to the extent i
          // can reuse code for inevitable sorting for second soonest? use tree
          // / other nice data structure?
          curr_ms = to_millis(curr.t);
          // TODO subtract before comparing
          if (curr_ms < soonest_ms) {
            soonest_ms = curr_ms;
          }
          j++;
        }
      }
      last_state_index[j] = req.seq.seq_length;
      soonest_ms = soonest_ms - rostime_millis_offset;

      // TODO  but i supposed the problem with this is that if no states are
      // counted as ~ start above it is unlikely any will be set in first
      // iteration
      //soonest_ms = start_ms;
      
      // TODO dynamically allocate? or use some kind of copy constructor and
      // just keep request obj around?
      // TODO this flag does not prevent callback from overwriting stored
      // LoadSequenceRequest object (only spinOnce-ing while this flag is true
      // should prevent it, but has problems)
      pulse_registered = true;
    }
    ros::ServiceServer<stimuli::LoadSequenceRequest, \
      stimuli::LoadSequenceResponse> server("load_seq", &load_next_sequence);

    void register_function_pair(char * id, void (*f_on)(), void (*f_off)()) {
        
    }

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

    // TODO way to specify OR / sum type for param, since implementation is
    // identical for all 3?
    // TODO auto required if no default?
    bool get_param(const char* n, int* param, const int* def, \
      bool required = false, int len = 1, int timeout = 1000) {

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

    bool get_param(const char* n, float* param, const float* def, \
      bool required = false, int len = 1, int timeout = 1000) {

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

    // TODO how to define default w/ const, while allowing its value to be
    // assigned to param?
    bool get_param(const char *n, char **param, char** def, \
      bool required = false, int len = 1, int timeout = 1000) {

      bool success;
      success = nh.getParam(n, param);
      if (!success) {
        if (def != NULL) {
          // sufficient? TODO test
          *param = *def;
        } else if (required) {
          // TODO concatenate w/ param name (will need to use a diff type
          // though) should not return. board will reset.
          fail("could not get parameter");
        } else {
          return false;
        }
      }
      return true;
    }

    // is this syntax with default args correct?
    bool get_param(const char* n, int* param, \
      bool required = false, int len = 1, int timeout = 1000) {

      return get_param(n, param, NULL, required = required, len = len, \
        timeout = timeout);
    }

    bool get_param(const char* n, float* param, \
      bool required = false, int len = 1, int timeout = 1000) {

      return get_param(n, param, NULL, required = required, len = len, \
        timeout = timeout);
    }

    bool get_param(const char *n, char **param, \
      bool required = false, int len = 1, int timeout = 1000) {

      return get_param(n, param, NULL, required = required, len = len, \
        timeout = timeout);
    }

    // TODO TODO keep trying to get params if any requests are not successful?

    // TODO modify these library functions so they can take string or any kind
    // of const char input? what does Serial.print do?
    // TODO check for republishing node or fail (will be missing important
    // parameters)
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
      if (their_max > max_num_pins) {
        fail("Arduino can't dynamically allocate pins. Change max_num_pins "\
          "in Arduino code to a higher value");
      }

      // TODO way to load them into variable of same name somehow (or compile
      // time macro for it?)?  (so i can operate on a list of param names)
      /*
        int default_odor_signaling_pin = pin_not_set;
        get_param("olf/odor_signaling_pin", &odor_signaling_pin, \
          &default_odor_signaling_pin);

        if (odor_signaling_pin != pin_not_set) {
        nh.logwarn("no odor signaling pin set. set the parameter"\
          " olf/odor_signaling_pin if you'd like to signal to a DAQ");
        pinMode(odor_signaling_pin, OUTPUT);
        }
      */
      nh.loginfo("stimulus arduino done loading parameters");
      return success;
    }

    static inline void init_state() {
      for (int i=0; i<max_num_pins; i++) {
        pins_for_default[i] = pin_not_set;
        pins[i] = pin_not_set;
        pins_to_signal[i] = pin_not_set;
        // next_time_ms elements are undefined unless next_state is not this
        // value
        next_state[i] = not_pwm;
      }
      for (int i=0; i<max_num_function_pairs; i++) {
        function_pair_ids[i] = "";
        function_pairs[i].f_on = no_function;
        function_pairs[i].f_off = no_function;
      }
      defaults_registered = false;
      pulse_registered = false;
      // TODO TODO set via parameter
      signal_ms_before = 500;
    }

    // TODO consolidate w/ init_state? move some of that here?
    static inline void clear_pins() {
      for (int i=0; i<max_num_pins; i++) {
        pins[i] = pin_not_set;
        pins_to_signal[i] = pin_not_set;
      }
    }

    // TODO maybe dont use delays?
    // get parameters re: signalling from ROS?
    // signal to the data acquisition which olfactometer pin we will pulse for
    // this trial ~2 ms period square wave. # pulses = pin #
    static inline void signal_pin(unsigned char pin) {
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

    static inline void signal_pins() {
      for (int i=0; i<max_num_pins; i++) {
        if (pins_to_signal[i] != pin_not_set) {
          signal_pin(pins_to_signal[i]);
        }
      }
    }

    // TODO unit test
    static inline unsigned long to_millis(ros::Time t) {
      return (unsigned long) (t.sec * 1000ull + t.nsec / 1000000ull);
    }

    static inline unsigned long to_micros(ros::Time t) {
      // TODO fix + test
      return (unsigned long) (t.sec * 1000000ull + t.nsec / 1000ull);
    }

    static inline void update_pwm_pinstates() {
      for (int i=0; i<max_num_pins; i++) {
        if (pins[i] != pin_not_set) {
          if (next_state[i] != not_pwm) {
            // TODO make robust to rollover
            if (next_time_ms[i] <= millis()) {
              // TODO maybe use port manipulation
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
          }
        } else {
          // if we reach an element of pins that is pin_not_set, no elements
          // after will be set
          return;
        }
      }
    }

    // TODO (fix this) soonest is broken, but as long as i only have one state
    // per sequence, it should work for now
    // TODO maybe spinOnce occasionally in here?
    static inline void update_pulses_ros() {
      // update pulses with transitions specified explicitly (slower changes)
      // TODO maybe only check millis periodically here, to update pwm better
      // (or call that multiple times?)
      //unsigned long now_ms = millis();
      //char str[30];
      // TODO fix rollover
      //sprintf(str, "now %lu", now_ms);
      //nh.loginfo(str);
      
      // TODO wait, was initial value of soonest off? i thought this loop should
      // be
      // triggered once at beginning?
      /*
      if (soonest_ms <= now_ms) {
        for (int i=0; i < seq.seq_length; i++) {
          sprintf(str, "p %d done %d", pins[i], next_state_index[i]);
          nh.loginfo(str);
          // TODO TODO fix. try to compare durations rather than times...
          // rollover!
          // TODO fix part about states after message redefinition
          if (next_state_index[i] != done &&
          (to_millis(seq.seq[i].states[next_state_index[i]].t) - \
            rostime_millis_offset) <= now_ms) {

            // TODO not sure this can be trusted (it probably can as long as we
            // can guarantee callback isn't reached to load another one, which
            // we can, but not spinOnce-ing)
            // TODO fix state update
            stimuli::State s = seq.seq[i].states[next_state_index[i]].s;
            sprintf(str, "new state on %lu off %lu", s.ms_on, s.ms_off);
            nh.loginfo(str);
            if (s.ms_on == 0) {
              digitalWrite(seq.pins[i], LOW);
              next_state[i] = not_pwm;
              nh.loginfo("setting not_pwm");
            } else {
              digitalWrite(seq.pins[i], HIGH);
              if (s.ms_off != 0) {
                sprintf(str, "s.ms_off = %lu", s.ms_off);
                nh.loginfo(str);
                nh.loginfo("setting PWM!!!");
                next_state[i] = LOW;
                next_time_ms[i] = now_ms + ms_on[i];
              } else {
                nh.loginfo("setting not_pwm");
                next_state[i] = not_pwm;
              }
            }
            
            sprintf(str, "n %d l %d", next_state_index[i], last_state_index[i]);
            nh.loginfo(str);
            
            if (next_state_index[i] < last_state_index[i]) {
              next_state_index[i]++;
            } else if (next_state_index[i] == last_state_index[i]) {
              // TODO print when some are done to check length is set correctly
              // TODO TODO TODO why aren't all being set to done on the first
              // iteration?
              next_state_index[i] = done;
              nh.loginfo("done with this pin");
            }
          }
        }
        sprintf(str, "prev soonest %lu", soonest_ms);
        nh.logwarn(str);

        unsigned long curr_ms;
        // TODO this will only work if i sort by time in python. will still need
        // to increment index here
        // fix
        soonest_ms = to_millis(seq.seq[0].t);
        sprintf(str, "init soonest %lu", soonest_ms);
        nh.logwarn(str);
        for (int i=0; i < seq.seq_length; i++) {
          sprintf(str, "i=%d, nsi[i]=%d", i, next_state_index[i]);
          nh.logwarn(str);
          // TODO check for / fix different wraparound than millis() (? still an
          // issue?)
          if (next_state_index[i] != done) {
            // TODO fix given sequence message redefinition
            curr_ms = to_millis(seq.seq[i].states[next_state_index[i]].t);
            sprintf(str, "curr_ms %lu", curr_ms);
            nh.logwarn(str);
            // TODO fix rollover bug
            if (curr_ms < soonest_ms) {
              soonest_ms = curr_ms;
              nh.logwarn("new soonest");
            }
          }
        }
        // TODO TODO need to do the subtraction before comparison to avoid
        // wraparound problems?
        soonest_ms = soonest_ms - rostime_millis_offset;
        
        sprintf(str, "soonest_ms %lu", soonest_ms);
        nh.loginfo(str);
      }
      */
      // TODO store minimum time in loop above to avoid need to loop until we
      // are there?  maybe call this multiple times for each of the above?
      // update pins locked in a specified squarewave until next explicit
      // transition
      update_pwm_pinstates();
    }


    /* Sets pins that have the same state for the duration of a sequence. */
    static inline void set_non_pwm_pins() {
      // TODO need to do anything with soonest?
      for (int i=0; i<max_num_pins; i++) {
        if (pins[i] != pin_not_set && next_state[i] == not_pwm) {
          if (ms_on[i] == 0) {
            // TODO port manipulations
            digitalWrite(pins[i], LOW);
          } else {
            digitalWrite(pins[i], HIGH);
          }
        }
      }
    }

    static inline void end_sequence() {
      // TODO add debug prints
      for (int i=0; i<max_num_pins; i++) {
        if (pins_for_default[i] != pin_not_set) {
          digitalWrite(pins_for_default[i], default_state[i]);
        } else {
          break;
        }
      }
      pulse_registered = false;
    }

    // TODO it seems the pulses were a little jittery? why do they appear to
    // jump low briefly?  solution?
    // TODO TODO is the above still an issue?
    static inline void update_pulses_blocking() {
      nh.loginfo("stimulus arduino beginning sequence");
      digitalWrite(LED_BUILTIN, HIGH);
      set_non_pwm_pins();
      while (millis() <= end_ms) {
      // while (micros() < end_us) {
        update_pulses_ros();
      }
      end_sequence();
      digitalWrite(LED_BUILTIN, LOW);
      nh.loginfo("stimulus arduino finished sequence");
    }

    // TODO change things named pulse_XXX to sequence_XXX
    
    void init() {
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
    
    // worth an inline declaration? would need to declare in header / extern?
    void update() {
      //char str[30];
      //sprintf(str, "before spin %lu", millis());
      //nh.logwarn(str);
      // bounds on how long this will take?
      // TODO debug compile flags to track distribution of times this takes, w/
      // emphasis on extreme values? for reporting.
      // i think for now i'm just going to block while executing the sequence
      nh.spinOnce();

      if (pulse_registered) {
        if (start_ms <= millis()) {
          update_pulses_blocking();
        }
        /* else if (start_ms - signal_ms_before <= millis()) {
          nh.logwarn("signalling pins");
          signal_pins();
          nh.logwarn("done with that");
        }*/
      }
    }
}
