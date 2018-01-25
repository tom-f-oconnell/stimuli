
#include <stimuli.h>

// TODO include enum type from generated ROS library (w/ USER_FUNCTION_1, etc)?
// or in stimuli? or just have an index and a max? max as param? const?

using namespace stim;

void function_on() {
  digitalWrite(LED_BUILTIN, HIGH);
}

void function_off() {
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  // sets initial state and disables watchdog timer
  stim::init();

  // registers a pair of functions that have opposite effects
  // TODO elaborate after implementing
  stim::register_function_pair(0, &function_on, &function_off);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // updates state of communication with computer
  // and begins (blocking) pulse trains when appropriate
  stim::update();
}
