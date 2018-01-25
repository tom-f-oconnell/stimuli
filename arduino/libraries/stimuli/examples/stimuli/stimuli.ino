
#include <stimuli.h>

using namespace stim;

void setup() {
  // sets initial state and disables watchdog timer
  stim::init();
}

void loop() {
  // updates state of communication with computer
  // and begins (blocking) pulse trains when appropriate
  // TODO make it more explicit what's happening in here? or two options:
  // one like this, and one where the waiting is explicit, so user can do
  // something when not in pulse train?
  stim::update();
}
