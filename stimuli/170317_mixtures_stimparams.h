
#ifndef STIM_PARAMS
#define STIM_PARAMS

// set true if another valve (its normally open port connected to the mock vial / directly into manifold)
// is to be switched each time any other valve / valve combination is used
const boolean SEPARATE_BALANCE_VALVE = false;

// irrelevant if above is false
const boolean BALANCE_NORMALLY_OPEN = false;

const int scopePin = 3;
const int odor_signaling_pin = 12;   // will send a number of pulses = digital pin # pulsed for current trial

//set stimulus variables
const int balance_pin = -1;
const int max_num_odors = 8;

const int ITI = 90;            // intertrial interval in seconds
                               // *** the time from last time scopePin was high in one trial
                               //     to first time it goes high in the next next ***
const int odorPulseLen_ms = 1000;    // length of the odor pulse in milliseconds
const int scopeLen = 30;       // length of scope acquisition time in seconds
const int odorPulseOnset = 10; // onset time of odor pulse in seconds

// uses all pins in this range as digital outputs
// one valve per pin
// the balance is handled separately if flags are set above
const int min_olfactometer_pin = 4;
const int max_olfactometer_pin = min_olfactometer_pin + max_num_odors - 1;

#else
#error "Only one .h file with stimulus parameters should be included."
#endif
