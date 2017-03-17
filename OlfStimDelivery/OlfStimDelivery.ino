
#include "170317_mixtures_stimparams.h"
#include "OlfStimDelivery.h"

// setup_fn and loop_fn are both defined in OlfStimDelivery.h, which is copied here
// by the preprocessor, beneath the parameters included in the first line

void setup() {
    setup_fn();
}

void loop() {
    loop_fn();
}
