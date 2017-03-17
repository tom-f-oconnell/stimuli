
#ifndef STIM_PARAMS
#error "need to include a parameter .h file that defines STIM_PARAMS. see examples."
#endif

#include <RBD_Timer.h>

// will hold an integer corresponding to pin number to be pulsed in each index
// -1 in other indices
int odor_buffer[max_num_odors];
int trial_index;
// TODO label more descriptively
int flag;

//create timer object
RBD::Timer trialTimer;
RBD::Timer OlfStartTimer;
RBD::Timer OlfLenTimer;

void init_pinstates() {
  if (SEPARATE_BALANCE_VALVE) {
    pinMode(balance_pin, OUTPUT);
  }
  pinMode(scopePin, OUTPUT);
  pinMode(odor_signaling_pin, OUTPUT);

  // TODO check
  if (SEPARATE_BALANCE_VALVE) {
    if (BALANCE_NORMALLY_OPEN) {
      digitalWrite(balance_pin, LOW);
    } else {
      digitalWrite(balance_pin, HIGH);
    }
  }
  digitalWrite(scopePin, LOW);
  digitalWrite(odor_signaling_pin, LOW);

  for (int i = min_olfactometer_pin; i <= max_olfactometer_pin; i++) {
    pinMode(i, OUTPUT);
  }
  for (int i = min_olfactometer_pin; i <= max_olfactometer_pin; i++) {
    digitalWrite(i, LOW);
  }
}

void init_everything() {
  /*
  delay(500);
  Serial.println("waiting...");
  Serial.flush();
  */
  
  init_pinstates();
  // indices not holding a pin to be used in a trial should be -1
  for (int i=0;i<max_num_odors;i++) {
    odor_buffer[i] = -1;
  }
  flag = 0;
  trial_index = 0;

  String msg;
  while (true) {
    if (Serial.available() > 0) {
      msg = Serial.readString();
      if (msg.equals("start")) {
        //Serial.println("starting...");
        break;
      }
    }
  }

  // TODO casting order correct? (not a problem now, but potential for overflow)
  trialTimer.setTimeout((unsigned long) scopeLen * 1000); // timer runs from start to end of a trial
  OlfStartTimer.setTimeout((unsigned long) odorPulseOnset * 1000);
  OlfLenTimer.setTimeout((unsigned long) odorPulseLen_ms);

  // seemed like this might not have worked here...
  // from evidence in code as received from Remy
  trialTimer.restart();
  
  OlfStartTimer.stop();
  OlfLenTimer.stop();
}

/** Sends the trial_index to the listening Python script.
 *  Expects a list of integers, one per line, 
 *  that are the pins to be used on the current trial.
 *  Expects a -1 after list of pins.
 *  Enters these integers in to the odor_buffer.
 *  
 *  Returns true if we are done with the trial / block,
 *  false otherwise.
 */
boolean fill_odor_buffer() {
  // tell the Python script we are ready to receive
  // which pins should be high in the next trial
  Serial.println(trial_index);

  int curr;
  for (int i=0;i<max_num_odors;i++) {
    curr = Serial.parseInt();

    // parseInt returns 0 after Serial.setTimeout()
    // inconvenient b/c ambiguous w/ pin 0, but we 
    // don't ever want to set that pin anyway
    if (curr == 0 or curr == -2) {
      return true;
    }
    
    odor_buffer[i] = curr;
    if (curr == -1) {
      return false;
    }
  }
  // everything was filled with a valid pin number
  return false;
}

// signal to the data acquisition which olfactometer pin we will pulse for this trial
// ~2 ms period square wave. # pulses = pin #
void signal_odor(unsigned int pin) {
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

void output_odor_buffer() {
  Serial.print("odor_buffer: ");
  for (int i=0;i<max_num_odors;i++) {
     Serial.print(odor_buffer[i]);
     Serial.print(" ");
  }
  Serial.println("");
}

void setup_fn() {
  Serial.begin(57600);
  init_everything();
}

// the loop function runs over and over again forever
void loop_fn() {
  if (trialTimer.onActive()) {
    trial_index = trial_index + 1;

    // this will modify the buffer accessed below
    boolean done = fill_odor_buffer();
    
    // return (should) function like 'continue' normally does, but for the Arduino's main loop()
    // fill_odor_buffer will set done to true if Python sends a -2 over serial (indicating block over)
    if (done) {
      init_everything();
      return;
    }

    boolean have_data = false;
    for (int i=0;i<max_num_odors;i++) {
      if (odor_buffer[i] != -1) {
        have_data = true;
      }
    }
    if (!have_data) {
      init_everything();
      return;
    }

    // it is important that this comes after turning the scope pin on
    // because this takes ~20 milliseconds per odor (though could be made faster)
    for (int i=0;i<max_num_odors;i++) {
      if (odor_buffer[i] != -1) {
        signal_odor(odor_buffer[i]);
      }
    }
    output_odor_buffer();
    
    OlfStartTimer.restart();
    digitalWrite(scopePin, HIGH);
  } // end onActive check

  if (trialTimer.isActive()) {
    if (OlfStartTimer.onExpired()) {
      OlfLenTimer.restart();
      flag = 1;
    }

    if (OlfLenTimer.onActive()) {
      // if we don't want to power the solenoids with a high duty cycle, 
      // could switch outputs of balance pin and invert logic
      // (we want the balance pin to provide an equivalent path for flow whenever the odor valves 
      //  are not open, to keep the total airflow constant)
      if (SEPARATE_BALANCE_VALVE) {
        if (BALANCE_NORMALLY_OPEN) {
          digitalWrite(balance_pin, HIGH);
        } else {
          // otherwise treat the logic the same as the other pins
          digitalWrite(balance_pin, LOW);
        }
      }
      for (int i=0;i<max_num_odors;i++) {
        if (odor_buffer[i] != -1)
          digitalWrite(odor_buffer[i], HIGH);
      }
      digitalWrite(odor_signaling_pin, HIGH);
    }
    // TODO documentation makes it seem like it would work without flag (onExpired only returns true once per .restart())
    if (OlfLenTimer.onExpired() && flag == 1) {
      for (int i=0;i<max_num_odors;i++) {
        if (odor_buffer[i] != -1) {
          digitalWrite(odor_buffer[i], LOW);
          odor_buffer[i] = -1;
        }
      }

      if (SEPARATE_BALANCE_VALVE) {
        if (BALANCE_NORMALLY_OPEN) {
          digitalWrite(balance_pin, LOW);
        } else {
          digitalWrite(balance_pin, HIGH);
        }
      }
      
      digitalWrite(odor_signaling_pin, LOW);
    }
  } // end isActive check

  // assumes we don't reach this branch if done with block / experiment
  if (trialTimer.onExpired()) {
    digitalWrite(scopePin, LOW);
    
    // TODO be careful mixing delays and timers
    // CHECK!!!
    delay(((long) ITI) * 1000);
    trialTimer.restart();
  } // end onExpired check
} // end loop

