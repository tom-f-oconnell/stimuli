
#define LED 13

const int num_odors = 8;
const int min_olfactometer_pin = 54;
const int max_olfactometer_pin = min_olfactometer_pin + num_odors - 1;

void setup() {
  Serial.begin(9600);
  Serial.print("Testing olfactometer valves on Arduino pins ");
  Serial.print(min_olfactometer_pin);
  Serial.print(" through ");
  Serial.println(max_olfactometer_pin);
  
  for (int i=min_olfactometer_pin;i<=max_olfactometer_pin;i++) {
    pinMode(i, OUTPUT);
  }
  
  for (int i=min_olfactometer_pin;i<=max_olfactometer_pin;i++) {
    digitalWrite(i, LOW);
  }
}

void loop() {
  for (int i=min_olfactometer_pin;i<=max_olfactometer_pin;i++) {
    //for (int j=0;j<3;j++) {
      Serial.println(i);
      digitalWrite(i, HIGH);
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(i, LOW);
      digitalWrite(LED, LOW);
      delay(2500);
    //}
  }
}
