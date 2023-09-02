const int dir_pin_A = 7;
const int step_pin_A = 6;

const int steps_bit_A_0 = 3;
const int steps_bit_A_1 = 4;
const int steps_bit_A_2 = 5;

const int steps_bit_B_0 = 14;
const int steps_bit_B_1 = 15;
const int steps_bit_B_2 = 16;

const int dir_pin_B = 8;
const int step_pin_B = 9;

int microPausa = 100;

void setup() {
    pinMode(dir_pin_A, OUTPUT);
    pinMode(step_pin_A, OUTPUT);

    pinMode(dir_pin_B, OUTPUT);
    pinMode(step_pin_B, OUTPUT);

    pinMode(steps_bit_A_0, OUTPUT);
    pinMode(steps_bit_A_1, OUTPUT);
    pinMode(steps_bit_A_2, OUTPUT);

    pinMode(steps_bit_B_0, OUTPUT);
    pinMode(steps_bit_B_1, OUTPUT);
    pinMode(steps_bit_B_2, OUTPUT);
}

void loop() {
  digitalWrite(steps_bit_A_0, HIGH);
  digitalWrite(steps_bit_A_1, HIGH);
  digitalWrite(steps_bit_A_2, HIGH);

  digitalWrite(steps_bit_B_0, HIGH);
  digitalWrite(steps_bit_B_1, HIGH);
  digitalWrite(steps_bit_B_2, HIGH);

  digitalWrite(dir_pin_A, LOW);
  digitalWrite(dir_pin_B, LOW); 
  
  digitalWrite(step_pin_A, HIGH);
  digitalWrite(step_pin_B, HIGH);
  delayMicroseconds(microPausa);
  digitalWrite(step_pin_A, LOW);
  digitalWrite(step_pin_B, LOW);
  delayMicroseconds(microPausa);
}