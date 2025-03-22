char msg[3];
uint8_t steer;
uint8_t left_speed;
uint8_t right_speed;

void setup() {
  Serial.begin(38400);
}

void loop() {
  if (Serial.available() > 0) {
    for(int i = 0; i < 3; i++){
      msg[i] = Serial.read();
      }

      steer = msg[0];
      left_speed = msg[1];
      right_speed = msg[2];
      
      Serial.print(steer);
      Serial.print(" ");
      Serial.print(left_speed);
      Serial.print(" ");
      Serial.print(right_speed);
      Serial.print("\n");
  }
  delay(10);
}
