const int pin_x = A0;
const int pin_y = A1;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int xPosition = analogRead(pin_x);
  int yPosition = analogRead(pin_y);

  Serial.print("X軸: ");
  Serial.print(xPosition);
  Serial.print(" | Y軸: ");
  Serial.println(yPosition);
  delay(1000);
}
