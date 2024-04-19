void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

class ScanKB {
private:
  int wp[4] = { 8, 9, 10, 11 };
  int hp[4] = { 4, 5, 6, 7 };
public:
  ScanKB() {
    for (int i = 0; i < 4; ++i) {
      pinMode(wp[i], OUTPUT);
      pinMode(hp[i], INPUT);
      digitalWrite(wp[i], 0);
    }
  }
  void scan() {
    for (int i = 0; i < 4; ++i) {
      digitalWrite(wp[i], 1);
      for (int j = 0; j < 4; ++j) {
        if (digitalRead(hp[j]))
        Serial.printf("%d ", i * 4 + j);
      }
      digitalWrite(wp[i], 0);
    }
    Serial.printf("\n");
  }
};
ScanKB kb;
void loop() {
  kb.scan();
  delay(50);
}
