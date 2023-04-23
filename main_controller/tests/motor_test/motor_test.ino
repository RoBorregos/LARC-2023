//motor tests
#define SPEED 50

struct motor{
    int pinPwm;
    int pinA;
    int pinB;
};

struct motor br = {5, 25, 24};
struct motor fr = {4, 22, 23};
struct motor bl = {6, 27, 26};
struct motor fl = {7, 28, 29};

void setup() {
    pinMode(fl.pinPwm, OUTPUT);
    pinMode(fl.pinA, OUTPUT);
    pinMode(fl.pinB, OUTPUT);
    pinMode(fr.pinPwm, OUTPUT);
    pinMode(fr.pinA, OUTPUT);
    pinMode(fr.pinB, OUTPUT);
    pinMode(bl.pinPwm, OUTPUT);
    pinMode(bl.pinA, OUTPUT);
    pinMode(bl.pinB, OUTPUT);
    pinMode(br.pinPwm, OUTPUT);
    pinMode(br.pinA, OUTPUT);
    pinMode(br.pinB, OUTPUT);
}

void loop() {
    analogWrite(fl.pinPwm, SPEED);
    analogWrite(fr.pinPwm, SPEED);
    analogWrite(bl.pinPwm, SPEED);
    analogWrite(br.pinPwm, SPEED);
    
    digitalWrite(fl.pinA, HIGH);
    digitalWrite(fl.pinB, LOW);
    delay(1500);
    digitalWrite(fl.pinA, LOW);
    digitalWrite(fl.pinB, HIGH);
    delay(1000);
    digitalWrite(fl.pinA, LOW);
    digitalWrite(fl.pinB, LOW);
    digitalWrite(fr.pinA, HIGH);
    digitalWrite(fr.pinB, LOW);
    delay(1500);
    digitalWrite(fr.pinA, LOW);
    digitalWrite(fr.pinB, HIGH);
    delay(1000);
    digitalWrite(fr.pinA, LOW);
    digitalWrite(fr.pinB, LOW);
    digitalWrite(bl.pinA, HIGH);
    digitalWrite(bl.pinB, LOW);
    delay(1500);
    digitalWrite(bl.pinA, LOW);
    digitalWrite(bl.pinB, HIGH);
    delay(1000);
    digitalWrite(bl.pinA, LOW);
    digitalWrite(bl.pinB, LOW);
    digitalWrite(br.pinA, HIGH);
    digitalWrite(br.pinB, LOW);
    delay(1500);
    digitalWrite(br.pinA, LOW);
    digitalWrite(br.pinB, HIGH);
    delay(1000);
    digitalWrite(br.pinA, LOW);
    digitalWrite(br.pinB, LOW);

}
