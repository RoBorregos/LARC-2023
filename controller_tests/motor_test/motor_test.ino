//motor tests

//motor pinout
#define FL1 5
#define FL2 4
#define FR1 3
#define FR2 2
#define BL1 6
#define BL2 7
#define BR1 0
#define BR2 1

void setup() {
    // put your setup code here, to run once:
    pinMode(FL1, OUTPUT);
    pinMode(FL2, OUTPUT);
    pinMode(FR1, OUTPUT);
    pinMode(FR2, OUTPUT);
    pinMode(BL1, OUTPUT);
    pinMode(BL2, OUTPUT);
    pinMode(BR1, OUTPUT);
    pinMode(BR2, OUTPUT);
}

void loop() {
    analogWrite(FL1, 255);
    analogWrite(FL2, 0);
    delay(1500);
    analogWrite(FL1, 0);
    analogWrite(FL2, 255);
    delay(1000);
    analogWrite(FL1, 0);
    analogWrite(FL2, 0);
    analogWrite(FR1, 255);
    analogWrite(FR2, 0);
    delay(1500);
    analogWrite(FR1, 0);
    analogWrite(FR2, 255);
    delay(1000);
    analogWrite(FR1, 0);
    analogWrite(FR2, 0);
    analogWrite(BL1, 255);
    analogWrite(BL2, 0);
    delay(1500);
    analogWrite(BL1, 0);
    analogWrite(BL2, 255);
    delay(1000);
    analogWrite(BL1, 0);
    analogWrite(BL2, 0);
    analogWrite(BR1, 255);
    analogWrite(BR2, 0);
    delay(1500);
    analogWrite(BR1, 0);
    analogWrite(BR2, 255);
    delay(1000);
    analogWrite(BR1, 0);
    analogWrite(BR2, 0);

}
