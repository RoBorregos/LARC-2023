//encoder tests

#define EN1 12
#define EN2 23
#define EN3 26
#define EN4 27

volatile long en1_count = 0;
volatile long en2_count = 0;
volatile long en3_count = 0;
volatile long en4_count = 0;

volatile long en1_prev = 0;
volatile long en2_prev = 0;
volatile long en3_prev = 0;
volatile long en4_prev = 0;

void encoder1() {
    en1_count++;
}

void encoder2() {
    en2_count++;
}

void encoder3() {
    en3_count++;
}

void encoder4() {
    en4_count++;
}

void setup() {
    // put your setup code here, to run once:
    pinMode(EN1, INPUT_PULLUP);
    pinMode(EN2, INPUT_PULLUP);
    pinMode(EN3, INPUT_PULLUP);
    pinMode(EN4, INPUT_PULLUP);
    Serial.begin(9600);

    //attach interrupt to encoder pins
    attachInterrupt(digitalPinToInterrupt(EN1), encoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(EN2), encoder2, RISING);
    attachInterrupt(digitalPinToInterrupt(EN3), encoder3, RISING);
    attachInterrupt(digitalPinToInterrupt(EN4), encoder4, RISING);
}

void loop() {
    //long current_en1_count = en1_count;
    Serial.print("E1: ");
    Serial.print(digitalRead(EN1));
    Serial.print(" ");
    Serial.print(en1_count);
    Serial.print(" E2: ");
    Serial.print(digitalRead(EN2));
    Serial.print(" ");
    Serial.print(en2_count);
    Serial.print(" E3: ");
    Serial.print(digitalRead(EN3));
    Serial.print(" ");
    Serial.print(en3_count);
    Serial.print(" E4: ");
    Serial.print(digitalRead(EN4));
    Serial.print(" ");
    Serial.println(en4_count);

    delay(100);
}