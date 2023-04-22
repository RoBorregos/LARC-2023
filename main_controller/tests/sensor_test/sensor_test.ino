//encoder tests
#define EN1 2
#define EN2 3
#define EN3 18
#define EN4 19

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

//multiplexor tests
#define S0 A11
#define S1 A12
#define S2 A13
#define S3 A14
#define SIG A15

int last_value[16] = {0};

void setup() {
    // put your setup code here, to run once:
    pinMode(EN1, INPUT_PULLUP);
    pinMode(EN2, INPUT_PULLUP);
    pinMode(EN3, INPUT_PULLUP);
    pinMode(EN4, INPUT_PULLUP);
    Serial.begin(9600);

    //attach interrupt to encoder pins
    attachInterrupt(digitalPinToInterrupt(EN1), encoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EN2), encoder2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EN3), encoder3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EN4), encoder4, CHANGE);

    //multiplexor pinout
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(SIG, INPUT);
    
       
}

void loop() {
    //encoder 
    /*
    Serial.print("EN// E1: ");
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
    //*/

    //multiplexor test
    ///*
    Serial.print("Li//");
    for (int i = 0; i < 16; i++) {
        digitalWrite(S0, i & 1);
        digitalWrite(S1, i & 2);
        digitalWrite(S2, i & 4);
        digitalWrite(S3, i & 8);
        //Serial.print(" ");
        //Serial.print(analogRead(SIG));
        if( abs(analogRead(SIG)-last_value[i]) > 30){
            Serial.print(i);
            Serial.print(" ");
        }
        last_value[i] = analogRead(SIG);
    }
    Serial.println();
    //*/

    delay(100);
}