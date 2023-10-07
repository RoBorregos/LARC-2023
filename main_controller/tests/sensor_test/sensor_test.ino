//encoder tests
#define EN1 19
#define EN2 3
#define EN3 18
#define EN4 2

int treshold = 300;

bool PRINT_VALUE = false;
bool PRINT_RESULT = false;
bool LIMIT_SWITCH = true;

int limitSwitch = 26;

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
#define S0 14
#define S1 41
#define S2 40
#define S3 39
#define SIG 38

int averages[16] = {0};
void setup() {
    // put your setup code here, to run once:
    pinMode(EN1, INPUT_PULLUP);
    pinMode(EN2, INPUT_PULLUP);
    pinMode(EN3, INPUT_PULLUP);
    pinMode(EN4, INPUT_PULLUP);
    pinMode(limitSwitch, INPUT);
    Serial.begin(115200);

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

    // obtain an average of 100 readings from each sensor
    delay(2000);
    long sums[16] = {0};

    for (int i = 0; i < 100; i++) {
        for (int j = 0; j < 16; j++) {
            digitalWrite(S0, j & 1);
            digitalWrite(S1, j & 2);
            digitalWrite(S2, j & 4);
            digitalWrite(S3, j & 8);
            sums[j] += analogRead(SIG);
        }
    }

    // print the average values
    Serial.println("AVERAGE VALUES");
    for (int i = 0; i < 16; i++) {
        averages[i] = sums[i] / 100;
        Serial.print(sums[i] / 100);
        Serial.print(" ");
    }
    Serial.println();
    delay(2000);
    
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
    if (PRINT_VALUE){
        Serial.print("Li//");
        for (int i = 0; i < 16; i++) {
            digitalWrite(S0, i & 1);
            digitalWrite(S1, i & 2);
            digitalWrite(S2, i & 4);
            digitalWrite(S3, i & 8);
            Serial.print(" ");
            Serial.print(analogRead(SIG));
            /*if( analogRead(SIG) > 800){
                Serial.print(i);
                Serial.print(" ");
            }*/
        }
        Serial.println();
        //*/
    }
    else if (PRINT_RESULT) {
        Serial.print("Li//");
        for (int i = 0; i < 16; i++) {
            digitalWrite(S0, i & 1);
            digitalWrite(S1, i & 2);
            digitalWrite(S2, i & 4);
            digitalWrite(S3, i & 8);
            Serial.print(" ");
            bool result = analogRead(SIG) > averages[i] + treshold;
            Serial.print(result);
            /*if( analogRead(SIG) > 800){
                Serial.print(i);
                Serial.print(" ");
            }*/
        }
        Serial.println(); 
    }
    else if (LIMIT_SWITCH){
        Serial.println(digitalRead(limitSwitch));
    }

    delay(10);
}