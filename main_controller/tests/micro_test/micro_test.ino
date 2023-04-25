#define IN1S 9
#define IN1A 32
#define IN1B 33
#define IN2S 10
#define IN2A 35
#define IN2B 34

#define UPA 49
#define UPB 47
#define MDA 51
#define MDB 52
#define LWS 8
#define LWA 30
#define LWB 31

void setup(){
    pinMode(IN1S, OUTPUT);
    pinMode(IN1A, OUTPUT); //intake
    pinMode(IN1B, OUTPUT);
    pinMode(IN2S, OUTPUT);
    pinMode(IN2A, OUTPUT);
    pinMode(IN2B, OUTPUT);

    pinMode(UPA, OUTPUT); //almacen
    pinMode(UPB, OUTPUT);
    pinMode(MDA, OUTPUT);
    pinMode(MDB, OUTPUT);
    pinMode(LWS, OUTPUT);
    pinMode(LWA, OUTPUT);
    pinMode(LWB, OUTPUT);
    
    pinMode(45, INPUT);
    pinMode(44, INPUT);
    Serial.begin(9600);
}

void loop(){
    analogWrite(IN1S, 255); //intake
    analogWrite(IN2S, 255);
    digitalWrite(IN1A, HIGH);
    digitalWrite(IN1B, LOW);
    digitalWrite(IN2A, LOW);
    digitalWrite(IN2B, HIGH);
    
    //pin 1 adelante, pin 2 atras
    analogWrite(LWS, 180); 
    digitalWrite(LWA, true); //abajo
    digitalWrite(LWB, !true);
    analogWrite(MDA, 0);     //arriba
    analogWrite(MDB, 180); 
    analogWrite(UPA, 200);  //medio
    analogWrite(UPB, 0);
    delay(1000);
    digitalWrite(LWA, 0); //abajo
    digitalWrite(LWB, 1);
    analogWrite(MDA, 180);     //arriba
    analogWrite(MDB, 0);
    analogWrite(UPA, 0);  //medio
    analogWrite(UPB, 200);
    delay(1000);
    //analogWrite(35, 150);     //medio
    //analogWrite(36, 0); 
    /*delay(1000);
    analogWrite(35, 0);     //medio
    analogWrite(36, 150);
    delay(1000);*/
    //analogWrite(37, 180);  //arriba
    //analogWrite(38, 0);

    Serial.print(digitalRead(45)); //limit
    Serial.print(" ");
    Serial.print(digitalRead(44)); // cubo
    Serial.print(" ");
    Serial.println(digitalRead(11)); // limit
    delay(100);
}