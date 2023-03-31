void setup(){
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(37, OUTPUT);
    pinMode(38, OUTPUT);

    pinMode(27, INPUT);
    pinMode(30, INPUT);
    pinMode(31, INPUT);
    Serial.begin(9600);
}

void loop(){
    analogWrite(8, 0);
    analogWrite(9, 255);
    analogWrite(10, 255);
    analogWrite(11, 0);
//    analogWrite(37, 255);
//    analogWrite(38, 0);
    Serial.print(digitalRead(27));
    Serial.print(" ");
    Serial.print(digitalRead(30)); // cubo
    Serial.print(" ");
    Serial.println(digitalRead(31)); // limit
    delay(100);
}