void setup(){
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
}

void loop(){
    analogWrite(8, 255);
    analogWrite(9, 0);
    analogWrite(10, 0);
    analogWrite(11, 255);
}