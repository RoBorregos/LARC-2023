
void setup() {
    pinMode(13, OUTPUT);
    Serial.begin(9600);
}

void loop() {
//    Serial.println( digitalRead(13) ); 
    //Serial.println( Serial.read() );
    if( Serial.read() != -1 ){
        digitalWrite(13, HIGH);
    }
    else{
        digitalWrite(13, LOW);
    }
    delay(100);
}
