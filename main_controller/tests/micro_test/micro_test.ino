void setup(){
    pinMode(9, OUTPUT);
    pinMode(32, OUTPUT); //intake
    pinMode(33, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(34, OUTPUT);
    pinMode(35, OUTPUT);

    pinMode(47, OUTPUT); //almacen
    pinMode(49, OUTPUT);
    pinMode(51, OUTPUT);
    pinMode(52, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(15, OUTPUT);
    pinMode(45, INPUT);
    pinMode(44, INPUT);
    pinMode(31, INPUT);
    Serial.begin(9600);
}

void loop(){
    analogWrite(9, 255); //intake
    analogWrite(10, 255);
    digitalWrite(32, HIGH);
    digitalWrite(33, LOW);
    analogWrite(34, LOW);
    analogWrite(35, HIGH);
    
    //pin 1 adelante, pin 2 atras
    analogWrite(14, 200); //abajo
    analogWrite(15, 0);
    analogWrite(47, 180);     //arriba
    analogWrite(49, 0); 
    analogWrite(51, 200);  //medio
    analogWrite(52, 0);
    delay(1000);
    analogWrite(14, 0); //abajo
    analogWrite(15, 200);
    analogWrite(47, 0);     //arriba
    analogWrite(49, 180);
    analogWrite(51, 0);  //medio
    analogWrite(52, 200);
    delay(1000);
    //analogWrite(35, 150);     //medio
    //analogWrite(36, 0); 
    /*delay(1000);
    analogWrite(35, 0);     //medio
    analogWrite(36, 150);
    delay(1000);*/
    //analogWrite(37, 180);  //arriba
    //analogWrite(38, 0);

    Serial.print(digitalRead(45));
    Serial.print(" ");
    Serial.print(digitalRead(44)); // cubo
    Serial.print(" ");
    Serial.println(digitalRead(31)); // limit
    delay(100);
}