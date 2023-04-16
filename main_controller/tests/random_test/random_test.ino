void setup(){
    pinMode(8, OUTPUT); //intake
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);

    pinMode(35, OUTPUT); //almacen
    pinMode(36, OUTPUT);
    pinMode(37, OUTPUT);
    pinMode(38, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(15, OUTPUT);
    pinMode(27, INPUT);
    pinMode(30, INPUT);
    pinMode(31, INPUT);
    Serial.begin(9600);
}

void loop(){
    /*analogWrite(8, 255); //intake
    analogWrite(9, 0);
    analogWrite(10, 0);
    analogWrite(11, 255);*/
    
    //pin 1 adelante, pin 2 atras
    analogWrite(14, 0); //abajo
    analogWrite(15, 255);
    delay(1000);
    analogWrite(14, 255); //abajo
    analogWrite(15, 0);
    delay(1000);
    //analogWrite(35, 150);     //medio
    //analogWrite(36, 0); 
    /*delay(1000);
    analogWrite(35, 0);     //medio
    analogWrite(36, 150);
    delay(1000);*/
    //analogWrite(37, 180);  //arriba
    //analogWrite(38, 0);

    Serial.print(digitalRead(27));
    Serial.print(" ");
    Serial.print(digitalRead(30)); // cubo
    Serial.print(" ");
    Serial.println(digitalRead(31)); // limit
    delay(100);
}