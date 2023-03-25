//mecanum test
/*
arduino-cli monitor port -p usb:80001/3/0/2
arduino-cli upload -p usb:80001/3/0/2 --fqbn teensy:avr:teensy40 drive_test
arduino-cli compile --fqbn teensy:avr:teensy40 sensor_test
arduino-cli core install --additional-urls https://www.pjrc.com/teensy/td_153/td_153.json  \
arduino-cli board list
*/

//motor pinout
#define FL1 2
#define FL2 3
#define FR1 4
#define FR2 5
#define BL1 6
#define BL2 7
#define BR1 0
#define BR2 1

//robot measures (cm)
#define WHEEL_RADIUS 0.27
#define WHEEL_BASE 15.5
#define WHEEL_TRACK 23.0

float linear_x = 1;
float linear_y = 0;
float angular_z = 0;

float wheel_fl = 0;
float wheel_fr = 0;
float wheel_bl = 0;
float wheel_br = 0;

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
    Serial.begin(9600);
}

void loop() {
    // sigular wheel speed
    wheel_fl = (1/WHEEL_RADIUS)*(linear_x - linear_y - (WHEEL_TRACK + WHEEL_BASE)*angular_z);
    wheel_fr = (1/WHEEL_RADIUS)*(linear_x + linear_y + (WHEEL_TRACK + WHEEL_BASE)*angular_z);
    wheel_bl = (1/WHEEL_RADIUS)*(linear_x + linear_y - (WHEEL_TRACK + WHEEL_BASE)*angular_z);
    wheel_br = (1/WHEEL_RADIUS)*(linear_x - linear_y + (WHEEL_TRACK + WHEEL_BASE)*angular_z);

    //debug speeds
    Serial.print("FL: ");
    Serial.print(wheel_fl);
    Serial.print(" FR: ");
    Serial.print(wheel_fr);
    Serial.print(" BL: ");
    Serial.print(wheel_bl);
    Serial.print(" BR: ");
    Serial.println(wheel_br);

    //motor control
    if (wheel_fl > 0) {
        analogWrite(FL1, wheel_fl*50);
        analogWrite(FL2, 0);
    } else {
        analogWrite(FL1, 0);
        analogWrite(FL2, -wheel_fl*50);
    }
    if (wheel_fr > 0) {
        analogWrite(FR1, wheel_fr*50);
        analogWrite(FR2, 0);
    } else {
        analogWrite(FR1, 0);
        analogWrite(FR2, -wheel_fr*50);
    }
    if (wheel_bl > 0) {
        analogWrite(BL1, wheel_bl*50);
        analogWrite(BL2, 0);
    } else {
        analogWrite(BL1, 0);
        analogWrite(BL2, -wheel_bl*50);
    }
    if (wheel_br > 0) {
        analogWrite(BR1, wheel_br*50);
        analogWrite(BR2, 0);
    } else {
        analogWrite(BR1, 0);
        analogWrite(BR2, -wheel_br*50);
    }

    delay(1500);
    if(linear_x > 0){
        linear_x = 0;
        linear_y = 1;
    } else if(linear_y > 0){
        linear_x = 1;
        linear_y = 0;
    }
}
