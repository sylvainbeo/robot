#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_APDS9960.h>
#include <Servo.h>

#include "init.h"


Adafruit_MotorShield motorShield = Adafruit_MotorShield();


// TODO
/*
 * Quand il y aura la boussole, et capteur de position (?), détecter si le robot ne bouge plus  ?
 * 
 */

////////////////////////////////////////////////////////////////////////
class Flasher
{
    // Class Member Variables
    // These are initialized at startup
    int _ledPin;      // the number of the LED pin
    long _OnTime;     // milliseconds of on-time
    long _OffTime;    // milliseconds of off-time

    // These maintain the current state
    int _ledState;                 // ledState used to set the LED
    unsigned long _previousMillis;   // will store last time LED was updated

    // Constructor - creates a Flasher
    // and initializes the member variables and state
    public:
    Flasher(int pin, long on, long off)
    {
        this->_ledPin = pin;
        pinMode(this->_ledPin, OUTPUT);

        this->_OnTime = on;
        this->_OffTime = off;

        this->_ledState = LOW;
        this->_previousMillis = 0;
    }

    void changeDelays(long on, long off) {
        this->_OnTime = on;
        this->_OffTime = off;
    }

    void update(float distance)
    {
        if (distance > 100) {
            changeDelays(100, 400);
        } else if (distance > OBSTACLE_DISTANCE_STOP) {
            changeDelays(50, 200);
        } else {
            changeDelays(10, 50);
        }

        // check to see if it's time to change the state of the LED
        unsigned long currentMillis = millis();

        if ((this->_ledState == HIGH) && (currentMillis - this->_previousMillis >= this->_OnTime))
        {
            this->_ledState = LOW;  // Turn it off
            this->_previousMillis = currentMillis;  // Remember the time
            digitalWrite(this->_ledPin, this->_ledState);  // update the actual LED
        }
        else if ((this->_ledState == LOW) && (currentMillis - this->_previousMillis >= this->_OffTime))
        {
            this->_ledState = HIGH;  // turn it on
            this->_previousMillis = currentMillis;   // Remember the time
            digitalWrite(this->_ledPin, this->_ledState);   // update the actual LED
        }
    }
};


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
class UltraSound
{
    int _triggerPin;
    int _echoPin;

    // Constructor - creates a Sound
    // and initializes the member variables and state
    public:
    UltraSound(int triggerPin, int echoPin)
    {
        this->_triggerPin = triggerPin;
        this->_echoPin = echoPin;

        pinMode(this->_triggerPin, OUTPUT);
        digitalWrite(this->_triggerPin, LOW); // La broche TRIGGER doit être à LOW au repos
        pinMode(this->_echoPin, INPUT);
    }

    float getDistance() // returns the distance (cm)
    {
        long duration, distance;

        digitalWrite(this->_triggerPin, HIGH); // We send a 10us pulse
        delayMicroseconds(10);
        digitalWrite(this->_triggerPin, LOW);

        duration = pulseIn(this->_echoPin, HIGH, 20000); // We wait for the echo to come back, with a timeout of 20ms, which corresponds approximately to 3m

        // pulseIn will only return 0 if it timed out. (or if echoPin was already to 1, but it should not happen)
        if (duration == 0) // If we timed out
        {
            pinMode(this->_echoPin, OUTPUT); // Then we set echo pin to output mode
            digitalWrite(this->_echoPin, LOW); // We send a LOW pulse to the echo pin
            delayMicroseconds(200);
            pinMode(this->_echoPin, INPUT); // And finaly we come back to input mode
        }

        distance = (duration / 2) / 29.1; // We calculate the distance (sound speed in air is aprox. 291m/s), /2 because of the pulse going and coming

        return distance; //We return the result. Here you can find a 0 if we timed out
    }
};


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
class GestureSensor
{

    Adafruit_APDS9960 apds;

    public:
    GestureSensor() {
    }

    init() {
        if (!this->apds.begin()) {
            if (DEBUG)
            Serial.println("failed to initialize device! Please check your wiring.");
        }
        else  {
            if (DEBUG)
            Serial.println("Device initialized!");
        }

      //gesture mode will be entered once proximity mode senses something close
      this->apds.enableProximity(true);
      this->apds.enableGesture(true);
    }

    uint8_t getOrder() {
        //read a gesture from the device
        uint8_t gesture = this->apds.readGesture();
        if (gesture == APDS9960_DOWN) {
            if (DEBUG)
                Serial.println("v");
        }
        if (gesture == APDS9960_UP) {
            if (DEBUG)
                Serial.println("^");
        }
        if (gesture == APDS9960_LEFT) {
            if (DEBUG)
                Serial.println("<");
        }
        if (gesture == APDS9960_RIGHT) {
            if (DEBUG)
                Serial.println(">");
        }
        return gesture;
    }
};


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
class ColorSensor
{

    Adafruit_APDS9960 apds;

    public:
    ColorSensor() {
    }

    init() {
        if (!this->apds.begin()) {
            if (DEBUG)
            Serial.println("failed to initialize device! Please check your wiring.");
        }
        else  {
            if (DEBUG)
                Serial.println("Device initialized!");
        }

      this->apds.enableColor(true);
    }

    uint8_t getColor() {
        uint16_t r, g, b, c;

        //wait for color data to be ready
        while(!this->apds.colorDataReady()){
            delay(5);
        }

        //get the data and print the different channels
        this->apds.getColorData(&r, &g, &b, &c);
        /*
        Serial.print("red: ");
        Serial.print(r);

        Serial.print(" green: ");
        Serial.print(g);

        Serial.print(" blue: ");
        Serial.print(b);

        Serial.print(" clear: ");
        Serial.println(c);
        Serial.println();
        */
        pinMode(12, OUTPUT);
        pinMode(13, OUTPUT);

        if(r > g) {
            digitalWrite(13, HIGH); 
            digitalWrite(12, LOW);
        } else {
            digitalWrite(13, LOW); 
            digitalWrite(12, HIGH);
        }
    }
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
class ReflectanceSensor
{
    int _analogPin = 3;

    public:
    ReflectanceSensor() {
    }

    uint8_t getReflectance() {
        int val = analogRead(this->_analogPin);
        float voltage = (val/1024.0)*5.0;
        if (DEBUG)
            Serial.println(voltage);
        return val;
    }
};


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
class Motor
{
    int _motorPin;
    Adafruit_DCMotor *_motor;

    // Constructor - creates a Sound
    // and initializes the member variables and state
    public:
    Motor(int motorPin)
    {
        this->_motorPin = motorPin;
        pinMode(this->_motorPin, OUTPUT);
        this->_motor = motorShield.getMotor(this->_motorPin);
    }

    void forward(int speed=MAX_SPEED) {
        this->_motor->setSpeed(speed);
        this->_motor->run(FORWARD);
    }

    void backward(int speed=MAX_SPEED) {
        this->_motor->setSpeed(speed);
        this->_motor->run(BACKWARD);
    }

    void stop() {
        this->_motor->run(RELEASE);
    }
};


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
class Movement
{
    int _speed = MAX_SPEED;
    int _maxSpeed;
    int _lowSpeed;
    int _turnSpeed;
    int _turnMode;
    int _turn180Time;
    int _turn90Time;
    int _direction;
    float _distanceFront;
    float _distanceLeft;
    float _distanceRight;
    Motor *_motorFrontLeft;
    Motor *_motorBackLeft;
    Motor *_motorFrontRight;
    Motor *_motorBackRight;

    public:
    Movement(Motor *mFrontLeft, Motor *mBackLeft, Motor *mFrontRight, Motor *mBackRight)
    {
        this->_motorFrontLeft = mFrontLeft;
        this->_motorBackLeft = mBackLeft;
        this->_motorFrontRight = mFrontRight;
        this->_motorBackRight = mBackRight;

        this->_maxSpeed = MAX_SPEED;
        this->_lowSpeed = LOW_SPEED;
        this->_turnSpeed = TURN_SPEED;
        this->_turn180Time = TIME_TURN_180;
        this->_turn90Time = TIME_TURN_90;
    }

    void setSpeed(int speed)
    {
        this->_speed = speed;
    }

    void setMaxSpeed(int speed)
    {
        this->_maxSpeed = speed;
    }

    void setTurnSpeed(int speed)
    {
        this->_turnSpeed = speed;
        this->setTurnTime(speed);
    }

    void setDistanceFront(float distance) {
        this->_distanceFront = distance;
    }

    void setDistanceLeft(float distance) {
        this->_distanceLeft = distance;
    }

    void setDistanceRight(float distance) {
        this->_distanceRight = distance;
    }

    void setTurnMode(int turnMode) {
        this->_turnMode = turnMode;
    }

    void setTurnTime(int speed) {
        this->_turn180Time = map(speed, 0, 255, 500, TIME_TURN_180);
        this->_turn90Time = map(speed, 0, 255, 200, TIME_TURN_90);
    }

    int update() {
        // According to the distanceFront, slow down, and turn (if distanceLeft and distanceRight are OK)
        if (this->_distanceFront > OBSTACLE_DISTANCE_STOP) {
            this->_direction = 90;
            if (this->_distanceFront > OBSTACLE_DISTANCE_SLOWDOWN) {
                this->setSpeed(this->_maxSpeed);
                this->forward();
                this->_direction = 90;
                if (DEBUG) {
                    Serial.println("FORWARD FULL SPEED");
                }
            } else if (this->_distanceFront > OBSTACLE_DISTANCE_STOP) {
                int valeur = map(this->_distanceFront, 10, 100, this->_lowSpeed, this->_maxSpeed); // Conversion en valeur
                this->setSpeed(valeur); // On définit la vitesse de rotation
                this->forward();
                if (DEBUG) {
                    Serial.println("FORWARD ");
                    Serial.println(valeur);
                }
            }
            // Direction adjustments
            this->correctLeft();
            this->correctRight();
        } else {
            // TURN mode
            // Check sensor on the left and right, and turn accordingly
            // there might be a bug there. Because if there are 2 obstacles on left and right, the robot will turn on  the side where the obstacle is the farest.... but while turning, the obstacle may become closer, and then the robot try to turn the other way....stuck !
            // SOLUTION => detect that case (if there are too many left turn / right turn / left turn / right turn...then force a local loop to procede a 180 degree (according to the current speed, make some tests to determine the necessary time to do that)
            if((this->_distanceLeft < this->_distanceRight) && (this->_distanceRight > OBSTACLE_DISTANCE_TURN)) {
                // we have to turn right until the FRONT is clear (_distanceFront > OBSTACLE_DISTANCE_TURN), and the LEFT is away suffisant from the right obstacle (_distanceLeft >    OBSTACLE_DISTANCE_TURN)
                // But i think it will be done by the correct function
                if (DEBUG) {
                    Serial.println("TURN RIGHT");
                }
                this->turnRight();
                this->_direction = 180;
            }
            else if((this->_distanceLeft > this->_distanceRight) && (this->_distanceLeft > OBSTACLE_DISTANCE_TURN)) {
                // we have to turn left until the FRONT is clear (_distanceFront > OBSTACLE_DISTANCE_TURN), and the RIGHT is away suffisant from the left obstacle (_distanceRight > OBSTACLE_DISTANCE_TURN)
                // But i think it will be done by the correct function
                if (DEBUG) {
                    Serial.println("TURN LEFT");
                }
                this->turnLeft();
                this->_direction = 0;
            } else {
                // Do 180 turn
                this->turn180();
                this->_direction = 90;
            }
        }

        return this->_direction;
    }

    void forward() {
        this->_motorFrontLeft->forward(this->_speed);
        this->_motorBackLeft->forward(this->_speed);
        this->_motorFrontRight->forward(this->_speed);
        this->_motorBackRight->forward(this->_speed);
    }

    void backward() {
        this->_motorFrontLeft->backward(this->_speed);
        this->_motorBackLeft->backward(this->_speed);
        this->_motorFrontRight->backward(this->_speed);
        this->_motorBackRight->backward(this->_speed);
    }

    void turnLeft() {
        // if _turnMode == 0 , in place, else smooth
        if(this->_turnMode == 0) {
            this->_motorFrontLeft->backward(this->_turnSpeed);
            this->_motorBackLeft->backward(this->_turnSpeed);
            this->_motorFrontRight->forward(this->_turnSpeed);
            this->_motorBackRight->forward(this->_turnSpeed);
        } else if(this->_turnMode == 1) {
            this->_motorFrontLeft->forward(this->_turnSpeed / 2);
            this->_motorBackLeft->forward(this->_turnSpeed / 2);
            this->_motorFrontRight->forward(this->_turnSpeed);
            this->_motorBackRight->forward(this->_turnSpeed);
        } else {
            this->turn90(0);
        }
    }

    void turnRight() {
        // if _turnMode == 0 , in place, else smooth
        if(this->_turnMode == 0) {
            this->_motorFrontLeft->forward(this->_turnSpeed);
            this->_motorBackLeft->forward(this->_turnSpeed);
            this->_motorFrontRight->backward(this->_turnSpeed);
            this->_motorBackRight->backward(this->_turnSpeed);
        } else if(_turnMode == 1) {
            this->_motorFrontLeft->forward(this->_turnSpeed);
            this->_motorBackLeft->forward(this->_turnSpeed);
            this->_motorFrontRight->forward(this->_turnSpeed / 2);
            this->_motorBackRight->forward(this->_turnSpeed / 2);
        } else {
            this->turn90(1);
        }
    }

    void turn90(int way) { // way = 0: left, 1: right
        unsigned long previousMillis;
        unsigned long currentMillis = millis();
        previousMillis = currentMillis;

        while((currentMillis - previousMillis) <= this->_turn90Time)
        {
            if (DEBUG) {
                Serial.println("TURNING 90");
            }
            currentMillis = millis();
            if(way) {
                this->_motorFrontLeft->forward(this->_turnSpeed);
                this->_motorBackLeft->forward(this->_turnSpeed);
                this->_motorFrontRight->backward(this->_turnSpeed);
                this->_motorBackRight->backward(this->_turnSpeed);
            }
            else {
                this->_motorFrontLeft->backward(this->_turnSpeed);
                this->_motorBackLeft->backward(this->_turnSpeed);
                this->_motorFrontRight->forward(this->_turnSpeed);
                this->_motorBackRight->forward(this->_turnSpeed);
            }
        }
    }

    void turn180() {
        unsigned long previousMillis;
        unsigned long currentMillis = millis();
        previousMillis = currentMillis;

        while((currentMillis - previousMillis) <= this->_turn180Time)
        {
            if (DEBUG) {
                Serial.println("TURNING 180");
            }
            currentMillis = millis();
            this->_motorFrontLeft->backward(this->_turnSpeed);
            this->_motorBackLeft->backward(this->_turnSpeed);
            this->_motorFrontRight->forward(this->_turnSpeed);
            this->_motorBackRight->forward(this->_turnSpeed);
        }
    }

    void correctLeft() {
        // If an obstacle get closer on the LEFT, then slow down the RIGHT motors to correct the direction
        if (this->_distanceLeft < OBSTACLE_DISTANCE_SIDE_CORRECT) {
            if (DEBUG) {
                Serial.println("FORWARD: correcting LEFT");
            }
            this->_motorFrontRight->backward(CORRECT_SPEED);
            this->_motorBackRight->backward(CORRECT_SPEED);
        }
    }

    void correctRight() {
        // If an obstacle get closer on the RIGHT, then slow down the LEFT motors to corect the direction
        if (this->_distanceRight < OBSTACLE_DISTANCE_SIDE_CORRECT) {
             if (DEBUG) {
                 Serial.println("FORWARD: correcting RIGHT");
             }
            this->_motorFrontLeft->backward(CORRECT_SPEED);
            this->_motorBackLeft->backward(CORRECT_SPEED);
        }
    }

    void stop() {
        this->_motorFrontLeft->stop();
        this->_motorBackLeft->stop();
        this->_motorFrontRight->stop();
        this->_motorBackRight->stop();
    }
};


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
/*
class ServoControl
{
    Servo *_servo;

    // Constructor - creates a Sound
    // and initializes the member variables and state
    public:
    ServoControl(Servo *servo, int servoPin)
    {
        _servo = servo;
        _servo->attach(servoPin);
        _servo->write(90);
    }

    void update(int pos) {
        if(pos < 0 or pos > 180)
            return;
        _servo->write(pos);
    }
};
*/
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

//Flasher led1(LED_PIN, 100, 400);
UltraSound sensorFront(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
UltraSound sensorLeft(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
UltraSound sensorRight(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);
//GestureSensor gestureSensor;
//ColorSensor colorSensor;
ReflectanceSensor reflectanceSensor;

Motor motorFrontLeft(1);
Motor motorBackLeft(2);

Motor motorFrontRight(4);
Motor motorBackRight(3);

Movement m(&motorFrontLeft, &motorBackLeft, &motorFrontRight, &motorBackRight);

Servo servo;
//ServoControl servoControl(&servo, 10);


bool systemOn = true;
unsigned long previousMillis = 0;
int turnMode = TURN_MODE;

void setup() {
    if (DEBUG)
        Serial.begin(9600);
    //gestureSensor.init();
    //colorSensor.init();

    //servo.attach(10);
    motorShield.begin(); //On lance la communication avec le shield
}



void loop() {
    /* DO NOT USE ANY DELAY IN THE MAIN LOOP */

    float distanceFront = sensorFront.getDistance();
    float distanceLeft = sensorLeft.getDistance();
    float distanceRight = sensorRight.getDistance();
    //uint8_t order = gestureSensor.getOrder();

    //int reflectanceSensorVal = analogRead(REFLECTANCE_PIN);
    //float voltage = (reflectanceSensorVal/1024.0) * 5.0;
    //Serial.println(voltage);

    //reflectanceSensor.getReflectance();


    int valPoten = analogRead(POTENTIOMETER_PIN);
    int newSpeed = map(valPoten, 0, 1023, 0, 255);

    // Stop the whole system if TIME_OUT is reached
    unsigned long currentMillis = millis();
    if (newSpeed < 10 || (TIME_OUT != -1 && (currentMillis - previousMillis >= TIME_OUT))) {
        systemOn = false;
        m.stop();
    } else {
        systemOn = true;
    }

    //Serial.println(distanceFront);
    if (!systemOn)
        return;

    if(newSpeed <= 255) {
        m.setMaxSpeed(newSpeed);
        m.setTurnSpeed(newSpeed);
    }

    if (distanceFront > 1) {
        m.setTurnMode(turnMode);
        m.setDistanceFront(distanceFront);
        m.setDistanceLeft(distanceLeft);
        m.setDistanceRight(distanceRight);
        int direction = m.update();
        //Serial.println(direction);
        //servoControl.update(direction);
        //servo.write(direction);
    }
    //led1.update(distanceFront);
    //colorSensor.getColor();
}


