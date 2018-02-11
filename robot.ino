#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Servo.h> 

#include "init.h"


Adafruit_MotorShield motorShield = Adafruit_MotorShield();
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);


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
class Color
{
    //int _ledState;                 // ledState used to set the LED

    // Constructor - creates a Flasher
    // and initializes the member variables and state
    public:
    Color()
    {
    }

    void init() {
        pinMode(PIN_LED_R, OUTPUT);
        pinMode(PIN_LED_G, OUTPUT);
        pinMode(PIN_LED_B, OUTPUT);
        //displayColor(0, 255, 0);
        this->displayColor(COLOR_BLUE);
        delay(100);
        this->displayColor(COLOR_RED);
        delay(100);
        this->displayColor(COLOR_GREEN);
        delay(100);
        this->displayColor(COLOR_VIOLET);
        delay(100);
        this->displayColor(COLOR_YELLOW);
        delay(100);
        this->displayColor(COLOR_BROWN);
        delay(100);
    }

    //void displayColor(byte r, byte g, byte b) {
    void displayColor(int c[]) {
        // Assigne l'état des broches
        // Version anode commune
        analogWrite(PIN_LED_R, 255 - c[0]);
        analogWrite(PIN_LED_G, 255 - c[1]);
        analogWrite(PIN_LED_B, 255 - c[2]);
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
class Compass
{
    int _motorPin;
    Adafruit_DCMotor *_motor;

    // Constructor - creates a Sound
    // and initializes the member variables and state
    public:
    Compass()
    {
    }

    void init() {
        if(!mag.begin())
        {
            if(DEBUG_COMPASS)
                Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
            while(1);
        }
    }

    float getHeading() {
        sensors_event_t event; 
        mag.getEvent(&event);

        float Pi = 3.14159;

        float tabHeadings[5];

//         for(int i = 0; i < 5; i++) {
            // Calculate the angle of the vector y,x

            // calibration    
            //                  x       y      z
            //  Mag Minimums: -47.82  -70.64  -69.29
            //  Mag Maximums: 58.27  36.36  44.18      
            float avgX = (-47.82 + 58.27) / 2;
            float avgY = (-70.64 + 36.36) / 2;
            
            float deltaX = event.magnetic.x - avgX;
            float deltaY = event.magnetic.y - avgY;
            
            //float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
            float heading = (atan2(deltaY ,deltaX) * 180) / Pi;

            // Normalize to 0-360
            if (heading < 0)
            {
                heading = 360 + heading;
            }

            //tabHeadings[i] = heading;

            if(DEBUG_COMPASS) {
//                 Serial.print("  Heading: ");
//                 Serial.println(heading);
            }

            // TODO 
            // Il y a des fois des valeurs absurdes
            // Il faut donc prendre 5 mesures et prendre  celle qui est la plus différentes des autres ,et renvoyer une moyenne
            // ex:  5 6 5 300 5 => on enelève le 300
//         }
        
        // TODO avg tabHeadings
        
        return heading;
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
    int _turnDegTime;
    int _direction;
    float _distanceFront;
    float _distanceLeft;
    float _distanceRight;
    Motor *_motorFrontLeft;
    Motor *_motorBackLeft;
    Motor *_motorFrontRight;
    Motor *_motorBackRight;
    UltraSound *_sensorFront;
    Color *_color;
    Compass *_compass;

    public:
    Movement(Compass *mCompass, Motor *mFrontLeft, Motor *mBackLeft, Motor *mFrontRight, Motor *mBackRight, UltraSound *mSensorFront, Color *mColor)
    {
        this->_compass = mCompass;

        this->_motorFrontLeft = mFrontLeft;
        this->_motorBackLeft = mBackLeft;
        this->_motorFrontRight = mFrontRight;
        this->_motorBackRight = mBackRight;
        this->_sensorFront = mSensorFront;
        this->_color = mColor;

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

    //void setTurnSpeed(int speed, int time)
    void setTurnSpeed(int speed)
    {
        this->_turnSpeed = speed;
        //this->setTurnTime(speed, time);
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

    //void setTurnTime(int speed, int time) {
    void setTurnTime(int speed) {
        this->_turn180Time = map(speed, 0, 255, 500, TIME_TURN_180);
        this->_turn90Time = map(speed, 0, 255, 200, TIME_TURN_90);
        //this->_turnDegTime = time;
        this->_turnDegTime = this->_turn90Time;
    }

    int update() {
        // According to the distanceFront, slow down, and turn (if distanceLeft and distanceRight are OK)
        if (this->_distanceFront > OBSTACLE_DISTANCE_STOP) {
            _color->displayColor(COLOR_GREEN);
            this->_direction = 90;
            if (this->_distanceFront > OBSTACLE_DISTANCE_SLOWDOWN) {
                this->setSpeed(this->_maxSpeed);
                this->forward();
                this->_direction = 90;
                if (DEBUG) {
                    //Serial.println("FORWARD FULL SPEED");
                    Serial.print("FORWARD FULL SPEED     ");
                    Serial.println(_compass->getHeading());
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
                _color->displayColor(COLOR_YELLOW);
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
                _color->displayColor(COLOR_BLUE);
                if (DEBUG) {
                    Serial.println("TURN LEFT");
                }
                this->turnLeft();
                this->_direction = 0;
            } else {
                // Do 180 turn
                _color->displayColor(COLOR_RED);
                this->turnDeg(0, 180); // TODO variable for this value
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
            //this->turnDeg(0, this->_turnDegTime);
            this->turnDeg(0, TURN_DEGREE); // TODO variable for this value
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
            //this->turnDeg(1, this->_turnDegTime);
            this->turnDeg(1, TURN_DEGREE); // variable for this value
        }
    }

    void turnDeg(int way, int deg) { // way = 0: left, 1: right, deg is given in millisecond (time to turn)
        float startHeading = _compass->getHeading();
        float currentHeading = startHeading;
        //float deltaDeg = abs(currentHeading - startHeading);
        float deltaDeg = 0.0;
        float target = 0.0;
        if(way) {
            // RIGHT
            target = currentHeading + deg;
            if(target > 360)
                target -= 360;
        }
        else {
            // LEFT
            target = currentHeading - deg;
            if(target < 0)
                target += 360;
        }

        //while(deltaDeg <= deg)
        //while(currentHeading >= target)
        //while((target - currentHeading) >= 0)
        while(1)
        {
            // Check if th front is clear, if so, stop the turn, and go
            float distanceFront = _sensorFront->getDistance();
            if (distanceFront > OBSTACLE_DISTANCE_STOP && deg != 180) {  // If 180, we force the halfspin
                break;
            }

            currentHeading = _compass->getHeading();
            if (DEBUG) {
                if(way == 0)
                    Serial.print("TURNING LEFT: ");
                else
                    Serial.print("TURNING RIGHT: ");
                Serial.print(deg);
                Serial.print("                  TARGET: ");
                Serial.print(target);
                Serial.print("    CURRENT: ");
                Serial.println(currentHeading);
            }

            if(way) {
                // RIGHT
//                 deltaDeg = startHeading + currentHeading;
                if(currentHeading > 360)
                    currentHeading -= 360;
                if(currentHeading > target)
                    break;
                this->_motorFrontLeft->forward(this->_turnSpeed);
                this->_motorBackLeft->forward(this->_turnSpeed);
                this->_motorFrontRight->backward(this->_turnSpeed);
                this->_motorBackRight->backward(this->_turnSpeed);
            }
            else {
                // LEFT
//                 deltaDeg = currentHeading - startHeading;
                if(currentHeading < 0)
                    currentHeading += 360;
                if(currentHeading < target)
                    break;
                this->_motorFrontLeft->backward(this->_turnSpeed);
                this->_motorBackLeft->backward(this->_turnSpeed);
                this->_motorFrontRight->forward(this->_turnSpeed);
                this->_motorBackRight->forward(this->_turnSpeed);
            }

            /*if (DEBUG) {
                Serial.print("                              ");
                Serial.println(deltaDeg);
            }*/
        }
    }
    /*
    void turn180() {

        float startHeading = _compass->getHeading();
        float currentHeading = startHeading;
        float deltaDeg = abs(currentHeading - startHeading);

        while(deltaDeg <= 180)
        {
            if (DEBUG) {
                Serial.println("TURNING 180");
            }
            currentHeading = _compass->getHeading();

            currentHeading = _compass->getHeading();
            deltaDeg = abs(currentHeading - startHeading);

            this->_motorFrontLeft->backward(this->_turnSpeed);
            this->_motorBackLeft->backward(this->_turnSpeed);
            this->_motorFrontRight->forward(this->_turnSpeed);
            this->_motorBackRight->forward(this->_turnSpeed);
        }
    }
    */

    void correctLeft() {
        // If an obstacle get closer on the LEFT, then slow down the RIGHT motors to correct the direction
        if (this->_distanceLeft < OBSTACLE_DISTANCE_SIDE_CORRECT) {
            _color->displayColor(COLOR_VIOLET);
            if (DEBUG) {
                Serial.print("   correcting LEFT  ");
            }
            this->_motorFrontRight->backward(CORRECT_SPEED);
            this->_motorBackRight->backward(CORRECT_SPEED);
            //this->turnDeg(0, 10); // variable for this value
        }
    }

    void correctRight() {
        // If an obstacle get closer on the RIGHT, then slow down the LEFT motors to corect the direction
        if (this->_distanceRight < OBSTACLE_DISTANCE_SIDE_CORRECT) {
            _color->displayColor(COLOR_BROWN);
             if (DEBUG) {
                Serial.print("   correcting RIGHT  ");
             }
             this->_motorFrontLeft->backward(CORRECT_SPEED);
             this->_motorBackLeft->backward(CORRECT_SPEED);
            //this->turnDeg(1, 10); // variable for this value
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

Compass compass;

Color color;

Movement m(&compass, &motorFrontLeft, &motorBackLeft, &motorFrontRight, &motorBackRight, &sensorFront, &color);

//Servo servo;




//ServoControl servoControl(&servo, 10);


bool systemOn = true;
unsigned long previousMillis = 0;
int turnMode = TURN_MODE;

void setup() {
    if (DEBUG)
        Serial.begin(9600);
    if (DEBUG_POTENT)
        Serial.begin(9600);
    if(DEBUG_COMPASS)
        Serial.begin(9600);
    //gestureSensor.init();
    //colorSensor.init();

    //servo.attach(10);
    motorShield.begin(); //On lance la communication avec le shield

    compass.init();

    color.init();
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
    if(DEBUG_POTENT) {
        Serial.print("Speed: ");
        Serial.println(valPoten);
    }
    int newSpeed = map(valPoten, 0, 1023, 0, 255);

    //color.displayColor(newSpeed, 0, newSpeed);

    /*int valTurnPoten = analogRead(POTENTIOMETER_TURN_PIN);
    int newTurnTime = map(valTurnPoten, 0, 1023, 0, 1000);
    if(DEBUG_POTENT) {
        Serial.print("Turn: ");
        Serial.println(newTurnTime);
    }*/

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
        //m.setTurnSpeed(newSpeed, newTurnTime);
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


