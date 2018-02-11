const bool DEBUG = true;
const bool DEBUG_POTENT = false;
const bool DEBUG_SENSOR = true;
const bool DEBUG_COMPASS = true;

const int TIME_OUT = 50000;

const byte RIGHT_TRIGGER_PIN = 2; // Broche TRIGGER
const byte RIGHT_ECHO_PIN = 3;    // Broche ECHO

const byte FRONT_TRIGGER_PIN = 4; // Broche TRIGGER
const byte FRONT_ECHO_PIN = 5;    // Broche ECHO

const byte LEFT_TRIGGER_PIN = 6; // Broche TRIGGER
const byte LEFT_ECHO_PIN = 7;    // Broche ECHO

const int LED_PIN = 8;
const int REFLECTANCE_PIN = A0;

const int POTENTIOMETER_PIN = A1;
const int POTENTIOMETER_TURN_PIN = A2;

const int MAX_SPEED = 100;
const int LOW_SPEED = 50;
const int TURN_SPEED = 100;
const int CORRECT_SPEED = 200;

// const int OBSTACLE_DISTANCE_SLOWDOWN = 100;
// const int OBSTACLE_DISTANCE_STOP = 50;
// const int OBSTACLE_DISTANCE_TURN = 40;
const int OBSTACLE_DISTANCE_SLOWDOWN = 70;
const int OBSTACLE_DISTANCE_STOP = 50;
const int OBSTACLE_DISTANCE_TURN = 40;
const int OBSTACLE_DISTANCE_SIDE_CORRECT = 20;

const int TIME_TURN_180 = 1000;
const int TIME_TURN_90 = 500;
const int TIME_TURN_10 = 100;

const int TURN_DEGREE = 30;

const int TURN_MODE = 2; // 0: in place, 1 smooth, 2: constant (turn approx. 90 degrees each time)
/*
 * In place: left wheels forward, right wheels backwards
 * Smooth: left wheel hald the speed of the right wheels
 * */

/* Constantes pour le timeout */
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s

/* Vitesse du son dans l'air en mm/us */
const float SOUND_SPEED = 340.0 / 1000;


/* Couleurs */
/* Couleurs (format RGB) */
const int COLOR_BLACK[3]  = {0, 0, 0};
const int COLOR_RED[3]    = {255, 0, 0};
const int COLOR_GREEN[3]  = {0, 255, 0};
const int COLOR_BLUE[3]   = {0, 0, 255};
const int COLOR_YELLOW[3] = {255, 255, 0};
const int COLOR_BROWN[3]  = {160, 90, 44};
const int COLOR_VIOLET[3] = {128, 0, 128};


/* Broches */
const int PIN_LED_R = 12;
const int PIN_LED_G = 11;
const int PIN_LED_B = 10;



