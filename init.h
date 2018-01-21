const bool DEBUG = true;
const bool DEBUG_SENSOR = false;

const int TIME_OUT = 50000;

const byte FRONT_TRIGGER_PIN = 3; // Broche TRIGGER
const byte FRONT_ECHO_PIN = 2;    // Broche ECHO

const byte LEFT_TRIGGER_PIN = 5; // Broche TRIGGER
const byte LEFT_ECHO_PIN = 4;    // Broche ECHO

const byte RIGHT_TRIGGER_PIN = 7; // Broche TRIGGER
const byte RIGHT_ECHO_PIN = 6;    // Broche ECHO

const int LED_PIN = 8;
const int REFLECTANCE_PIN = A0;

const int POTENTIOMETER_PIN = A1;

const int MAX_SPEED = 100;
const int LOW_SPEED = 50;
const int TURN_SPEED = 100;
const int CORRECT_SPEED = 100;

const int OBSTACLE_DISTANCE_SLOWDOWN = 100;
const int OBSTACLE_DISTANCE_STOP = 50;
const int OBSTACLE_DISTANCE_TURN = 40;
const int OBSTACLE_DISTANCE_SIDE_CORRECT = 20;

const int TIME_TURN_180 = 1000;
const int TIME_TURN_90 = 500;
const int TIME_TURN_10 = 100;


const int TURN_MODE = 2; // 0: in place, 1 smooth, 2: constant (turn approx. 90 degrees each time)
/*
 * In place: left wheels forward, right wheels backwards
 * Smooth: left wheel hald the speed of the right wheels
 * */

/* Constantes pour le timeout */
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s

/* Vitesse du son dans l'air en mm/us */
const float SOUND_SPEED = 340.0 / 1000;
