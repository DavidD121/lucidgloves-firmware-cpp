#include <Arduino.h>
#include <mutex>
#include <condition_variable>
#include <queue>

int lockTime = 0;
int lockLoops = 1;
int lockTimeTotal = 0;
int lockTimeLast = 0;

class ordered_lock {
    std::queue<std::condition_variable *> cvar;
    std::mutex                            cvar_lock;
    bool                                  locked;
public:
    ordered_lock() : locked(false) {};
    void lock() {
        lockTime = micros() - lockTimeLast;
        lockTimeLast = micros();
        lockTimeTotal += lockTime;
        lockLoops++;
        std::unique_lock<std::mutex> acquire(cvar_lock);
        if (locked) {
            std::condition_variable signal;
            cvar.emplace(&signal);
            signal.wait(acquire);
        } else {
            locked = true;
        }
    }
    void unlock() {
        std::unique_lock<std::mutex> acquire(cvar_lock);
        if (cvar.empty()) {
            locked = false;
        } else {
            cvar.front()->notify_one();
            cvar.pop();
        }
    }
};

//Comm
#define COMM_SERIAL 0   
#define COMM_BTSERIAL 1 

//Encode
#define ENCODE_LEGACY 0
#define ENCODE_ALPHA  1

//Multiplexer
#define MUX(p) (p + 100)
#define UNMUX(p) (p % 100)
#define ISMUX(p) (p >= 100)

//intermediate filtering
#define INTERFILTER_NONE 0
#define INTERFILTER_LIMITS 1
#define INTERFILTER_ALL 2

#define LOOP_TIME 1 //How much time between data sends (ms), set to 0 for a good time :)
#define CALIBRATION_LOOPS -1//How many loops should be calibrated. Set to -1 to always be calibrated.

//Encoding
#define ENCODING ENCODE_ALPHA

//Bluetooth advaned settings
#define BT_ECHO false//Should the bluetooth data be echoed over serial for debugging

#define INTERMEDIATE_CALIBRATION true //should intermediate values (used in some forms of mixing such as MIXING_SINCOS be calibrated
//Intermediate values: If not autocalibrating, you may need to adjust these for accuracy
#define INTER_MAX 1500 //only used if intermediate calibration is false
#define INTER_MIN 1200 //only used if intermediate calibration is false


//Finger indeces (not used for legacy)
#define PINKY_IND 4
#define RING_IND 3
#define MIDDLE_IND 2
#define INDEX_IND 1
#define THUMB_IND 0

#define ANALOG_MAX 4095 

#define MULTIPLEXER_DELAY   5 //How many microseconds should be delayed between multiplexer reads

// You must install RunningMedian library to use this feature
// https://www.arduino.cc/reference/en/libraries/runningmedian/
#define ENABLE_MEDIAN_FILTER false //use the median of the previous values, helps reduce noise
#define MEDIAN_SAMPLES 10

//intermediate filtering. Options are INTERFILTER_NONE, INTERFILTER_LIMITS (filter is only used for limit calib), INTERFILTER_ALL (filter all the way)
#define INTERFILTER_MODE INTERFILTER_NONE
#define INTERFILTER_SAMPLES 100


//This is the configuration file, main structure in _main.ino
//CONFIGURATION SETTINGS:
#define COMMUNICATION COMM_SERIAL //Which communication protocol to use. Options are: COMM_SERIAL (usb), COMM_BTSERIAL (bluetooth)
//serial over USB
#define SERIAL_BAUD_RATE 115200

//serial over Bluetooth
#define BTSERIAL_DEVICE_NAME "lucidgloves-left"

//ANALOG INPUT CONFIG
#define USING_SPLAY false //whether or not your glove tracks splay. - tracks the side to side "wag" of fingers. Requires 5 more inputs.
#define USING_MULTIPLEXER true //Whether or not you are using a multiplexer for inputs
#define USING_STRAIN_GAUGE false //Whether or not you are using strain gauge to detect force applied by each finger. Requires 5 more inputs
#define USING_CURRENT_SENSOR false //Whether or not you are using a current sensor
#define FLIP_FLEXION  false  //Flip values from potentiometers (for fingers!) if they are backwards
#define FLIP_SPLAY true //Flip values for splay
#define FLIP_STRAIN true //Flip values for force readings

//Gesture enables, make false to use button override
#define TRIGGER_GESTURE true
#define GRAB_GESTURE    true
#define PINCH_GESTURE   true

//BUTTON INVERT
//If a button registers as pressed when not and vice versa (eg. using normally-closed switches),
//you can invert their behaviour here by setting their line to true.
//If unsure, set to false
#define INVERT_A false
#define INVERT_B false
#define INVERT_JOY false
#define INVERT_MENU false
#define INVERT_CALIB false
//These only apply with gesture button override:
#define INVERT_TRIGGER false
#define INVERT_GRAB false
#define INVERT_PINCH false


//joystick configuration
#define JOYSTICK_BLANK true //make true if not using the joystick
#define JOY_FLIP_X false
#define JOY_FLIP_Y false
#define JOYSTICK_DEADZONE 10 //deadzone in the joystick to prevent drift (in percent)

#define NO_THUMB false //If for some reason you don't want to track the thumb

#define USING_CALIB_PIN true //When PIN_CALIB is shorted (or it's button pushed) it will reset calibration if this is on.
#define USING_FORCE_FEEDBACK false //Force feedback haptics allow you to feel the solid objects you hold
#define FLIP_FORCE_FEEDBACK true
#define SERVO_SCALING false //dynamic scaling of servo motors

#define DISABLE_BUTTONS true // disable all buttons except for calibration

//(This configuration is for ESP32-S2 so make sure to change if you're on another board)

//These 10 pins are for flexion
#define PIN_THUMB           0
#define PIN_THUMB_SECOND    1
#define PIN_INDEX           2
#define PIN_INDEX_SECOND    3
#define PIN_MIDDLE          4
#define PIN_MIDDLE_SECOND   5
#define PIN_RING            6
#define PIN_RING_SECOND     7
#define PIN_PINKY           8 
#define PIN_PINKY_SECOND    9 

#define PIN_JOY_X     26
#define PIN_JOY_Y     26
#define PIN_JOY_BTN   26
#define PIN_A_BTN     40
#define PIN_B_BTN     41
#define PIN_TRIG_BTN  12 //unused if gesture set
#define PIN_GRAB_BTN  13 //unused if gesture set
#define PIN_PNCH_BTN  23 //unused if gesture set
#define PIN_CALIB     42 //button for recalibration (You can set this to GPIO0 to use the BOOT button, but only when using Bluetooth.)
//#define DEBUG_LED 2
#define PIN_PINKY_MOTOR     18  //used for force feedback
#define PIN_RING_MOTOR      5   //^
#define PIN_MIDDLE_MOTOR    17  //^
#define PIN_INDEX_MOTOR     16  //^
#define PIN_THUMB_MOTOR     4   //^
#define PIN_MENU_BTN        26
#define PIN_CURRENT_SENSOR_PINKY  33
#define PIN_CURRENT_SENSOR_RING   34
#define PIN_CURRENT_SENSOR_MIDDLE 35
#define PIN_CURRENT_SENSOR_INDEX  36
#define PIN_CURRENT_SENSOR_THUMB  37

#define PIN_DOUT 32

//Splay pins. Only used for splay tracking gloves. Use MUX(pin) if you are using a multiplexer for it.

#define PIN_PINKY_SPLAY  26
#define PIN_RING_SPLAY   26
#define PIN_MIDDLE_SPLAY 26
#define PIN_INDEX_SPLAY  26
#define PIN_THUMB_SPLAY  26