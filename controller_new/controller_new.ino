#include <HX711.h>
#include <Servo.h>
#include <CommandParser.h>

typedef CommandParser<10, 6, 8, 64, 128> Controller;
Controller controller; // parser
HX711 scale; // import scale
Servo clip;  // 建立一個 servo 物件，最多可建立 12個 servo

//設定升降步進角位
#define DT_PIN 5
#define SCK_PIN 9
#define LOAD_DIR_PIN 2
#define LOAD_MOTOR_PIN 3
#define TOSS_DIR_PIN 10
#define TOSS_MOTOR_PIN 11
#define LED_GROUND 12
#define LED_PIN 13
#define STEP_PER_ROUND 1600
#define HOME_BACKLASH_DISTANCE 200
#define HOME_DELAY_TIME_MICRO_SEC 1e6 / ( 0.7 * STEP_PER_ROUND * 2.0 )

#define GRAB_RELEASE 100
#define GRAB_TIGHT   180

#define SCALE_FACTOR -1102.019635632186 //比例參數，從校正程式中取得

// log level
int logLevel = 0;

//phototransistor
int limit = 0;

//stop
bool stopFlag = false;

// float conversoin buffer
char float1[32] = {'\0'};
char float2[32] = {'\0'};

// IO buffer
char line[128];
char response[Controller::MAX_RESPONSE_SIZE];

void log(Controller::Argument *args, char *response)
{
    logLevel = args[0].asInt64;
    // Log
    if (logLevel > 5) {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "log level = %d", logLevel);
    } else {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "OK");
    }
}

void home(Controller::Argument *args, char *response)
{
    // 歸位--------------------------------------
    digitalWrite(LED_PIN, LOW); 
    digitalWrite(TOSS_DIR_PIN, LOW);
    for (int i = 0; i < 250; i++)
    {
        // if (stopFlag) break;
        digitalWrite(TOSS_MOTOR_PIN, HIGH);
        delayMicroseconds(HOME_DELAY_TIME_MICRO_SEC);
        digitalWrite(TOSS_MOTOR_PIN, LOW);
        delayMicroseconds(HOME_DELAY_TIME_MICRO_SEC);
    }
    for (int i = 0; i < 6400; i++)
    {
        // if (stopFlag) break;
        digitalWrite(TOSS_MOTOR_PIN, HIGH);
        delayMicroseconds(HOME_DELAY_TIME_MICRO_SEC );
        digitalWrite(TOSS_MOTOR_PIN, LOW);
        delayMicroseconds(HOME_DELAY_TIME_MICRO_SEC );
        limit = analogRead(A0);
        if (limit >700) break;
    }
    // 微調整回來角度-----------------
    for (int i = 0; i < HOME_BACKLASH_DISTANCE; i++)
    {   
        // if (stopFlag) break;
        digitalWrite(TOSS_MOTOR_PIN, HIGH);
        delayMicroseconds(HOME_DELAY_TIME_MICRO_SEC);
        digitalWrite(TOSS_MOTOR_PIN, LOW);
        delayMicroseconds(HOME_DELAY_TIME_MICRO_SEC);
    }  
    digitalWrite(TOSS_DIR_PIN, HIGH);
    for (int i = 0; i < STEP_PER_ROUND / 2; i++)
    {   
        // if (stopFlag) break;
        digitalWrite(TOSS_MOTOR_PIN, HIGH);
        delayMicroseconds(HOME_DELAY_TIME_MICRO_SEC);
        digitalWrite(TOSS_MOTOR_PIN, LOW);
        delayMicroseconds(HOME_DELAY_TIME_MICRO_SEC);
    }
    // if (stopFlag) stopFlag = false;
    // Log
    if (logLevel > 5) {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "home");
    } else {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "OK");
    }
}

void move(Controller::Argument *args, char *response)
{
    double rps = args[0].asDouble;
    int angle = round(args[1].asDouble / 360 * STEP_PER_ROUND);
    unsigned int dt = round( 1e6 / ( rps * STEP_PER_ROUND  *  2.0) );
    if (angle > 0) {
        digitalWrite(TOSS_DIR_PIN, HIGH);
    } else {
        digitalWrite(TOSS_DIR_PIN, LOW);
    }
    for (int i = 0; i < abs(angle); i++)
    {
        // if (stopFlag) break;
        digitalWrite(TOSS_MOTOR_PIN, HIGH);
        delayMicroseconds(dt);
        digitalWrite(TOSS_MOTOR_PIN, LOW);
        delayMicroseconds(dt);
    }
    // if (stopFlag) stopFlag = false;
    // Log
    if (logLevel > 5) {
        dtostrf(rps, 4, 2, float1);
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "move rps = %s, angle = %d, dt = %u", float1, angle, dt);
    } else {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "OK");
    }
}

void lift(Controller::Argument *args, char *response)
{
    double rps = args[0].asDouble;
    unsigned int up = round(args[1].asDouble);
    unsigned int dt = round( 1e6 / (rps * STEP_PER_ROUND  *  2.0) );

    digitalWrite(LOAD_DIR_PIN, LOW);
    for (unsigned int i = 0; i < up; i++)
    {
        // if (stopFlag) break;
        digitalWrite(LOAD_MOTOR_PIN, HIGH);
        delayMicroseconds(dt);
        digitalWrite(LOAD_MOTOR_PIN, LOW);
        delayMicroseconds(dt);
    }
    // if (stopFlag) stopFlag = false;
    // Log
    if (logLevel > 5) {
        dtostrf(rps, 4, 2, float1);
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "lift rps = %s, up = %u, dt = %u", float1, up, dt);
    } else {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "OK");
    }
}

void drop(Controller::Argument *args, char *response)
{
    double rps = args[0].asDouble;
    unsigned int down = round(args[1].asDouble);
    unsigned int dt = round( 1e6 / ( args[0].asDouble * STEP_PER_ROUND  * 2.0) );

    digitalWrite(LOAD_DIR_PIN, HIGH);
    for (unsigned int i = 0; i < down; i++)
    {
        // if (stopFlag) break;
        digitalWrite(LOAD_MOTOR_PIN, HIGH);
        delayMicroseconds(dt);
        digitalWrite(LOAD_MOTOR_PIN, LOW);
        delayMicroseconds(dt);
    }
    // if (stopFlag) stopFlag = false;
    // Log
    if (logLevel > 5) {
        dtostrf(rps, 4, 2, float1);
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "drop rps = %s, down = %u, dt = %u", float1, down, dt);
    } else {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "OK");
    }
}

void grab(Controller::Argument *args, char *response)
{
    digitalWrite(LED_PIN, HIGH);
    clip.write(GRAB_TIGHT);
 
    // Log
    if (logLevel > 5) {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "grab");
    } else {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "OK");
    }
}

void release(Controller::Argument *args, char *response)
{
    digitalWrite(LED_PIN, LOW); 
    clip.write(GRAB_RELEASE);

    // Log
    if (logLevel > 5) {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "release");
    } else {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "OK");
    }
}

void toss(Controller::Argument *args, char *response)
{  
    double start_rps = args[0].asDouble;
    unsigned int start = round(args[1].asDouble / 360 * STEP_PER_ROUND);
    unsigned int start_dt = round( 1e6 / ( start_rps * STEP_PER_ROUND  * 2.0) );

    double end_rps = args[2].asDouble;
    unsigned int end = round(args[3].asDouble / 360 * STEP_PER_ROUND);
    unsigned int end_dt = round( 1e6 / ( end_rps * STEP_PER_ROUND * 2.0) );
    
    unsigned int target = round( args[4].asDouble / 360 * STEP_PER_ROUND );
    unsigned int pause = round(args[5].asDouble);

    digitalWrite(TOSS_DIR_PIN, LOW);
    for (unsigned int i = 0; i < start; i++)
    {
        // if (stopFlag) break;
        digitalWrite(TOSS_MOTOR_PIN, HIGH);
        delayMicroseconds(start_dt);
        digitalWrite(TOSS_MOTOR_PIN, LOW);
        delayMicroseconds(start_dt);
    }
    delay(pause);
    digitalWrite(TOSS_DIR_PIN, HIGH);
    for (unsigned int i = 0; i < end; i++)
    {
        // if (stopFlag) break;
        digitalWrite(TOSS_MOTOR_PIN, HIGH);
        delayMicroseconds(end_dt);
        digitalWrite(TOSS_MOTOR_PIN, LOW);
        delayMicroseconds(end_dt);
        if (i >= target) {
          clip.write(GRAB_RELEASE); // 鬆夾
          digitalWrite(LED_PIN, HIGH);
        }
    }
    // if (stopFlag) stopFlag = false;
    // Log
    if (logLevel > 5) {
        dtostrf(start_rps, 4, 2, float1);
        dtostrf(end_rps, 4, 2, float2);
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "toss start_rps = %s, start_angle = %u, start_dt = %u, end_rps = %s, end_angle = %u, end_dt = %u, target = %u, pause = %u",
                                                            float1, start, start_dt, float2, end, end_dt, target, pause);
    } else {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "OK");
    }
}

void weight(Controller::Argument *args, char *response)
{
    scale.power_up();
    double a = scale.get_units(1);
    Serial.println(a);
    scale.power_down();
    // Log
    if (logLevel > 5) {
        dtostrf(a, 4, 2, float1);
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "weight = %s", float1);
    } else {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "OK");
    }
}

void stop(Controller::Argument *args, char *response)
{
    stopFlag = true;
    // Log
    if (logLevel > 5) {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "stop");
    } else {
        snprintf(response, Controller::MAX_RESPONSE_SIZE, "OK");
    }
}

void setup() {
    controller.registerCommand("lift",    "dd",    &lift); // speed, step
    controller.registerCommand("drop",    "dd",    &drop); 
    controller.registerCommand("move",    "dd",    &move);
    controller.registerCommand("toss",    "dddddd", &toss); // start_speed angle, toss_speed angle, target angle
    controller.registerCommand("log",     "i",     &log);
    controller.registerCommand("home",    "",      &home);
    controller.registerCommand("stop",    "",      &stop);
    controller.registerCommand("grab",    "",      &grab);
    controller.registerCommand("release", "",      &release);
    controller.registerCommand("weight",  "",      &weight);

    Serial.begin(9600);
    //夾子
    clip.attach(6);  // 將夾子馬達連接到 6
  
    //步進
    pinMode(LOAD_DIR_PIN,   OUTPUT);
    pinMode(LOAD_MOTOR_PIN, OUTPUT);
    pinMode(TOSS_DIR_PIN,   OUTPUT);
    pinMode(TOSS_MOTOR_PIN, OUTPUT);
    pinMode(A0,             INPUT);
    //LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    // pinMode(LED_GROUND, OUTPUT);
    // digitalWrite(LED_GROUND, LOW);
    //  loadcell set-up---------------
    // Serial.println("Initializing the scale");
    scale.begin(DT_PIN, SCK_PIN);
    // Serial.println("Before setting up the scale:"); 
    // Serial.println(scale.get_units(1), 5);  //未設定比例參數前的數值
    scale.set_scale(SCALE_FACTOR); // 設定比例參數
    delay(1000);
    scale.tare();               // 歸零
    // Serial.println("After setting up the scale:"); 
    // Serial.println(scale.get_units(1), 5);  //設定比例參數後的數值
    // Serial.println("Readings:");  //在這個訊息之前都不要放東西在電子稱上
}

void loop() {
    if (Serial.available()) {
        size_t lineLength = Serial.readBytesUntil('\n', line, 127);
        line[lineLength] = '\0';
        controller.processCommand(line, response);
        Serial.println(response);
    }
} 