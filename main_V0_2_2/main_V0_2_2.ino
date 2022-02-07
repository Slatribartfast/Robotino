//#define USE_USBCON
/************************************************************
*   includes                                             *
*************************************************************/
//Neopixel stuff copied
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

//ros
#include <ros.h>
#include <ros/time.h>

//ros msges
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>


/************************************************************
*   definitions                                             *
*************************************************************/
#define BOUNDRATE 57600

/************************************************************
*   Pins                                             *
*************************************************************/
//motor hardfware drivers
#define INA11 13
#define INA21 12
#define PWMA1 11
#define LO11PIN 10
#define LO21PIN 9

#define INA12 8
#define INA22 7
#define PWMA2 6
#define LO12PIN 14
#define LO22PIN 15

#define INA13 5
#define INA23 4
#define PWMA3 2
#define LO13PIN 22
#define LO23PIN 24

//stepper camera
#define STEPPERDIRPIN A4
#define STEPPERENABLE A1
#define STEPPERSTEPPIN A3
#define STEPTIME 50 //in millis

//Neo Pixel
#define NEOPIXELPIN 44 //any pwm should work
#define NUMPIXELS 144

//IR Sensors
#define IRSENSOR0 A7
#define IRSENSOR1 A8
#define IRSENSOR2 A9
#define IRSENSOR3 A10
#define IRSENSOR4 A11
#define IRSENSOR5 A12
#define IRSENSOR6 A13
#define IRSENSOR7 A14
#define IRSENSOR8 A15

//bumper
#define BUMPERPIN A2

//batterie
#define BATTERIEPIN A0

//button
#define BUTTON1PIN 27
#define BUTTON2PIN 35
#define BUTTON3PIN 43

//end stop camea
#define ENDSTOPCAMERAPIN 23

//encoder
#define ENCODER11 3
#define ENCODER12 26
#define ENCODER21 18
#define ENCODER22 28
#define ENCODER31 19
#define ENCODER32 30

//buzzer
#define BUZZERPIN 46

//LOOP Time
#define LOOPTIME 1

/************************************************************
*   globals                                                 *
*************************************************************/
//ir
int ir_array[9];

//stepper
int stepCounterCameraStepper = 0;
int stepperPinStatus = true;
unsigned long lastActionStepper = 0;

//buzzer 
int repeatCounterBuzzer = 0;
int buzzerPinStatus = true;
int buzzerFrequency = 0;
int buzzerDuration = 0;
unsigned long lastActionBuzzer = 0;


//encoder
volatile int encoderCounterOne = 0;
volatile int encoderCounterTwo = 0;
volatile int encoderCounterThree = 0;

//neopixel
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);

//ros
ros::NodeHandle nh;

//loop
unsigned long lastLoop = 0;
/************************************************************
*   messages                                                *
*************************************************************/

//led
std_msgs::Int16MultiArray color_array;

//ir Sensors
std_msgs::Int16MultiArray ir_sensor_array;

//stepper
std_msgs::Int16 stepper_int;

//bumper
std_msgs::Int16 bumper_int;

//battery voltage
std_msgs::Int16 battery_int;

//buttons
std_msgs::Bool button1_bool;
std_msgs::Bool button2_bool;
std_msgs::Bool button3_bool;

//encoder
std_msgs::Int16 encoder1_int;
std_msgs::Int16 encoder2_int;
std_msgs::Int16 encoder3_int;

//motor error
std_msgs::Int16 motor_error_int;

//stepper end stop
std_msgs::Bool stepper_end_stop_bool;

/************************************************************
*   get/set functions                                             *
*************************************************************/
void set_motor1_wheel(std_msgs::Int16 &msg){
  
  //push in bounderies
  if (msg.data < -255){      
    msg.data = -255;
  }
  if (msg.data > 255){
    msg.data = 255;
  }
  
  if(msg.data >= 0){    
    analogWrite(PWMA1, abs(msg.data));
    digitalWrite(INA11,  HIGH);
    digitalWrite(INA21, LOW);
  }
  else{
    analogWrite(PWMA1, abs(msg.data));
    digitalWrite(INA11,  LOW);
    digitalWrite(INA21, HIGH);
  }
}

void set_motor2_wheel(std_msgs::Int16 &msg){ 
  //push in bounderies
  if (msg.data < -255){      
    msg.data = -255;
  }
  if (msg.data > 255){
    msg.data = 255;
  }
  
  if(msg.data >= 0){
    analogWrite(PWMA2, abs(msg.data));
    digitalWrite(INA12,  HIGH);
    digitalWrite(INA22, LOW);
  }
  else{
    analogWrite(PWMA2, abs(msg.data));
    digitalWrite(INA12,  LOW);
    digitalWrite(INA22, HIGH);
  }
}

void set_motor3_wheel(std_msgs::Int16 &msg){
  //push in bounderies
  if (msg.data < -255){      
    msg.data = -255;
  }
  if (msg.data > 255){
    msg.data = 255;
  }

  if(msg.data >= 0){
    analogWrite(PWMA3, abs(msg.data));
    digitalWrite(INA13,  HIGH);
    digitalWrite(INA23, LOW);
  }
  else{
    analogWrite(PWMA3, abs(msg.data));
    digitalWrite(INA13,  LOW);
    digitalWrite(INA23, HIGH); 
  }
}

void set_led_strip(std_msgs::Int16MultiArray &msg){
  if(msg.data[3] >= NUMPIXELS){
      return;
  }
  if(msg.data[3] > 0){
    pixels.setPixelColor(msg.data[3], pixels.Color(msg.data[0], msg.data[1], msg.data[2]));
    pixels.show();   // Send the updated pixel colors to the hardware.
  }
  else{
    for (int i = 0; i < NUMPIXELS; i++){
      pixels.setPixelColor(i, pixels.Color(msg.data[0], msg.data[1], msg.data[2]));
    } 
    pixels.show();   // Send the updated pixel colors to the hardware.    
  }
}

void add_stepper_camera_counter(std_msgs::Int16 &msg){
  stepCounterCameraStepper += msg.data;
}

void set_buzzer(std_msgs::Int16MultiArray &msg){
  repeatCounterBuzzer = msg.data[2];
  buzzerPinStatus = false;
  buzzerFrequency = msg.data[0];
  buzzerDuration = msg.data[1];
  lastActionBuzzer = 0;
}

void get_ir_sensors(){  
  int sensorPins[9] = {IRSENSOR0,IRSENSOR1,IRSENSOR2,IRSENSOR3,IRSENSOR4,IRSENSOR5,IRSENSOR6,IRSENSOR7,IRSENSOR8};
  for(int i = 0; i < 9;i++){
    ir_array[i] = analogRead(sensorPins[i]);
  }
  ir_sensor_array.data = ir_array;
}

void get_bumper(){
  bumper_int.data = analogRead(BUMPERPIN);
}


void get_motor_error(){
  int LOPins[6] = {LO11PIN,LO21PIN,LO12PIN,LO22PIN,LO13PIN,LO23PIN};

  for(int i = 0; i < 3; i++){
    int error = 0;
    if (digitalRead(LOPins[i*2]) == LOW) error =  1;
    if (digitalRead(LOPins[i*2 + 1]) == LOW) error +=  2;
    motor_error_int.data = error;   
  }
}

void get_batterie_voltage(){
  battery_int.data = analogRead(BATTERIEPIN);
}

void get_buttons(){  
   button1_bool.data = digitalRead(BUTTON1PIN) == HIGH;
   button2_bool.data = digitalRead(BUTTON2PIN) == HIGH;
   button3_bool.data = digitalRead(BUTTON3PIN) == HIGH;   
}

void get_endstop_camera(){
  stepper_end_stop_bool.data = digitalRead(ENDSTOPCAMERAPIN);
}

void get_encoder(){
  /*
  int encoder1 = encoderCounterOne;
  encoderCounterOne = encoderCounterOne - encoder1;

  int encoder2 = encoderCounterTwo;
  encoderCounterTwo = encoderCounterTwo - encoder2;

  int encoder3 = encoderCounterThree;
  encoderCounterThree = encoderCounterThree - encoder3;
  
  encoder1_int.data = encoder1;
  encoder2_int.data = encoder2;
  encoder3_int.data = encoder3;
 */
  encoder1_int.data = encoderCounterOne;
  encoder2_int.data = encoderCounterTwo;
  encoder3_int.data = encoderCounterThree;
 
}


/************************************************************
*   internal functions                                      *
*************************************************************/

void doStep(){
  bool dir = false;
  if (stepCounterCameraStepper == 0) return;
  if (stepCounterCameraStepper > 0){
    digitalWrite(STEPPERDIRPIN, HIGH);
    dir = true;  
  }
  else{
    digitalWrite(STEPPERDIRPIN, LOW);
    dir = false;   
  }

  if(stepperPinStatus){
    stepperPinStatus = false;
    digitalWrite(STEPPERSTEPPIN, HIGH);
    if(dir){
      stepCounterCameraStepper --;
    }
    else{
      stepCounterCameraStepper++;
    }
  }
  else{
    stepperPinStatus = true;
    digitalWrite(STEPPERSTEPPIN, LOW);
  }
}

void doBuzzing(){
   
  if (repeatCounterBuzzer == 0) return;
  if (repeatCounterBuzzer > 0){
    if(buzzerPinStatus){
      tone(BUZZERPIN, buzzerFrequency);
      buzzerPinStatus = false;
      repeatCounterBuzzer--;
    }
    else{
      noTone(BUZZERPIN);
      buzzerPinStatus = true;
    }
  }
}

void encoderInterrupt1(){
  if((PINA & B00010000) > 0){
    encoderCounterOne++;
  }
  else{
    encoderCounterOne--;
  }
}

void encoderInterrupt2(){
  if((PINA & B01000000) > 0){
    encoderCounterTwo++;
  }
  else{
    encoderCounterTwo--;
  }
}

void encoderInterrupt3(){
  if((PINC & B10000000) > 0){
    encoderCounterThree++;
  }
  else{
    encoderCounterThree--;
  }
}

/************************************************************
*   defining publishers and subscribers                     *
*************************************************************/


//publisher
//IR Sensors
ros::Publisher pub_ir_sensors("ir_sensors_top",&ir_sensor_array);
//bumper
ros::Publisher pub_bumper("bumper_sensors_top",&bumper_int);
//battery voltage
ros::Publisher pub_battery("battery_sensors_top",&battery_int);
//buttons
ros::Publisher pub_button1("button1_top",&button1_bool);
ros::Publisher pub_button2("button2_top",&button2_bool);
ros::Publisher pub_button3("button3_top",&button3_bool);
//encoder
ros::Publisher pub_encoder1("encoder1_top",&encoder1_int);
ros::Publisher pub_encoder2("encoder2_top",&encoder2_int);
ros::Publisher pub_encoder3("encoder3_top",&encoder3_int);
//motor error
ros::Publisher pub_motor_error_int("motor_error_sensors_top",&motor_error_int);
//endstop stepper
ros::Publisher pub_stepper_end_stop("stepper_end_stop_top",&stepper_end_stop_bool);

//subcriber
//led
ros::Subscriber<std_msgs::Int16MultiArray> sub_led("led_top", &set_led_strip);
//stepper
ros::Subscriber<std_msgs::Int16> sub_stepper("stepper_top", &add_stepper_camera_counter);
//motor
ros::Subscriber<std_msgs::Int16> sub_motor1("motor1_top", &set_motor1_wheel);
ros::Subscriber<std_msgs::Int16> sub_motor2("motor2_top", &set_motor2_wheel);
ros::Subscriber<std_msgs::Int16> sub_motor3("motor3_top", &set_motor3_wheel);
//buzzer
ros::Subscriber<std_msgs::Int16MultiArray> sub_buzzer("buzzer_top", &doBuzzing);




void setup() {
  //pins
  //wheels
  pinMode(INA11, OUTPUT);
  pinMode(INA21, OUTPUT);
  pinMode(PWMA1, OUTPUT);
  pinMode(LO11PIN, INPUT);
  pinMode(LO21PIN, INPUT);

  pinMode(INA12, OUTPUT);
  pinMode(INA22, OUTPUT);
  pinMode(PWMA2, OUTPUT);
  pinMode(LO12PIN, INPUT);
  pinMode(LO22PIN, INPUT);

  pinMode(INA13, OUTPUT);
  pinMode(INA23, OUTPUT);
  pinMode(PWMA3, OUTPUT);
  pinMode(LO13PIN, INPUT);
  pinMode(LO23PIN, INPUT);

 // Neo pixel
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();
  pixels.show();

  pixels.setPixelColor(7, pixels.Color(120, 120, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.

  delay(1000);

  pixels.clear();
  pixels.show();
  
  //ir Sensors
  pinMode(IRSENSOR0, INPUT);
  pinMode(IRSENSOR1, INPUT);
  pinMode(IRSENSOR2, INPUT);
  pinMode(IRSENSOR3, INPUT);
  pinMode(IRSENSOR4, INPUT);
  pinMode(IRSENSOR5, INPUT);
  pinMode(IRSENSOR6, INPUT);
  pinMode(IRSENSOR7, INPUT);
  pinMode(IRSENSOR8, INPUT);
  ir_sensor_array.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension));
  ir_sensor_array.layout.dim[0].label = "distance";
  ir_sensor_array.data_length = 9;


  //bumper
  pinMode(BUMPERPIN, INPUT);

  //batterie
  pinMode(BATTERIEPIN, INPUT);

  //button
  pinMode(BUTTON1PIN, INPUT);
  pinMode(BUTTON2PIN, INPUT);
  pinMode(BUTTON3PIN, INPUT);

  //end stop camera
  pinMode(ENDSTOPCAMERAPIN, INPUT);

  //encoder
  pinMode(ENCODER11, INPUT_PULLUP);
  pinMode(ENCODER12, INPUT_PULLUP);
  pinMode(ENCODER21, INPUT_PULLUP);
  pinMode(ENCODER22, INPUT_PULLUP);
  pinMode(ENCODER31, INPUT_PULLUP);
  pinMode(ENCODER32, INPUT_PULLUP);

  //stepper
  pinMode(STEPPERDIRPIN,INPUT);
  pinMode(STEPPERENABLE,INPUT);
  pinMode(STEPPERSTEPPIN,INPUT);

  digitalWrite(STEPPERENABLE, LOW);

  //
  pinMode(BUZZERPIN, OUTPUT);

  //encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER11), encoderInterrupt1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER21), encoderInterrupt2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER31), encoderInterrupt3, RISING);



  // Set baudrate
  nh.getHardware()->setBaud(BOUNDRATE);
  // Initialise ros node
  nh.initNode();

  //publisher
  //IR Sensors
  nh.advertise(pub_ir_sensors);
  //bumper
  nh.advertise(pub_bumper);
  
  //battery voltage
  nh.advertise(pub_battery);
  //buttons
  nh.advertise(pub_button1);
  nh.advertise(pub_button2);
  nh.advertise(pub_button3);
  
  //encoder 
  nh.advertise(pub_encoder1);
  nh.advertise(pub_encoder2);
  nh.advertise(pub_encoder3);
  
  //motor error
  nh.advertise(pub_motor_error_int);
  //endstop stepper
  
  nh.advertise(pub_stepper_end_stop);
  
  //subcriber
  //led
  nh.subscribe(sub_led); 
  nh.subscribe(sub_stepper);
  //motor
  nh.subscribe(sub_motor1);
  nh.subscribe(sub_motor2);
  nh.subscribe(sub_motor3);
  //buzzer
  nh.subscribe(sub_buzzer);
}


void loop() {
  
  get_ir_sensors();
  pub_ir_sensors.publish(&ir_sensor_array);
  get_bumper();
  pub_bumper.publish(&bumper_int);
  get_motor_error();
  pub_motor_error_int.publish(&motor_error_int);
  
  get_batterie_voltage();
  //pub_battery.publish(&battery_int);
  get_encoder();
  pub_encoder1.publish(&encoder1_int);
  pub_encoder2.publish(&encoder2_int);
  pub_encoder3.publish(&encoder3_int);

  pub_battery.publish(&battery_int); 

  get_endstop_camera();
  pub_stepper_end_stop.publish(&stepper_end_stop_bool);
  
  if (millis() < lastActionStepper) lastActionStepper = 0;
  if (millis() - lastActionStepper >= STEPTIME) {
    lastActionStepper += STEPTIME;
      doStep();
  }

  if (millis() < lastActionBuzzer) lastActionBuzzer = 0;
  if (millis() - lastActionBuzzer >= buzzerDuration) {
    lastActionBuzzer += buzzerDuration;
      doBuzzing();
  }
  
  
  nh.spinOnce();
  
  if(millis() < lastLoop){
    lastLoop = 0;
    delay(LOOPTIME);
  }
  else{
    delay(LOOPTIME - (lastLoop - millis()));
  }
  
  lastLoop = millis();
  
}
