#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <Servo.h>

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
#define DISPLAY_FLIP
#define SERVO_PIN   9
#define WHEEL_PIN   A0
#define BTN_PIN     A1
#define VM_PIN      A6

//calibrate
//#define VM_CALIBRATE
#define VM_MEASURED_V 3.623
#define VM_MEASURED_N 262

//servo
#define SERVO_MIN_VAL 544
#define SERVO_MAX_VAL 2400

//sbus
#define SBUS_FRAME_HEADER 0x0F
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04

//ppm
#define PPM_CHANNELS 8  //set the number of chanels
#define PPM_DEFAULT_VAL 1500  //set the default ch value
#define PPM_FRAME_LEN 22500  //set the PPM frame length in microseconds (1ms = 1000us)
#define PPM_PULSE_LEN 300  //set the pulse length
#define PPM_INVERSE 1  //set polarity: 1 is positive, 0 is negative
#define PPM_PIN 9     //set PPM signal pin

//obj
SSD1306AsciiWire oled;
Servo servo;

//vars
uint8_t channel_num = 2; //current channel for copy wheel val [0..3]
uint16_t wheel, wheel_old, servo_last, vm, vm_old;
uint16_t ppm[PPM_CHANNELS];
uint32_t sbus_time;
float wheel_filter, vm_filter;
bool mode_center = false;
bool ppm_vs_pwm = false; //ppm instead pwm

//------------------------------------------------------------------------------
void setup() {
  //sbus out
  Serial.begin(100000, SERIAL_8E2);
  //pins
  pinMode(BTN_PIN, INPUT_PULLUP); //button
  //servo
  servo.attach(SERVO_PIN, SERVO_MIN_VAL, SERVO_MAX_VAL);
  //display
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont(X11fixed7x14B);
  #ifdef DISPLAY_FLIP
  oled.displayRemap(true);
  #endif
  oled.clear();
  oled.set2X();
  oled.print("RC tester");
  delay(500);
}
//------------------------------------------------------------------------------
void loop() {
  if(!mode_center) wheel = wheel_adc();
  vm = vm_adc();
  
  //get button
  if(!digitalRead(BTN_PIN)) { //servo center or change channel ppm/sbus
    delay(150);
    uint32_t tim = millis();
    while( (!digitalRead(BTN_PIN)) && (millis() - tim < 500) ) delay(100);
    if(millis() - tim > 400) { //long press
      if(wheel < SERVO_MIN_VAL + 4) { //set ppm/pwm
        oled.clear();
        oled.setFont(X11fixed7x14B);
        oled.set2X();
        if(!ppm_vs_pwm) {
          ppm_vs_pwm = true;
          servo.detach();
          pinMode(PPM_PIN, OUTPUT); //ppm
          digitalWrite(PPM_PIN, !PPM_INVERSE);
          for(int i=0; i<PPM_CHANNELS; i++) ppm[i] = PPM_DEFAULT_VAL;
          oled.print("PPM");
        }
        else {
          ppm_vs_pwm = false;
          servo.attach(SERVO_PIN, SERVO_MIN_VAL, SERVO_MAX_VAL);
          oled.print("PWM / SBUS");          
        }
        delay(500);
        display_now();
      }
      else { //set channel
        for(int i=0; i<PPM_CHANNELS; i++) ppm[i] = PPM_DEFAULT_VAL; //reset ppm values
        channel_num++;
        if(channel_num > 3) channel_num = 0;
        oled.clear();
        oled.setFont(X11fixed7x14B);
        oled.set2X();
        oled.print("CH ");
        oled.print(channel_num + 1);
        delay(500);
        display_now(); //restore display
      }
    }
    else { //short press
      if(!mode_center) {
         mode_center = true;
         wheel_old = wheel;
         wheel = 1500;
         servo_last = wheel;
         servo.writeMicroseconds(wheel);
         display_now();
      }
      else {
        mode_center = false;
        wheel = wheel_adc();
        wheel_old = 0;
      }
      delay(150);
    }
  }

  //check center mode, wheel in deadband
  if( (mode_center) && ( (wheel_old + 20 < wheel_adc()) || (wheel_old - 20 > wheel_adc()) ) ) {
    mode_center = false;
    wheel = wheel_adc();
    wheel_old = 0;
  }
  
  //display if need
  if( (wheel != wheel_old || vm != vm_old) && !mode_center ) {
    display_now();
    wheel_old = wheel;
    vm_old = vm;
  }

  if(!ppm_vs_pwm) { //not ppm
    //servo
    if(wheel != servo_last) {
      servo.writeMicroseconds(wheel);
      servo_last = wheel;
    }
  
    //sbus
    if(millis() > sbus_time + 14) {
      sbus_gen();
      sbus_time = millis();
    }
  }//not ppm
  else { //ppm
    ppm_gen();
  }
}

void display_now() {
  oled.clear();
  oled.setFont(lcdnums12x16);
  oled.set2X();
  if(wheel < 1000) oled.print('0');
  oled.print(wheel);
  oled.setFont(cp437font8x8);
  oled.set1X();
  #ifdef VM_CALIBRATE
    oled.println(vm);
  #else
    oled.print(vm/10);
    oled.print(".");
    oled.print(vm%10);
    oled.println("V");
  #endif
}

uint16_t wheel_adc() {
  uint16_t adc = 0;
  for (uint8_t i=0; i<8; i++) {
    adc += analogRead(WHEEL_PIN);
    delay(1);
  }
  adc = adc / 8;
  float k;
  if (abs(adc - wheel_filter) > 50) k = 0.8; else k = 0.1;
  wheel_filter += (adc - wheel_filter) * k;
  
  return constrain( map( wheel_filter, 0, 1024, SERVO_MAX_VAL + 4, SERVO_MIN_VAL - 4), SERVO_MIN_VAL, SERVO_MAX_VAL );
}

uint16_t vm_adc() {
  uint16_t adc = 0;
  for (uint8_t i=0; i<8; i++) {
    adc += analogRead(VM_PIN);
    delay(1);
  }
  adc = adc / 8;
  float k;
  if (abs(adc - vm_filter) > 50) k = 0.5; else k = 0.1;
  vm_filter += (adc - vm_filter) * k;

  #ifdef VM_CALIBRATE
    return (uint16_t)vm_filter; //for calibrate
  #else
    return vm_filter * 10 * VM_MEASURED_V / VM_MEASURED_N;
  #endif
}

void sbus_gen() {
  uint16_t output[16] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
  
  output[channel_num] = (uint16_t)(wheel - 1000) * 0.85;
  
  uint8_t stateByte = 0x00;
  /*if (isSignalLoss) {
      stateByte |= SBUS_STATE_SIGNALLOSS;
  }
  if (isFailsafe) {
      stateByte |= SBUS_STATE_FAILSAFE;
  }*/
  
  uint8_t   packet[25];
  packet[0] = SBUS_FRAME_HEADER; //Header 

  packet[1] = (uint8_t) (output[0] & 0x07FF);
  packet[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
  packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
  packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
  packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
  packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
  packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
  packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
  packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
  packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
  packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
  packet[12] = (uint8_t) ((output[8] & 0x07FF));
  packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
  packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
  packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
  packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
  packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
  packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
  packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
  packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
  packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
  packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);

  packet[23] = stateByte; //Flags byte
  packet[24] = SBUS_FRAME_FOOTER; //Footer
  Serial.write(packet, 16);
}

void ppm_gen() {
  static unsigned long lastFrLen;
  static unsigned long lastServo;
  static unsigned long lastPulse;
  static boolean PPM_run;
  static boolean pulse;
  static boolean pulseStart = true;
  static byte counter;
  static byte part = true;

  ppm[channel_num] = wheel;
  
  if(micros() - lastFrLen >= PPM_FRAME_LEN){  //start PPM signal after PPM_FRAME_LEN has passed
    lastFrLen = micros();
    PPM_run = true;
  }

  if(counter >= PPM_CHANNELS){
    PPM_run = false;
    counter = 0;
    pulse = true;  //put out the last pulse
  }

  if(PPM_run){
    if (part){  //put out the pulse
      pulse = true;
      part = false;
      lastServo = micros();
    }
    else{  //wait till servo signal time (values from the ppm array) has passed
      if(micros() - lastServo >= ppm[counter]){
        counter++;  //do the next channel
        part = true;
      }
    }
  }

  if(pulse){
    if(pulseStart == true){  //start the pulse
      digitalWrite(PPM_PIN, PPM_INVERSE);
      pulseStart = false;
      lastPulse = micros();
    }
    else{  //will wait till PPM_PULSE_LEN has passed and finish the pulse
      if(micros() - lastPulse >= PPM_PULSE_LEN){
        digitalWrite(PPM_PIN, !PPM_INVERSE);
        pulse = false;
        pulseStart = true;
      }
    }
  }
}
