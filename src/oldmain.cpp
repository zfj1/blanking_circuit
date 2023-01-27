// #include "Arduino.h"

// enum Request {
//   REQ_RESET,
  
//   REQ_GET_UV_BLANK,
//   REQ_GET_GREEN_BLANK,
//   REQ_GET_BLUE_BLANK,

//   REQ_SET_UV_BLANK,
//   REQ_SET_GREEN_BLANK,
//   REQ_SET_BLUE_BLANK,
  
//   REQ_GET_UV_PWM,
//   REQ_GET_GREEN_PWM,
//   REQ_GET_BLUE_PWM,

//   REQ_SET_UV_PWM,
//   REQ_SET_GREEN_PWM,
//   REQ_SET_BLUE_PWM,

// };

// // #define DEBUG
// #ifdef DEBUG
// #define PRINT(x) Serial.print(x)
// #define PRINTLN(x) Serial.println(x)
// #else
// #define PRINT(x) 
// #define PRINTLN(x)
// #endif

// #define UV_PWM_PIN 32
// #define GN_PWM_PIN 31
// #define BL_PWM_PIN 30

// #define UV_BLANK_PIN 28
// #define GN_BLANK_PIN 28
// #define BL_BLANK_PIN 28 // builtin LED

// #define PWM_FREQ 93750

// #define WritePWM(pin, level) analogWrite(pin, 256-level) 

// bool blue_blanking, green_blanking, uv_blanking;
// uint16_t blue_pwm, green_pwm, uv_pwm;

// void reset() {
//   blue_pwm = 256;
//   green_pwm = 256;
//   uv_pwm = 256;

//   WritePWM(UV_PWM_PIN, uv_pwm);
//   WritePWM(GN_PWM_PIN, green_pwm);
//   WritePWM(BL_PWM_PIN, blue_pwm);

//   blue_blanking = false;
//   green_blanking = false;
//   uv_blanking = false;

//   digitalWriteFast(UV_BLANK_PIN, uv_blanking);
//   digitalWriteFast(GN_BLANK_PIN, green_blanking);
//   digitalWriteFast(BL_BLANK_PIN, blue_blanking);

// }


// void setup() {
//   pinMode(UV_PWM_PIN, OUTPUT);
//   pinMode(GN_PWM_PIN, OUTPUT);
//   pinMode(BL_PWM_PIN, OUTPUT);
  
//   pinMode(UV_BLANK_PIN, OUTPUT);
//   pinMode(GN_BLANK_PIN, OUTPUT);
//   pinMode(BL_BLANK_PIN, OUTPUT);

//   //all the pins should be on the same timer, so we technically only need to change one pin's freq
//   analogWriteFrequency(UV_PWM_PIN, PWM_FREQ);
//   analogWriteFrequency(GN_PWM_PIN, PWM_FREQ);
//   analogWriteFrequency(BL_PWM_PIN, PWM_FREQ);
  

//   reset();  

//   Serial.begin(9600);
// }

// void loop() { 
//   // digitalToggleFast(LED_BUILTIN);
//   // delay(250);
//   if (Serial.available()) {
//     switch (auto x = Serial.read()) {
//       //Blanking
//       case REQ_GET_UV_BLANK:
//         Serial.write(uv_blanking);
//         break;
//       case REQ_GET_GREEN_BLANK:
//         Serial.write(green_blanking);
//         break;
//       case REQ_GET_BLUE_BLANK:
//         Serial.write(blue_blanking);
//         break;
//       case REQ_SET_BLUE_BLANK:
//         blue_blanking = Serial.read();
//         if (blue_blanking) blue_pwm = 0;
//         else blue_pwm = 256;
//         WritePWM(BL_PWM_PIN, blue_pwm);
//         digitalWriteFast(BL_BLANK_PIN, blue_blanking);
//         break;
//       case REQ_SET_GREEN_BLANK:
//         green_blanking = Serial.read();
//         if (green_blanking) green_pwm = 0;
//         else green_pwm = 256;
//         WritePWM(GN_PWM_PIN, green_pwm);
//         digitalWriteFast(GN_BLANK_PIN, green_blanking);
//         break;
//       case REQ_SET_UV_BLANK:
//         uv_blanking = Serial.read();
//         if (uv_blanking) uv_pwm = 0;
//         else uv_pwm = 256;
//         WritePWM(UV_PWM_PIN, uv_pwm);
//         digitalWriteFast(UV_BLANK_PIN, uv_blanking);
//         break;

//       //PWM
//       case REQ_GET_UV_PWM: 
//         Serial.write((uint8_t*)&uv_pwm, 2);
//           break;
//       case REQ_GET_GREEN_PWM:
//         Serial.write((uint8_t*)&green_pwm, 2);
//           break;
//       case REQ_GET_BLUE_PWM:
//         Serial.write((uint8_t*)&blue_pwm, 2);
//           break;
//       case REQ_SET_UV_PWM:
//         uv_pwm = Serial.read();
//         uv_blanking = false;
//         WritePWM(UV_PWM_PIN, uv_pwm);
//         digitalWriteFast(UV_BLANK_PIN, uv_blanking);
//         break;
//       case REQ_SET_GREEN_PWM:
//         green_pwm = Serial.read();
//         green_blanking = false;
//         WritePWM(GN_PWM_PIN, green_pwm);
//         digitalWriteFast(GN_BLANK_PIN, green_blanking);
//         break;
//       case REQ_SET_BLUE_PWM:
//         blue_pwm = Serial.read();
//         blue_blanking = false;
//         WritePWM(BL_PWM_PIN, blue_pwm);
//         digitalWriteFast(BL_BLANK_PIN, blue_blanking);
//         PRINTLN(blue_pwm);
//         break;
      
//       //reset
//       case REQ_RESET:
//         reset();
//         break;

//       default:
//         Serial.print("Error: got unknown data (");
//         Serial.println(")");
//         Serial.print(x);
//         break;
//     }

//     PRINT("blue blanking: ");
//     PRINTLN(blue_blanking);
//     PRINT("green blanking: ");
//     PRINTLN(green_blanking);    
//     PRINT("uv blanking: ");
//     PRINTLN(uv_blanking);

//     PRINT("blue pwm: ");
//     PRINTLN(blue_pwm);
//     PRINT("green pwm: ");
//     PRINTLN(green_pwm);    
//     PRINT("uv pwm: ");
//     PRINTLN(uv_pwm);

//   }
//   delay(250);
// }
