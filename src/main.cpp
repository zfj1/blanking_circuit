#include "Arduino.h"
#include <avr/EEPROM.h>
#include <string.h>
// #include <inttypes.h>

// #include <avr/io.h>
// #include <avr/interrupt.h>

enum Request {
    REQ_RESET = 0,
    REQ_GET_BLANKING = 1,
    REQ_SET_BLANKING = 2,

    REQ_COMPENSATE_FLICKER = 3,
    
    REQ_GET_LINE_ON_CYCLES = 4,
    REQ_SET_LINE_ON_CYCLES = 5,
    REQ_GET_LINE_OFF_CYCLES = 6,
    REQ_SET_LINE_OFF_CYCLES = 7,

    REQ_MEASURE_LINE_PERIOD = 8,
    
    
    // GET/SET frame/line clock, fan, LED on
    // GET LED voltage drop, current


};


// #define RESPOND(x) Serial.write(x)
// #define RESPOND32(x) Serial.write((char*)&x, 4)
#define RESPOND(x) Serial.println(x)
#define RESPOND32(x) Serial.println(x)
// #define RESPOND32(x) Serial.write(lowByte(x)) ; Serial.write(lowByte(x >> 8)) ; Serial.write(lowByte(x >> 16)) ; Serial.write(lowByte(x >> 24)) ; 
#define READ(x) do { x = Serial.read(); } while (0)
#define READ32(x) Serial.readBytes((char*)&x, 4)

#define FANPIN 5
#define FRAMECLOCK 6
#define LINECLOCK 7
#define BLANKEN 8
#define BLUEON 10
#define GREENON 11
#define UVON 12


#define BLUEA A1
#define BLUEB A5
#define BLUEC A9

#define GREENA A0
#define GREENB A3
#define GREENC A8

#define UVA A14
#define UVB A2
#define UVC A7


#define DEBUG
#ifdef DEBUG
#define PRINT(x) Serial.print(x)
#define PRINTLN(x) Serial.println(x)
#else
#define PRINT(x) 
#define PRINTLN(x)
#endif

bool is_blanking = 0;
volatile bool do_compensation = 0;
volatile bool do_measurement = 0;
volatile uint32_t line_on_cycles = 0;
volatile uint32_t line_off_cycles = 0;
volatile uint32_t linePeriod[2] = {};


void measureLinePeriod() {
    cli();
    static uint32_t n_calls, lastCnt;
    auto thisCnt = ARM_DWT_CYCCNT;
    if ((n_calls % 60) == 1) lastCnt = thisCnt;
    linePeriod[n_calls % 2] = thisCnt - lastCnt;
    lastCnt = thisCnt;
    n_calls += 1;
    // linePeriod[n_calls % 2 + 1] = n_calls;
    if (((n_calls % 60) == 0) && do_compensation)  {
        detachInterrupt(digitalPinToInterrupt(LINECLOCK));
    }
    sei();
}

volatile uint32_t nFrameResets = 0;
void goHighOnDelay() {
    cli();
    auto startTime = ARM_DWT_CYCCNT;

    // auto endTime = startTime + line_on_cycles;
    // while (ARM_DWT_CYCCNT < endTime) { asm volatile("nop"); };
    // digitalWrite(BLANKEN, false);

    // endTime += line_off_cycles ;
    // while (ARM_DWT_CYCCNT < endTime) { asm volatile("nop"); };

    // digitalWrite(BLANKEN, true);


    // simpler... blanks immediately, ignores line on time
    // will still appear as slight flicker, but will reliably reset blanking before next frame?
    digitalWrite(BLANKEN, false);

    auto endTime = startTime + line_off_cycles;
    while (ARM_DWT_CYCCNT < endTime) { asm volatile("nop"); }

    digitalWrite(BLANKEN, true);

    nFrameResets += 1;
    if (do_measurement) {
        attachInterrupt(digitalPinToInterrupt(LINECLOCK), measureLinePeriod, CHANGE);
    }
    sei();
}

void setup() {

    //count clock cycles
    // ARM_DEMCR |= ARM_DEMCR_TRCENA;
    // ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    //pinmode necessary for analog inputs?
    pinMode(BLUEA, INPUT);
    pinMode(BLUEB, INPUT);
    pinMode(BLUEC, INPUT);    
    
    pinMode(GREENA, INPUT);
    pinMode(GREENB, INPUT);
    pinMode(GREENC, INPUT);
    
    pinMode(UVA, INPUT);
    pinMode(UVB, INPUT);
    pinMode(UVC, INPUT);
    
    pinMode(FANPIN, OUTPUT);
    pinMode(FRAMECLOCK, INPUT);
    pinMode(LINECLOCK, INPUT);
    pinMode(BLANKEN, OUTPUT);
    pinMode(BLUEON, INPUT);
    pinMode(GREENON, INPUT);
    pinMode(UVON, INPUT);

    analogReadResolution(8);
    
    analogWrite(FANPIN, eeprom_read_byte((uint8_t*)FANPIN));

    is_blanking = eeprom_read_byte((uint8_t*)BLANKEN);
    digitalWrite(BLANKEN, is_blanking);

    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    do_compensation = eeprom_read_byte((uint8_t*)1);
    do_measurement = eeprom_read_byte((uint8_t*)2);
    
    line_off_cycles = eeprom_read_dword((uint32_t*)128);
    line_off_cycles = eeprom_read_dword((uint32_t*)160);

    if (do_compensation) attachInterrupt(digitalPinToInterrupt(FRAMECLOCK), goHighOnDelay, FALLING);
    if (do_measurement) attachInterrupt(digitalPinToInterrupt(LINECLOCK), measureLinePeriod, CHANGE);

    Serial.begin(9600);



}

const char* delim = ",";
void loop() {
    if (Serial.available()) {

        char comm[128];
        Serial.readString().toCharArray(comm, 128);

        int x, arg = 0;
        x = atoi(strtok(comm, delim));
        if (x != 0) {
            arg = atoi(strtok(NULL, delim));
        }
        PRINT("Got value ");
        PRINT(x);
        PRINT(" with arg ");
        PRINTLN(arg);


        switch (x) {
            case REQ_RESET:
                PRINTLN("Got reset request. Ignored");
                break;
            case REQ_GET_BLANKING:
                RESPOND(is_blanking);
                break;
            case REQ_SET_BLANKING:
                // READ(is_blanking);
                is_blanking = arg;
                eeprom_write_byte((uint8_t*)BLANKEN, is_blanking);
                digitalWrite(BLANKEN, is_blanking);
                break;
            case REQ_COMPENSATE_FLICKER:
                // READ(do_compensation);
                PRINTLN("Got compensation request.");
                do_compensation = arg;
                PRINT("Compensation is now: ");
                PRINTLN(do_compensation);
                eeprom_write_byte((uint8_t*)1, do_compensation);

                if (do_compensation) {
                    is_blanking = true;
                    eeprom_write_byte((uint8_t*)BLANKEN, is_blanking);
                    digitalWrite(BLANKEN, is_blanking);

                    // detachInterrupt(digitalPinToInterrupt(LINECLOCK)); // not allowed to do both
                    attachInterrupt(digitalPinToInterrupt(FRAMECLOCK), goHighOnDelay, FALLING);
                } else {
                    detachInterrupt(digitalPinToInterrupt(FRAMECLOCK));
                }
                break;
            case REQ_GET_LINE_ON_CYCLES:
                RESPOND32(line_on_cycles);
                break;
            case REQ_SET_LINE_ON_CYCLES:
                // READ32(line_on_cycles);
                line_on_cycles = arg;
                eeprom_write_dword((uint32_t*)128, line_on_cycles);
                break;
            case REQ_GET_LINE_OFF_CYCLES:
                RESPOND32(line_off_cycles);            
                break;
            case REQ_SET_LINE_OFF_CYCLES:
                // READ32(line_off_cycles);
                line_off_cycles = arg;
                eeprom_write_dword((uint32_t*)160, line_off_cycles);
                break;
            case REQ_MEASURE_LINE_PERIOD:
                do_measurement =  arg;
                eeprom_write_byte((uint8_t*)2, do_measurement);
    
                if (do_measurement) {


                    // detachInterrupt(digitalPinToInterrupt(FRAMECLOCK)); // not allowed to do both
                    attachInterrupt(digitalPinToInterrupt(LINECLOCK), measureLinePeriod, CHANGE);
                }
                else detachInterrupt(digitalPinToInterrupt(LINECLOCK));
                break;

        }
    }

    // delay(250);
    PRINT(">UV_ON:");
    PRINTLN(digitalRead(UVON));
    // PRINT(">GREEN_ON:");
    // PRINTLN(digitalRead(GREENON));
    PRINT(">BLANK_EN:");
    PRINTLN(digitalRead(BLANKEN));

    PRINT(">FC:");
    PRINTLN(digitalRead(FRAMECLOCK));
    PRINT(">LC:");
    PRINTLN(digitalRead(FRAMECLOCK));    

    // PRINT(">GREENA:");
    // PRINTLN(analogRead(GREENA));
    // PRINT(">GREENB:");
    // PRINTLN(analogRead(GREENB));
    // PRINT(">GREENC:");
    // PRINTLN(analogRead(GREENC));

    
    PRINT(">COMP:");
    PRINTLN(do_compensation);

    
    PRINT(">MEAS:");
    PRINTLN(do_measurement);

    PRINT(">NFRAMES:");
    PRINTLN(nFrameResets);

    PRINT(">ONCLOCKS:");
    PRINTLN(linePeriod[1]);
    
    PRINT(">OFFCLOCKS:");
    PRINTLN(linePeriod[0]);

    delayMicroseconds(800);    
}

// clockCyclesPerMicrosecond()
// microsecondsToClockCycles()
// clockCyclesToMicroseconds()

// plan:

// new plan

// if doCompensation
//  accumulate the on and off time for each frame
//  track the last 