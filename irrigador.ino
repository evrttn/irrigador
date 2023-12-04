/*
 * 
 * melhorias TODO
 * -no máximo 2 alarmes
 * -usar AlarmOne e AlarmTwo no setup
 * -translate variables to english
 * 
 * 
 */
 
#include <avr/sleep.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include "prescaler.h"
RtcDS3231<TwoWire> Rtc(Wire);

#define interruptPin 3
#define irrigationInterval 15000 //15 s

const uint8_t hora[] = {9,14,19};
const uint8_t minutos[] = {0,0,0};
const uint8_t num_alarmes = 3;
uint8_t i = 0;

volatile bool irrigationMode = false;
 
void setup()
{
  pinMode(interruptPin, INPUT_PULLUP);

  Rtc.Begin();
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmOne); //Habilta alarme One

  DDRB |= 0x01; // same as pinMode(8, OUTPUT);
  PORTB |= 0x00; //same as digitalWrite(8, LOW);

  setClockPrescaler(CLOCK_PRESCALER_256);

  while(true){ //avoid entering loop() because it always checks serial comm.
    work();
  }
}

void work()
{
    uint8_t k = i++ % num_alarmes;
    DS3231AlarmOne alarm1(0, hora[k], minutos[k], 0, DS3231AlarmOneControl_HoursMinutesSecondsMatch);
    Rtc.SetAlarmOne(alarm1);
    Rtc.LatchAlarmsTriggeredFlags();// Efetiva os alarmes
    
    goToSleep();
  
    if(irrigationMode){
        pumpWater();    
        irrigationMode = false;
    }
}

void wakeUp()
{
  irrigationMode = true;    
}

void goToSleep()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();    
  attachInterrupt(digitalPinToInterrupt(interruptPin), wakeUp, LOW);
  sleep_cpu();
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(interruptPin));          
}

void pumpWater()
{
  PORTB |= 0x01;  
  trueDelay(irrigationInterval);
  PORTB |= 0x00; 
}

void loop()
{  

}
