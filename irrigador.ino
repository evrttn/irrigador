#include <avr/sleep.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include "prescaler.h"
RtcDS3231<TwoWire> Rtc(Wire);

#define INTERRUPT_PIN 3
#define IRRIGATION_INTERVAL 15000 //15 s
#define HOUR_ALARM_ONE 7
#define HOUR_ALARM_TWO 19

volatile bool irrigationMode = false;
 
void setup()
{
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  Rtc.Begin();
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmBoth); //enables alarms One and Two
  setWakeUpAlarms();

  DDRB |= 0x01; // same as pinMode(8, OUTPUT);
  PORTB |= 0x00; //same as digitalWrite(8, LOW);

  setClockPrescaler(CLOCK_PRESCALER_256);

  while(true){ //avoid entering loop() because it always checks serial comm.
    goToSleep();
    work();
  }
}

void setWakeUpAlarms(){
    DS3231AlarmOne alarm1(0, HOUR_ALARM_ONE, 0, 0, DS3231AlarmOneControl_HoursMinutesSecondsMatch);
    Rtc.SetAlarmOne(alarm1);

    DS3231AlarmTwo alarm2(0, HOUR_ALARM_TWO, 0, DS3231AlarmTwoControl_HoursMinutesMatch);
    Rtc.SetAlarmTwo(alarm2);
    
    Rtc.LatchAlarmsTriggeredFlags();// trigger alarms
}

void work()
{ 
  pumpWater();
  Rtc.LatchAlarmsTriggeredFlags();// allows for alarms to trigger again
  irrigationMode = false; //reset the flag
  
}

void wakeUp()
{
  irrigationMode = true;    
}

void goToSleep()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();    
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), wakeUp, LOW);
  sleep_cpu();
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));          
}

void pumpWater()
{
  PORTB |= 0x01;  
  trueDelay(IRRIGATION_INTERVAL);
  PORTB |= 0x00; 
}

void loop()
{  

}
