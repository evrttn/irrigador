#include <avr/sleep.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include "prescaler.h"
#include "UART.h"

#define BLUETOOTH_PIN 2
#define INTERRUPT_PIN 3
#define IRRIGATION_INTERVAL 7500 //15 s
#define HOUR_ALARM_ONE 7
#define HOUR_ALARM_TWO 21
#define MAX_BUFF_SIZE  38

RtcDS3231<TwoWire> Rtc(Wire);
volatile bool irrigationMode = false;
volatile bool bleMode = false;

volatile char UARTBUFF[MAX_BUFF_SIZE]={' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
                   ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
                   ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
                   ' ',' ',' ',' ',' ',' ',' ',' '};

volatile unsigned int addr = 0;
volatile boolean msgCompleted = false;

ISR(USART0_RX_vect){

  UARTBUFF[addr] = UDR0; 

  if(addr >= MAX_BUFF_SIZE)
    addr = 0;
  else
    addr++;

  if(UARTBUFF[addr] == '!')
    msgCompleted = true;
}

void handleBLEMessage(){
  while(!msgCompleted);

  //trata aqui
}
 
void setup()
{
  UART0_config(); //BLE communication 
  
  //pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  DDRD &= ~((1 << DDD3) | (1 << PD2));
  PORTD |= ((1 << PD3) | (1 << PD2));

  Rtc.Begin();
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmBoth); //enables alarms One and Two
  setWakeUpAlarms();

  DDRD |= (1 << DDD4); // same as pinMode(4, OUTPUT);
  PORTD &= ~(1 << PD4); //same as digitalWrite(4, LOW);

  setClockPrescaler(CLOCK_PRESCALER_256);

  //sei();?

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
  
  if(irrigationMode)
    pumpWater();
  else if(bleMode)
    handleBLEMessage();
  Rtc.LatchAlarmsTriggeredFlags();// allows for alarms to trigger again
  irrigationMode = false; //reset the flag
  bleMode = false;
}


void wakeUpBLE()
{
  bleMode = !bleMode;    
}

void wakeUp()
{
  irrigationMode = true;    
}

void goToSleep()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();    
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), wakeUp, LOW); // FALLING faz ligar e desligar ininterruptamente
  attachInterrupt(digitalPinToInterrupt(BLUETOOTH_PIN), wakeUpBLE, LOW); // FALLING faz ligar e desligar ininterruptamente  
  sleep_cpu();
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
  detachInterrupt(digitalPinToInterrupt(BLUETOOTH_PIN));
}

void pumpWater()
{
  PORTD |= (1 << PD4);  
  trueDelay(IRRIGATION_INTERVAL);
  PORTD &= ~(1 << PD4); 
}

void loop()
{  

}
