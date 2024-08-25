#include <avr/sleep.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include "prescaler.h"
#include "UART.h"

#define BLUETOOTH_PIN 2
#define INTERRUPT_PIN 3
#define HOUR_ALARM_ONE 7
#define HOUR_ALARM_TWO 21
#define MAX_BUFF_SIZE  38

RtcDS3231<TwoWire> Rtc(Wire);

volatile bool irrigationMode = false;
volatile bool bleMode = false;
volatile bool myloop = true;

uint16_t irrigInterval = 2000; //7.5s

volatile char UARTBUFF[MAX_BUFF_SIZE]={' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
                   ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
                   ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
                   ' ',' ',' ',' ',' ',' ',' ',' '};

volatile uint8_t addr = 0;

ISR(USART_RX_vect){
  char c = UDR0;
  
  if(addr >= MAX_BUFF_SIZE)
    addr = 0;

  if(c == '!'){
      myloop = false;
      UARTBUFF[addr] = '\0';
      addr = 0;
  }else
    UARTBUFF[addr++] = c;
}


void tratarHorario(char * comm){
  char *hora, *minuto;
  strcpy(hora, strtok(comm, ":"));
  strcpy(minuto, strtok(NULL, ":"));
  
  uint8_t h = atoi(hora);
  uint8_t m = atoi(minuto);

  RtcDateTime dateTime = Rtc.GetDateTime();
  dateTime.SetHour(h);
  dateTime.SetMinute(m);
  Rtc.SetDateTime(dateTime);
}

void tratarAlarmeOne(char * comm){
  char *hora, *minuto;
  
  hora = strtok(comm, ":");
  minuto = strtok(NULL, ":");
  
  uint8_t h = atoi(hora);
  uint8_t m = atoi(minuto);

  DS3231AlarmOne alarm1(0, h, m, 0, DS3231AlarmOneControl_HoursMinutesSecondsMatch);
  Rtc.SetAlarmOne(alarm1);
  Rtc.LatchAlarmsTriggeredFlags();// trigger alarms
}

void tratarAlarmeTwo(char * comm){
  char *hora, *minuto;
  
  hora = strtok(comm, ":");
  minuto = strtok(NULL, ":");
  
  uint8_t h = atoi(hora);
  uint8_t m = atoi(minuto);

  DS3231AlarmTwo alarm2(0, h, m, DS3231AlarmTwoControl_HoursMinutesMatch);
  Rtc.SetAlarmTwo(alarm2);
    
  Rtc.LatchAlarmsTriggeredFlags();
}

void tratarData(char * comm){
  char *dia, *mes, *ano;
  dia = strtok(comm, "/");
  mes = strtok(NULL, "/");
  ano = strtok(NULL, "/");

  uint8_t d = atoi(dia);
  uint8_t m = atoi(mes);
  uint16_t a = atoi(ano);

  RtcDateTime dateTime = Rtc.GetDateTime();
  dateTime.SetDay(d);
  dateTime.SetMonth(m);
  dateTime.SetYear(a);
  Rtc.SetDateTime(dateTime);
}

void tratarTempo(char * comm){
  uint16_t t = atoi(comm);
  irrigInterval = t;  
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Day(),
            dt.Month(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    UART0_enviaString(datestring);
}

void menu(char* comm){
  char c = comm[0];
  switch(c){
    case 'H':
      tratarHorario(comm+1);
      break;
    case 'A':
      tratarAlarmeOne(comm+1);
      break;
    case 'B':
      tratarAlarmeTwo(comm+1);
      break;
    case 'D':
      tratarData(comm+1);
      break;
    case 'T':
      tratarTempo(comm+1);
      break;
    case 'P':
      RtcDateTime now = Rtc.GetDateTime();
      printDateTime(now);
      break;
    default:
      break;
  }
}

void handleBLEMessage() {
  if(myloop)
    return;

  bleMode = false;
  
  UART0_enviaString("handling ble");
  char *p[5];
  int i = 0;
  
  // Returns first token 
    char *token = strtok(UARTBUFF, "#");
       
    // Keep printing tokens while one of the
    // delimiters present in str[].
    while (token != NULL)
    {
      p[i] = token;
        token = strtok(NULL, "#");
        i++;
    }
    
    int j = 0;
    char * comm;
    
    while(j < i)
    {
      comm = p[j];
      menu(comm);
      j++;
    }     
  PORTB &= ~(1 << PB4);
  UART0_enviaString("fim handling ble");
   
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

  PORTB &= ~(1 << PB4);

  setClockPrescaler(CLOCK_PRESCALER_256);

  //sei();

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
  //sei(); do not work
  if(irrigationMode)
    pumpWater();
  
  while(bleMode)
    handleBLEMessage();
    
  Rtc.LatchAlarmsTriggeredFlags();// allows for alarms to trigger again
  irrigationMode = false; //reset the flag
  myloop = true;
}


void wakeUpBLE()
{
  bleMode = true;
  PORTB |= (1 << PB4);
  //UART0_enviaString("hello, Everton"); not connected yet  
}

void wakeUp()
{
  irrigationMode = true;    
}

void goToSleep()
{
  cli();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();    
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), wakeUp, FALLING); //
  attachInterrupt(digitalPinToInterrupt(BLUETOOTH_PIN), wakeUpBLE, FALLING); //  
  sei();
  sleep_cpu();
  sleep_disable();
  sei();
  //detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
  //detachInterrupt(digitalPinToInterrupt(BLUETOOTH_PIN));
  //sei(); do not work
}

void pumpWater()
{
  PORTD |= (1 << PD4);  
  trueDelay(irrigInterval);
  PORTD &= ~(1 << PD4); 
}

void loop()
{  

}
