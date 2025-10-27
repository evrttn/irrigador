#include <avr/sleep.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include <EEPROM.h>
//#include "prescaler.h"

#define BLUETOOTH_PIN 2
#define INTERRUPT_PIN 3
#define MAX_BUFF_SIZE 38

volatile char UARTBUFF[MAX_BUFF_SIZE]={' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
                   ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
                   ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
                   ' ',' ',' ',' ',' ',' ',' ',' '};

volatile uint8_t addr = 0;
volatile bool msgComplete = false;
volatile bool bleMode = false;
volatile bool irrigMode = false;

RtcDS3231<TwoWire> Rtc(Wire);

uint16_t irrigInterval;

ISR(USART_RX_vect){
  char c = UDR0;
  
  if(addr >= MAX_BUFF_SIZE)
    addr = 0;
    
  if(c == '!'){
      msgComplete = true;
      UARTBUFF[addr] = '\0';
      addr = 0;
  }else
    UARTBUFF[addr++] = c;    
}

void wakeUp()
{
  irrigMode = true;
}

void wakeUpBLE()
{
  bleMode = !bleMode;
  if(bleMode)
    PORTB |= (1 << PB4);
  else
    PORTB &= ~(1 << PB4);
}

void setup() {
  // put your setup code here, to run once:
  UART0_config();
  //sei(); desnecessario
  DDRD &= ~((1 << DDD3) | (1 << PD2));
  DDRD |= (1 << DDD4);
  
  PORTD |= ((1 << PD3) | (1 << PD2));
  PORTD &= ~(1 << PD4);
  PORTB &= ~(1 << PB4);

  Rtc.Begin();
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmBoth); //enables alarms One and Two
  
  uint8_t t = EEPROM.read(0);
  irrigInterval = t * 1000;
  
  setWakeUpAlarms();  
  
  attachInterrupt(digitalPinToInterrupt(BLUETOOTH_PIN), wakeUpBLE, FALLING); //  
}

void setWakeUpAlarms()
{
  setWakeUpAlarmOne();
  setWakeUpAlarmTwo();
}

void setWakeUpAlarmOne()
{
  uint8_t hOne = EEPROM.read(2);
  uint8_t mOne = EEPROM.read(3);
  
    DS3231AlarmOne alarm1(0, hOne, mOne, 0, DS3231AlarmOneControl_HoursMinutesSecondsMatch);
    Rtc.SetAlarmOne(alarm1);    
}

void setWakeUpAlarmTwo()
{
  uint8_t hTwo = EEPROM.read(4);
  uint8_t mTwo = EEPROM.read(5);

  DS3231AlarmTwo alarm2(0, hTwo, mTwo, DS3231AlarmTwoControl_HoursMinutesMatch);
  Rtc.SetAlarmTwo(alarm2);
}

void goToSleep()
{
  Rtc.LatchAlarmsTriggeredFlags();// trigger alarms
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();    
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), wakeUp, FALLING); //
  sleep_cpu();
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
}

void loop() {
   if(bleMode)
    handleBLEMessage();
   else{  
    goToSleep();   
    pumpWater();
   }
}

void handleBLEMessage() {
  
  if(!msgComplete)
    return;

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
    msgComplete = false;
}

void menu(char* comm){
  char c = comm[0];
  switch(c){
    case 'H':
      tratarHorario(comm+1);
      break;
    case 'A':
      tratarAlarmeOne(comm+1);
      setWakeUpAlarmOne();
      break;
    case 'B':
      tratarAlarmeTwo(comm+1);
      setWakeUpAlarmTwo();
      break;
    case 'D':
      tratarData(comm+1);
      break;
    case 'T':
      tratarTempo(comm+1);
      break;
    case 'P':      
      printDateTime();
      break;
    default:
      break;
  }
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime()
{
    uint8_t t = EEPROM.read(0);
    
    RtcDateTime dt = Rtc.GetDateTime();
    DS3231AlarmOne aOne = Rtc.GetAlarmOne();
    DS3231AlarmTwo aTwo = Rtc.GetAlarmTwo();
    
    char datestring[43];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u A %02u:%02u B %02u:%02u T %d"),
            dt.Day(),
            dt.Month(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second(),
            aOne.Hour(),
            aOne.Minute(),
            aTwo.Hour(),
            aTwo.Minute(),
            t);
    
  uint8_t i=0;
  while (datestring[i] != '\0')
  {
    UART0_enviaCaractere(datestring[i++]);
  }    
}

void tratarHorario(char * comm){
  char *hora, *minuto;

  hora = strtok(comm, ":");
  minuto = strtok(NULL, ":");
   
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

  EEPROM.write(2, h);
  EEPROM.write(3, m);
  
}

void tratarAlarmeTwo(char * comm){
  char *hora, *minuto;
  
  hora = strtok(comm, ":");
  minuto = strtok(NULL, ":");
  
  uint8_t h = atoi(hora);
  uint8_t m = atoi(minuto);
  
  EEPROM.write(4, h);
  EEPROM.write(5, m);
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
  uint8_t t = atoi(comm);
  irrigInterval = t*1000;

  EEPROM.write(0, t);
}

void pumpWater()
{
  if(irrigMode){
    PORTD |= (1 << PD4);  
    delay(irrigInterval);
    PORTD &= ~(1 << PD4);
    irrigMode = false;
  } 
}

void UART0_config()
{
  //Baud Rate de 9600bps para um cristal de 16MHz (Datasheet)
  UBRR0 = 103;
  
  //Habilita a interrupcao de recepcao e os pinos TX e RX
  UCSR0B =  (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0) ;
  
  //Configura a UART com 8 bits de dados
  UCSR0C =  (1<<UCSZ01) | (1<<UCSZ00);
}

void UART0_enviaCaractere(unsigned char ch)
{
  UDR0 = ch;

  //Aguarda o buffer ser desocupado
  while (! (UCSR0A & (1<<UDRE0)) );
}

void UART0_enviaString(char *s)
{
  unsigned int i=0;
  while (s[i] != '\0')
  {
    UART0_enviaCaractere(s[i++]);
  }
}
