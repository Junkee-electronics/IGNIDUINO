#include "U8glib.h"
#include "EEPROM.h"

//parts: 1X ARDUINO NANO
//       1X 1.3" OLED W/ SH1106 DRIVER
//       4X BUTTON
//       1X POTENTIOMETER
//       1X HALL SENSOR
//       1X HV SWITCHING CIRCUIT TO COIL, OR LOW VOLTAGE IGN. COIL
//       1X MOSFET FOR DRIVING INJECTOR

#define hall 2
#define igni 4
#define fuel 5
#define advmi 8
#define advpl 9
#define ignpl 10
#define ignmi 11
#define tps A0
#define batt A1
//______________________________________________________________________ NOTES
//gotta set timer2 prescaler to 1024, where OCR2A iof 255 coresponds to ~ 32ms.
//for injecotr there should be only ~1.6ms at idle, dependent on voltage!
//for coil dwell should be only ~4ms, independent on voltage.


U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI


//         RPM           1k  2k  3k  4k  5k  6k  7k  8k  9k 10k 11k 12k 13k 14k 15k 16k
uint16_t adv[8][16] = {{  0, 50, 70, 75, 75, 80, 80, 85, 90, 90, 95, 95,100,100,110,110},//12,5%
                       { 15, 60, 60, 70, 80, 80, 90, 95,100,105,110,125,130,135,145,150},//25%
                       { 15, 60, 75, 80, 85, 90, 95,105,115,125,135,145,150,160,170,180},//37,5%
                       { 60, 80, 90,100,110,115,125,135,145,155,165,170,185,190,200,210},//50%
                       { 80,100,120,125,130,135,145,160,170,180,190,200,210,225,240,250},//62,5%
                       {100,110,120,125,135,140,160,180,200,210,225,240,250,260,275,285},//75%
                       {100,110,120,130,140,150,170,190,205,220,235,250,260,275,285,300},//87,5%
                       {100,110,120,130,145,160,180,200,220,235,250,270,285,300,315,320}};//100%


//         RPM          1k  2k  3k  4k  5k  6k  7k  8k  9k 10k 11k 12k 13k 14k 15k 16k
uint8_t inj[8][16] = {{ 35, 50, 70, 75, 75, 80, 80, 85, 90, 90, 95, 95,100,100,110,110},//12,5%
                      { 30, 60, 60, 70, 80, 80, 90, 95,100,105,110,125,130,135,145,150},//25%
                      { 35, 60, 75, 80, 85, 90, 95,105,115,125,135,145,150,160,170,180},//37,5%
                      { 40, 80, 90,100,110,115,125,135,145,155,165,170,185,190,200,210},//50%
                      { 80,100,120,125,130,135,145,160,170,180,190,200,210,225,235,245},//62,5%
                      {100,110,120,125,135,140,160,180,200,200,210,215,225,235,240,250},//75%
                      {100,110,120,130,140,150,170,190,205,220,225,230,235,240,245,250},//87,5%
                      {100,110,120,130,145,160,180,200,220,235,245,250,250,250,250,255}};//100%

long unsigned calc_adv;
long unsigned period;
long unsigned dbnce = millis();
long unsigned prev = micros();

int unsigned rpm;
int unsigned inj_ign_delay;

byte sector;
byte tps_sec;
byte calc_inj;
byte calc_dwell;

bool pres;


//==================================================================================================================================================================== NON-CRITICAL INTERRUPTS

//______________________________________________________________________ STORING/READING CHANGED VALUES FROM EEPROM SUBROUTINE
void reload(uint8_t oper, uint8_t type, uint8_t col, uint8_t row, int num){//read/write, adv or ign, column for write, row for write, value to write
  int index;
  switch(oper){
//______________________________________________________________________ READ MODE
    case 0:
      for(int iter = 0; iter <= 1; iter++){
        for(int x = 1; x <= 16; col++){
          for(int y = 1; y <= 8; row++){
            if(EEPROM.read(index) != 0){
              //with ign store the uns. int. cut up into a low and hi byte
              if(!iter){
                adv[row][col] = (EEPROM.read(index) << 8) + (EEPROM.read(index+1));
                Serial.print(EEPROM.read(index) << 8) + (EEPROM.read(index+1));
                Serial.print(", ");
              }
              //with inj store only 1 byte (val. limited to 0-255)
              else{
                inj[row][col] = EEPROM.read(index);
                Serial.print(EEPROM.read(index));
                Serial.print(", ");
              }
            }
            else Serial.print("0, ");
            index += 2;
          }
        Serial.println(" ");
        }
      Serial.println(" ");
      }
      break;
//______________________________________________________________________ WRITE MODE
    case 1:
      Serial.print(" val");
      Serial.print(num);
      Serial.print(" row");
      Serial.print(row);
      Serial.print(" col");
      Serial.print(col);
      Serial.println("w");
      EEPROM.write(col*row+(type*16*8*2),num & 0xff);
      EEPROM.write((col*row+(type*16*8*2))+1,num >> 8);
      break;
    default:
      break;
  }
}

//______________________________________________________________________ BUTTON ADV+/- & IGN+/- INTERRUPT
ISR(PCINT0_vect) {
  if(millis()-dbnce >= 250 && pres){
    dbnce = millis();
    switch(PINB & 0b00001111){
      case 14:
        adv[tps_sec][sector]+=5;
        reload(1,0,tps_sec,sector,adv[tps_sec][sector]);
        break;
      case 13:
        adv[tps_sec][sector]-=5;
        reload(1,0,tps_sec,sector,adv[tps_sec][sector]);
        break;
      case 11:
        inj[tps_sec][sector]+=2;
        reload(1,1,tps_sec,sector,inj[tps_sec][sector]);
        break;
      case 7:
        inj[tps_sec][sector]-=2;
        reload(1,1,tps_sec,sector,inj[tps_sec][sector]);
        break;
      default:
        break;
    }
  }
  pres =! pres;
}

//==================================================================================================================================================================== CRITICAL INTERRUPTS
void TDC() {
//calculate period, store millis() to prev. time, then set OCR0A to coresponding value, and enable timer0
  //use for only 1 pulse per rev.
/*
  period = micros() - prev;
  prev = micros();
  ICR1 = calc_adv;
  TIMSK1 |= (1 << OCIE0A);//enables timer1
*/
  //use for multi-pulse per rev, w/ 1 empty space. gotta also do some multiplication in rpm calculation depending on N# of teeth
//*
  if ((micros() - prev) <= ((period << 1) - 50)){
    period = micros() - prev;
    prev = micros();
  }
  else{
    ICR1 = calc_adv * 0.9;
    inj_ign_delay = calc_adv - ICR1;
    TIMSK1 |= (1 << OCIE0A);//enables timer1
  }
 
//*/  
}

//______________________________________________________________________ TIMER1 INTERRUPT
ISR(TIMER1_COMPA_vect){
  //turn on pin 4 via direct port manipulation
  //transfer spark dwell to timer0, enable it
  //disable timer1
  if(ICR1){
    //ignition timing
    PORTD |=  0b00010000;
    OCR2A = 150; //spark dwell
    TIMSK1 &= (0 << OCIE1A);//disables timer1
    TIMSK2 |= (1 << OCIE2A);//enables timer2
  }
  else{
    //injector timing
    PORTD |=  0b00100000;
    OCR2A = calc_inj;
    TIMSK2 |= (1 << OCIE2A);//enables timer2
  }
}

//______________________________________________________________________ TIMER2 INTERRUPT
ISR(TIMER2_COMPA_vect){
  //turn of pin 4 via direct port manipulation
  //turn timer0 off
  PORTD &=  0b11001111;
  TIMSK2 &= (0 << OCIE2A);//disables timer0
}

//==================================================================================================================================================================== SETUP
void setup() {

  Serial.begin(9600);

  u8g.setColorIndex(1); 
  u8g.setFont(u8g_font_unifont);

  pinMode(6, OUTPUT);
  pinMode(igni, OUTPUT);
  pinMode(fuel, OUTPUT);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(hall, INPUT_PULLUP);

  cli();//stop interrupts

//______________________________________________________________________ SETUP INTERRUPT ON LOWER HALF PORTB
  PCICR |= 0b00000001;
  PCMSK0 |= 0b00001111;
  
  attachInterrupt(digitalPinToInterrupt(hall), TDC, RISING);

//______________________________________________________________________ SETUP TIMER0 AS PWM (TO BE FAZED OUT)
//using timer 0 as pwm generator without messing up the internal counting of millis() and micros()
  //TCCR0A |= 0b10000011;
  //TCCR0B |= 0b00000001;

//______________________________________________________________________ SETUP TIMER1 AS CTC
  ICR1 = 25000;
  // Set CS00&01 bit for 64 prescaler
  TCCR1B = 0b00011010;

//______________________________________________________________________ SETUP TIMER2 AS CTC
  OCR2A = 100;
  TCCR2A = 0b0000010;
  // Set CS00&01&02 bit for 1024 prescaler
  TCCR2B = 0b00000111;


//______________________________________________________________________ NOTES
//strategy for optimisation of injection and ignition on 2 timers:
//in vector_timer1 if OCR1A == idk, like 10K it means it should do ign, Turn-On-Timer2 and shut off, otherwise it should do inj, TOT2, and load 10K into OCR1A. this should only take like 15-20 clock cycles :)
//in vector_timer2 turn of both ign. and inj. output

 sei();
}


//==================================================================================================================================================================== NON CRITICAL TIMING LOOP
void loop() {
//______________________________________________________________________ CALCULATIONS
  tps_sec = analogRead(tps)/128;
  sector = (((1.0/period)*60000000)+0)/1100;
  rpm = (1/((float)period))*60000000;

//gets the desired advance, and calculates it into a "time" format to paste into comp. reg. of timer 1 on TDC interrupt
  calc_adv = ((1-(adv[tps_sec][sector]/3600.0))*(period/4));

//does PWM for injector, might be supperceeded by just timing based, one-per-rev injection of varying length
  calc_inj = inj[tps_sec][sector];
/*  if(rpm >= 500){
    TCCR0A |= 0b10000011;
    OCR0A = inj[tps_sec][sector];
  }
  else {
    TCCR0A &= 0b00000000;
    OCR0A = 0;
  }*/

//______________________________________________________________________ DISPLAY "VITAL" INFO TO OLED
  u8g.firstPage();  
  do {
  u8g.setPrintPos(0, 10); 
  u8g.print("adv:");
  u8g.print(adv[tps_sec][sector]);
 
  u8g.setPrintPos(70, 10);
  u8g.print("inj:");
  u8g.print(OCR0A);
  
  u8g.setPrintPos(0, 25); 
  u8g.print("rpm:");
  u8g.print(rpm);
  
  u8g.setPrintPos(70, 25); 
  u8g.print("tps:");
  u8g.print(analogRead(tps)/4);
  } while( u8g.nextPage() );
  
//______________________________________________________________________ DEBUG
/*  
  Serial.print(" sector ");
  Serial.print(sector);
  Serial.print(" adv ");
  Serial.print(calc_adv);
  Serial.print(" inj ");
  Serial.print(OCR0A);
  Serial.print(" period ");
  Serial.print(period);
  Serial.print(" rpm ");
  Serial.println((int)((1/((float)period))*60000000));
*/

//______________________________________________________________________ NOTES
}
