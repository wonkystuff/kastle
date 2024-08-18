
/*
KASTLE LFO v 1.0


Features

Writen by Vaclav Pelousek 2016
open source license: CC BY SA
http://www.bastl-instruments.com
 
 
-software written in Arduino 1.0.6 - used to flash ATTINY 85 running at 8mHz
-created with help of the heavenly powers of internet and several tutorials that you can google out
-i hope somebody finds this code usefull

thanks to 
-Lennart Schierling for making some work on the register access
-Uwe Schuller for explaining the capacitance of zener diodes
-Peter Edwards for making the inspireing bitRanger
-Rob Hordijk for inventing the amazing concept of the Rungler
-Ondrej Merta for being the best boss
-and the whole bastl crew that made this project possible
 */
/*
 * January 2022:
 * Updated by J Tuffen for wonkystuff - some code reformatting and optimisation in preparation
 * for the wonkystuff kastle module for AE Modular, available early 2022.
 * Synthesis modes are untouched from the original.
 * 
 * Now builds under Arduino 1.8.x IDE, using the ATTinyCore board-support package:
 *  - Board: ATTiny25/45/85 (no bootloader)
 *  - Chip: ATTiny85
 *  - Clock Source: 8MHz (internal)
 *  - Timer 1 clock: CPU (CPU frequency)
 *  - LTO: Enabled
 *  - millis()/micros(): Disabled (saves flash)
 */
  
// F_CPU is now defined by the IDE, but is left here as a reminder
//#define F_CPU 8000000  // This is used by delay.h library
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>    // Adds delay_ms and delay_us functions


uint8_t analogChannelRead=1;
uint16_t analogValues[3];
uint8_t runglerByte;
int pwmCounter;
uint16_t upIncrement=0;
uint16_t downIncrement=255;
uint32_t _upIncrement=0;
uint32_t _downIncrement=255;
uint8_t pwmIncrement;
uint8_t waveshape,lastWaveshape;
long _value;
bool goingUp;
uint16_t counter;

bool resetState=false;
const uint8_t runglerMap[8]={
  0,   80,  120, 150, 
  180, 200, 220, 255
};

uint8_t analogPins[3]={
  A1, A2, A3
};

const uint8_t analogToDigitalPinMapping[4]={
    7, PORTB2, PORTB4, PORTB3
};

#define WSMAP_POINTS 5
  
uint16_t wsMap[10]= {
    0,  60,  127, 191, 255,
    50, 127, 190, 230, 254
};

bool       lfoFlop=true;
uint8_t    mapLookup[256];
uint8_t    lfoValue=0;
bool       firstRead=false;
uint16_t   runglerOut;
const bool usePin[4]={
    true, false, true, false
};

void
createLookup(void)
{
  for(uint16_t i=0;i<256;i++){
    mapLookup[i]=curveMap(i,WSMAP_POINTS,wsMap);
  }
}


uint32_t
curveMap(uint8_t value, uint8_t numberOfPoints, uint16_t * tableMap)
{
  uint32_t inMin=0, inMax=255, outMin=0, outMax=255;
  for(int i=0;i<numberOfPoints-1;i++)
  {
    if(value >= tableMap[i] && value <= tableMap[i+1])
    {
      inMax=tableMap[i+1];
      inMin=tableMap[i];
      outMax=tableMap[numberOfPoints+i+1];
      outMin=tableMap[numberOfPoints+i];
      i=numberOfPoints+10;
    }
  }
  return map(value,inMin,inMax,outMin,outMax);
}

// Get some random low bit
uint8_t
rnd8(void)
{
  static uint8_t r = 0x23;
  uint8_t lsb = r & 1;
  r >>= 1;
  r ^= (-lsb) & 0xB8;
  return r;
}

void
setup(void)
{ 
  digitalWrite(5,HIGH);
  pinMode(4, INPUT);
  digitalWrite(4,HIGH);
  createLookup();
  setTimers(); //setup audiorate interrupt
  runglerByte=rnd8();
  pinMode(0, OUTPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);

  GIMSK = 0x20;               // turns on pin change interrupts
  PCMSK = 0x08;               // turn on pin-change interrupts on pin PB3
  digitalWrite(4,LOW);
  init();
  connectChannel(analogChannelRead);
  startConversion();

} 

void
setTimers(void)
{
  TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
  TCCR0B = 0<<WGM02 | 1<<CS00;

  //  setup timer 0 to run fast for audiorate interrupt 
  TCCR1 = 0;                  //stop the timer
  TCNT1 = 0;                  //zero the timer
  GTCCR = _BV(PSR1);          //reset the prescaler
  OCR1A = 250;                //set the compare value
  // OCR1C = 31;
  TIMSK = _BV(OCIE1A);        //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1    
  TCCR1 = _BV(CTC1) | _BV(CS12);//  | _BV(CS11) ;//| _BV(CS10); //| _BV(CS13) | _BV(CS12) | _BV(CS11) |
  bitWrite(TCCR1,CS12,0);
  sei();
}

volatile uint8_t resetHappened=0;

// Pin change interrupt - set on state change of I/O. We set this up
// to only be enabled on PCINT3 (PB3) so if we get here we know that
// something happened on PB3
ISR(PCINT0_vect)
{
    // If the pin is currently 'high' then signal to the timing ISR
    // that a clock occurred. This will hopefully do a bit of
    // debouncing for us.
    if (PINB & _BV(PB3))
    {
        resetHappened = 1;
        TIFR |= _BV(OCF1A); // Force the timer compare interrupt?
    }
}

ISR(ADC_vect)
{
  if(!firstRead)
  {
    analogValues[analogChannelRead] = getConversionResult();

    if(analogChannelRead==2)
    {
      setFrequency(mapLookup[analogValues[2]>>2]);
    }

    analogChannelRead++;
    while(!usePin[analogChannelRead]){
      analogChannelRead++;
      analogChannelRead %= 4;
    }

    if(analogChannelRead>2)
    {
      analogChannelRead=0;
    }
    connectChannel(analogChannelRead);
    firstRead=true;
    startConversion();
  }
  else
  {
    if(analogChannelRead==2)
    {
      if(analogValues[analogChannelRead]<750)
      {
        bitWrite(DDRB,analogToDigitalPinMapping[analogChannelRead],1);
      }
      bitWrite(DDRB,analogToDigitalPinMapping[analogChannelRead],0);
      bitWrite(PORTB,analogToDigitalPinMapping[analogChannelRead],0);
    }
    firstRead=false;
    startConversion();
  }
}

volatile uint8_t rBit;

void
loop(void)
{ 
  // create a random number which is unsynchronised from the timer interrupt
  rBit = rnd8() & 0x01;
}

ISR(TIMER1_COMPA_vect)  //audiorate interrupt
{
  bitWrite(PORTB,PINB2, (lfoFlop && (lfoValue<200)));

  lfoValue++;

  // Check whether the pin-change interrupt occured
  if(resetHappened)
  {
    lfoValue=0;
    lfoFlop=0;
    resetHappened = 0;
  }

  if(lfoValue==0)
  {
    lfoFlop=!lfoFlop;

    // Get top bit of the rungler byte
    bool newBit= bitRead(runglerByte,7) ;
    runglerByte=runglerByte<<1;

    // Mode input
    if((analogValues[0]>>2)<150) // Mode is LOW
    {
      newBit=newBit;    // Bit 7 --> Bit zero
    }
    else if((analogValues[0]>>2)>162) // Mode is HIGH
    {
      newBit=rBit;      // get a random-ish new bit zero from a fast timer
    }
    else    // Mode is resting
    {
      newBit=!newBit;   // just flip the bit
    }
    // set the bottom bit of the rungler byte
    runglerByte |= newBit;

    runglerOut=0;
    bitWrite(runglerOut,0,bitRead(runglerByte,0));
    bitWrite(runglerOut,1,bitRead(runglerByte,3));
    bitWrite(runglerOut,2,bitRead(runglerByte,5));
    runglerOut=runglerMap[runglerOut];
  }
  uint8_t out = lfoFlop ? 255-lfoValue : lfoValue;

  OCR0B = out;
  OCR0A = runglerOut;
  TCNT1 = 0; 
}

void
setFrequency(uint8_t freq)
{
  uint16_t _freq=(2048-(freq<<3))+20;
  uint8_t preScaler=_freq>>7;
  preScaler+=1; //*2
  for(uint8_t i=0;i<4;i++)
  {
    bitWrite(TCCR1,CS10+i,bitRead(preScaler,i));
  }
  uint8_t compare=_freq;
  bitWrite(compare,7,0);
  OCR1A=compare+128; 
}

// #### FUNCTIONS TO ACCES ADC REGISTERS
void
init(void)
{
  ADMUX  = 0;
  ADCSRA = _BV(ADEN)  | // adc enabled
           _BV(ADPS2) | // set prescaler
           _BV(ADPS1) | // set prescaler
           _BV(ADPS0) | // set prescaler
           _BV(ADIE);   //enable conversion finished interupt
}

// channel 8 can be used to measure the temperature of the chip
void
connectChannel(uint8_t number)
{
  ADMUX &= 0x30; //(11110000);
  ADMUX |= number;  
}

void
startConversion(void)
{
  bitWrite(ADCSRA,ADSC,1); //start conversion
}

uint16_t
getConversionResult(void) {
  uint16_t result = ADCL;
  return result | (ADCH<<8);
}
