
/*
  KASTLE DRUM
  
The Kastle Drum is a special edition of Bastl’s classic mini modular synth focusing on algorithmic industrial glitchy drums
How do you generate rhythm on a patchable drum machine that has neither buttons, nor a programmable sequencer? You discover it! 

Drum sound synthesis with a unique dynamic acceleration charged envelope makes this rhythm box surprisingly versatile and extremely fun to play. 
The built-in VC clock generator with a stepped pattern sequencer can either run on its own or it can be synchronized to analog clock, 
while retaining the triangle LFO for parameter modulation.

The Kastle Drum is a mini modular synthesizer with a headphone output, 
2 in/out ports for interfacing other gear, and it runs on just 3 AA batteries. It is ideal for beginners in modular synthesis, 
but it will add some quite unique functionality to any modular synthesizer system. It delivers the fun of modular synthesis at a 
low price and fits into your pocket so you can play it anywhere!



Kastle Drum Features 
  -8 drum synthesis styles
  -”noises” output for less tonal content
  -DRUM selects drum sounds
  -acceleration charge dynamic envelope
  -decay time
  -PITCH control with offset and CV input with attenuator
  -voltage-controllable clock with square and triangle output
  -stepped voltage generator with random, 8 step and 16 step loop mode
  -2 I/O CV ports that can be routed to any patch point 
  -the main output can drive headphones
  -3x AA battery operation or USB power selectable by a switch
  -open source
  -durable black & gold PCB enclosure



  Writen by Vaclav Pelousek 2020
  based on the earlier kastle v1.5
  open source license: CC BY SA
  http://www.bastl-instruments.com

  -this is the code for the VCO chip of the Kastle
  -software written in Arduino 1.8.12 - used to flash ATTINY 85 running at 8mHz
  http://highlowtech.org/?p=1695
  -created with help of the heavenly powers of internet and several tutorials that you can google out
  -i hope somebody finds this code usefull (i know it is a mess :( )

  thanks to
  -Lennart Schierling for making some work on the register access
  -Uwe Schuller for explaining the capacitance of zener diodes
  -Peter Edwards for making the inspireing bitRanger
  -Ondrej Merta for being the best boss
  -and the whole bastl crew that made this project possible
  -Arduino community and the forums for picking ups bits of code to setup the timers & interrupts
  -v1.5 and kastle drum uses bits of code from miniMO DCO http://www.minimosynth.com/
*/

/* Wonkystuff notes:
 *  Build with attiny support by David Mellis: https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json
 *  Version 1.0.2 produces the same binary as the production hex file
 */

#define F_CPU 8000000  // This is used by delay.h library
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>

//#include <CB_AT.h>
//#include <CB2_AT.h>
//#include <CB3_AT.h>
//#include <CB4_AT.h>
//#include <GB_BLIP_AT.h>
//#include <GB_BZZ_AT.h>
//#include <GB_GLITCH_AT.h>
//#include <GB_HH_AT.h>
//#include <GB_KICK_AT.h>
//#include <GB_SNARE_AT.h>
//#include <GB_TUI_AT.h>
//#include <GL_A_AT.h>
//#include <GL_B_AT.h>
//#include <GL_C_AT.h>
//#include <GL_D_AT.h>
//#include <GL_E_AT.h>
//#include <GL_F_AT.h>
//#include <GL_G_AT.h>
//#include <GL_H_AT.h>
//#include <GL_I_AT.h>
//#include <HAT_AT.h>
//#include <HAT2_AT.h>
//#include <KICK_AT.h>
//#include <KICK2_AT.h>
//#include <RIDE_AT.h>
//#include <SNARE_AT.h>
//#include <SNARE2_AT.h>
//#include <TR_CB_AT.h>
//#include <TR_CLAP_AT.h>

//#include <TR_KICK_AT.h>
//#include <TR_OH_AT.h>
//#include <TR_RIM_AT.h>
//#include <TR_SNARE_AT.h>
//#include <TR_TOM_AT.h>

//#include "SINE.h" //sinewave wavetable - modified but originnaly from the Mozzi library

#include "TR_HH_AT.h"
//#include <SAW.h>
//#include <CHEB4.h>

//global variables

volatile uint8_t mode;
uint8_t analogChannelRead = 1;
uint8_t analogValues[4];
uint8_t lastAnalogValues[4];
uint8_t lastAnalogChannelRead;
bool firstRead = false;

const uint8_t analogToDigitalPinMapping[4] = {
  PORTB5, PORTB2, PORTB4, PORTB3
};

//defines for synth types
//all are dual oscillator setups -
#define NOISE 1 // phase distortion -
#define FM 0 //aka phase modulation
#define TAH 2 //aka track & hold modulation (downsampling with T&H)


#define LOW_THRES 140
#define HIGH_THRES 162

//defines for synth parameters
#define PITCH 3
#define WS_1 2
#define WS_2 1

const char PROGMEM sinetable[128] = {
  0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 9, 10, 11, 12, 14, 15, 17, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35, 37, 40, 42, 44, 47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76, 79, 82, 85, 88, 90, 93, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124,
  128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173, 176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244, 245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255,
};

//the actual table that is read to generate the sound
uint8_t wavetable[256];

uint8_t startupRead = 0;

volatile uint8_t decayVolume;
volatile uint16_t decayTime = 50;

volatile uint8_t decayVolume2 = 0;

void setup()
{ //happends at the startup
  writeWave(0);
  digitalWrite(5, HIGH); //turn on pull up resistor for the reset pin
  //set outputs
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);

  setTimers(); //setup interrupts

  //setup ADC and run it in interrupt
  init();
  connectChannel(analogChannelRead);
  startConversion();
  _delay_us(100);
  while (startupRead < 12) {
    loop();
  }
}

void setTimers(void)
{
  /*
    TCCR0A=0;
    TCCR0B=0;
    bitWrite(TCCR0A,COM0A0,0);
    bitWrite(TCCR0A,COM0A1,1);
    bitWrite(TCCR0A,COM0B0,0);
    bitWrite(TCCR0A,COM0B1,1);
    bitWrite(TCCR0A,WGM00,1);
    bitWrite(TCCR0A,WGM01,1);
    bitWrite(TCCR0B,WGM02,0);
    bitWrite(TCCR0B,CS00,1);
  */
  PLLCSR |= (1 << PLLE);               // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1 << PLOCK)));    // Ensure PLL lock
  PLLCSR |= (1 << PCKE);               // Enable PLL as clock source for timer 1
  PLLCSR |= (1 << LSM); //low speed mode 32mhz
  cli();                               // Interrupts OFF (disable interrupts globally)


  TCCR0A = 2 << COM0A0 | 2 << COM0B0 | 3 << WGM00;
  TCCR0B = 0 << WGM02 | 1 << CS00;



  //  setup timer 0 to run fast for audiorate interrupt

  TCCR1 = 0;                  //stop the timer
  TCNT1 = 0;                  //zero the timer
  GTCCR = _BV(PSR1);          //reset the prescaler
  OCR1A = 255;                //set the compare value
  OCR1C = 255;
  // OCR1C = 31;
  TIMSK = _BV(OCIE1A);// | _BV(TOIE0);    //TOIE0    //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1
  TCCR1 = _BV(CTC1) | _BV(CS12);// | _BV(CS10) ;//| _BV(CS11);//  | _BV(CS11) ;// //| _BV(CS13) | _BV(CS12) | _BV(CS11) |

  // GTCCR  = (1 << PWM1B) | (1 << COM1B1); // PWM, output on pb4, compare with OCR1B (see interrupt below), reset on match with OCR1C
  //OCR1C  = 0xff;                         // 255
  // TCCR1  = (1 << CS10);                  // no prescale

  sei();
  // TIMSK |=_BV(TOIE0);
}

void writeWave(int wave) {
  switch (wave) {
    case 0:
      sineWave();
      break;
    case 1:
      triangleWave();
      break;
    case 2:
      squareWave();
      break;
    case 3:
      sawtoothWave();
      break;
    case 4:
      digitalWrite(0, LOW);
      zeroWave();
      break;
  }
}

//functions to populate the wavetable
void sineWave() {                                       //too costly to calculate on the fly, so it reads from the sine table. We use 128 values, then mirror them to get the whole cycle
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = pgm_read_byte_near(sinetable + i);
  }
  wavetable[128] = 255;
  for (int i = 129; i < 256; ++i) {
    wavetable[i] = wavetable[256 - i] ;
  }
}
void sawtoothWave() {
  for (int i = 0; i < 256; ++i) {
    wavetable[i] = i; // sawtooth
  }
}
void triangleWave() {
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = i * 2;
  }
  int value = 255;
  for (int i = 128; i < 256; ++i) {
    wavetable[i] = value;
    value -= 2;
  }
}
void squareWave() {
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = 255;
  }
  for (int i = 128; i < 256; ++i) {
    wavetable[i] = 1;                  //0 gives problems (offset and different freq), related to sample  = ((wavetable[phase >> 8]*amplitude)>>8);
  }
}
void zeroWave() {
  for (int i = 0; i < 256; ++i) {
    wavetable[i] = 1;                  //0 gives problems
  }
}
/*
  ISR(TIMER0_OVF_vect){ // increment _clocks at PWM interrupt overflow - this gives 16bit time consciousnes to the chip (aproxx 2 seconds before overflow)
  _clocks++;
  }
*/
volatile uint8_t  sample;
volatile uint16_t _phase, _lastPhase;
volatile uint16_t frequency;
volatile uint8_t  sample2;
volatile uint16_t _phase2, _phase4, _phase5, _phase6;
volatile uint16_t frequency2, frequency4, frequency5, frequency6;
volatile uint8_t  _phs;
volatile uint8_t  _phase3;
volatile uint8_t  pitchEnv;

ISR(TIMER1_COMPA_vect)  // render primary oscillator in the interupt
{
  OCR0A = sample;//(sample+sample2)>>1;
  OCR0B = sample2;//_phs;// sample90;

  //_lastPhase=_phase;
  if (pitchEnv) {
    _phase += (frequency + (decayVolume));
    _phase3 += (frequency + (decayVolume)); //frequency;//
    _phase2 += (frequency2 + (decayVolume));
  }
  else {
    _phase += frequency;
    _phase3 += frequency;
    _phase2 += frequency2;
  }
  if(!(_phase%4) )
  {
    _phase4 += frequency4;
  }


  //_phase5 += frequency5;
  // _phase5 += frequency5;
  if (mode == FM) {
    _phs = (_phase + (analogValues[WS_2] * wavetable[_phase2 >> 8])) >> 6;
    sample = ((wavetable[_phs] ) * decayVolume) >> 8;
  }
  else if (mode == NOISE)
  {
//    _sample = (char)pgm_read_byte_near(sampleTable + (_phase >> 2));
//    _sample = (_sample * wavetable[_phase2 >> 8]) >> 8;
//    sample = (sample * decayVolume) >> 8;
  }
}


void setFrequency2(uint16_t input) {

  mode = (input + 64) >> 7;

  if (mode > 6)
  {
    bitWrite(TCCR0B, CS00, 0), bitWrite(TCCR0B, CS01, 1);
  }
  else
  {
    bitWrite(TCCR0B, CS00, 1), bitWrite(TCCR0B, CS01, 0);
  }
  frequency2 = (input << 4) + 1;
  frequency4 = 512 - input;
}

void setFrequency(uint16_t input) {

  if (mode == NOISE )
  {
    frequency = ((input - 200) << 2) + 1; //NOISE
  }
  else if (mode > 3)
  {
    frequency = (input) + 1;
  }
  else
  {
    frequency = (input << 2) + 1;
  }
  frequency5 = frequency;
}


#define SAMPLE_PHASE_SHIFT 5
void synthesis(void) 
{
  uint8_t _sample2, _lastSample2, _sample;

  if ((_phase3 >> 2) >= (analogValues[WS_1]) << 4)
  {
    _phase3 = 0;
  }
  
  _lastSample2 = sample2;
  _sample2 = (char)pgm_read_byte_near(sampleTable + (_phase3) ) + 128;
  if (analogValues[PITCH] > wavetable[(_phase4 >> 9) + (_sample2>>5)])
  {
    _sample2 = _lastSample2; //128;//_lastSample2;
  }
  else
  {
    _sample2 = abs(_sample2 - _lastSample2);
  }
  sample2 =  ((_sample2 * (decayVolume2)) >> 8);
  
  switch(mode) {
    case FM:
      return; // avoid doing the common stuff below
      break;

    case NOISE:
      if ((_phase >> 2) >= (analogValues[WS_2] - 100) << 5) {
        _phase = 0;
      }
      _sample = (char)pgm_read_byte_near(&sampleTable[_phase >> 2]) + 128;
      _sample = (_sample * wavetable[_phase2 >> 8]) >> 8;
      break;
      
    case TAH:
      if ((_phase2 >> 8) < analogValues[WS_2] + 5) {
        _phs = _phase >> 8;
        _sample = wavetable[_phs];
      }
      break;

    default:
      // Sample-based sounds
      uint16_t zz = _phase >> SAMPLE_PHASE_SHIFT;
      const char *sPtr;
      if (mode == 3) {
        if (zz > sample2Length)
        {
          _phase = 0;
          zz = 0;
        }
        sPtr = &sample2Table[zz];
      }
      else if (mode == 4) {
        if (zz > sample3Length)
        {
          _phase = 0;
          zz = 0;
        }
        sPtr = &sample3Table[zz];
      }
      else if (mode == 5) {
        if (zz > sample4Length)
        {
          _phase = 0;
          zz = 0;
        }
        sPtr = &sample4Table[zz];
      }
      else if (mode == 6) {
        if (zz > sampleLength)
        {
          _phase = 0;
          zz = 0;
        }
        sPtr = &sampleTable[zz];
      }
      else if (mode == 7) {
        if (zz > sample4Length)
        {
          _phase = 0;
          zz = 0;
        }
        sPtr = &sample4Table[zz];
      }
      _sample = (char)pgm_read_byte_near(sPtr) + 128;
      break;
  }

  sample = (_sample * decayVolume) >> 8;
}

void loop(void)
{
  synthesis();
  renderDecay();
  //trigDetect();
}

void trigDetect(void)
{ //rather trigger machine
  static uint8_t trigState = 0;
  uint8_t lastTrigState = 0;

  lastTrigState = trigState;

  if (analogValues[0] < LOW_THRES)
  {
    trigState = 0; //, incr=11,_incr=6, bitShift=2, osc2offset=270;
  }
  else if (analogValues[0] > HIGH_THRES)
  {
    trigState = 2; //, incr=24,_incr=6,bitShift=4,osc2offset=255;
  }
  else
  {
    trigState = 1; //, incr=11,_incr=5,bitShift=4,osc2offset=255;
  }

  if (lastTrigState != trigState)
  {
    trigger(trigState, lastTrigState);
    _phase = 0;
  }
}

uint8_t trigger(uint8_t intensity_1, uint8_t intensity_2)
{
  //lastTriggerTime=triggerTime;
  //triggerTime=some increment shit
  //decayTime=triggerTime-lastTrigerTime;
  uint8_t s = abs(intensity_1 - intensity_2);
  if (s == 1)
  {
    decayVolume = 127;
    decayVolume2 = 255;
  }
  else
  {
    decayVolume = 255;
    decayVolume2 = 150;
  }
}


uint8_t analogChannelSequence[6] = {0, 1, 0, 2, 0, 3};
uint8_t analogChannelReadIndex;

void setDecay(uint8_t dec)
{
  if (dec > 100)
  {
    decayTime = constrain(dec - 120, 1, 255);
    pitchEnv = 0;
  }
  else
  {
    decayTime = (100 - dec);
    pitchEnv = 255; //decayTime;
  }
}

ISR(ADC_vect) { // interupt triggered ad completion of ADC counter
  startupRead++;
  if (!firstRead) { // discard first reading due to ADC multiplexer crosstalk
    //update values and remember last values
    lastAnalogValues[analogChannelRead] = analogValues[analogChannelRead];
    analogValues[analogChannelRead] = getConversionResult();
    //set ADC MULTIPLEXER to read the next channel
    lastAnalogChannelRead = analogChannelRead;
    if (!analogChannelRead)
    {
      trigDetect();
    }
    analogChannelReadIndex++;
    if (analogChannelReadIndex > 5)
    {
      analogChannelReadIndex = 0;
    }
    analogChannelRead = analogChannelSequence[analogChannelReadIndex];
    connectChannel(analogChannelRead);
    // set controll values if relevant (value changed)
    if (lastAnalogChannelRead == PITCH)
    {
      if (lastAnalogValues[PITCH] != analogValues[PITCH])
      {
        setFrequency(analogValues[PITCH] << 2);
        decayVolume2 = constrain(decayVolume2 + ((abs(lastAnalogValues[PITCH] - analogValues[PITCH]) << 3)), 0, 255);
//        uint8_t dyn = lastAnalogValues[PITCH] > analogValues[PITCH] ?
//                          lastAnalogValues[PITCH] - analogValues[PITCH] :
//                          analogValues[PITCH] - lastAnalogValues[PITCH];
//        uint16_t sum = decayVolume2 + (dyn << 3);
//        decayVolume2 = sum > 255 ? 255 : decayVolume2 + (dyn << 3);
      }
    }
    else if (lastAnalogChannelRead == WS_1)
    {
      if (lastAnalogValues[WS_1] != analogValues[WS_1])
      {
        setFrequency2(analogValues[WS_1] << 2);
        decayVolume = constrain(decayVolume + ((abs(lastAnalogValues[WS_1] - analogValues[WS_1]) << 2)), 0, 255);
//        uint8_t dyn = lastAnalogValues[WS_1] > analogValues[WS_1] ?
//                          lastAnalogValues[WS_1] - analogValues[WS_1] :
//                          analogValues[WS_1] - lastAnalogValues[WS_1];
//        uint16_t sum = decayVolume + (dyn << 2);
//        decayVolume = sum > 255 ? 255 : decayVolume + (dyn << 2);
      }
    }
    else if (lastAnalogChannelRead == WS_2)
    {
      if (lastAnalogValues[WS_2] != analogValues[WS_2])
      {
        setDecay(analogValues[WS_2]);
      }
    }
    firstRead = true;
  }
  else
  {
    /*
      at the first reading off the ADX (which will not used)
      something else will happen the input pin will briefly turn to output to
      discarge charge built up in passive mixing ciruit using zener diodes
      because zeners have some higher unpredictable capacitance, various voltages might get stuck on the pin
    */
    if (analogValues[analogChannelRead] < 200)
    {
      bitWrite(DDRB, analogToDigitalPinMapping[analogChannelRead], 1);
    }
    bitWrite(DDRB, analogToDigitalPinMapping[analogChannelRead], 0);
    bitWrite(PORTB, analogToDigitalPinMapping[analogChannelRead], 0);

    firstRead = false;
  }
  //start the ADC - at completion the interupt will be called again
  startConversion();
}


volatile uint16_t decayCounter = 0;

volatile uint16_t decayCounter2 = 0;

void renderDecay() {
  if (decayTime != 0) {
    decayCounter += 6;
    if (decayCounter >= decayTime)
    {
      decayCounter = 0;
      if (decayVolume > 0)
      {
        decayVolume -= ((decayVolume >> 6) + 1);
      }
    }

    decayCounter2 += 6;
    if (decayCounter2 >= decayTime)
    {
      decayCounter2 = 0;
      if (decayVolume2 > 0)
      {
        decayVolume2 -= ((decayVolume2 >> 6) + 1);
      }
    }
  }
}

// #### FUNCTIONS TO ACCES ADC REGISTERS
void init() {

  ADMUX  = _BV(ADLAR);      // only need 8 bit output
  bitWrite(ADCSRA, ADEN, 1); //adc enabled
  bitWrite(ADCSRA, ADPS2, 1); // set prescaler
  bitWrite(ADCSRA, ADPS1, 1); // set prescaler
  bitWrite(ADCSRA, ADPS0, 1); // set prescaler
  bitWrite(ADCSRA, ADIE, 1); //enable conversion finished interupt
  bitWrite(SREG, 7, 1);
  // prescaler = highest division
}


// channel 8 can be used to measure the temperature of the chip
void connectChannel(uint8_t number) {
  ADMUX &= (11110000);
  ADMUX |= number;
}

void startConversion() {
  bitWrite(ADCSRA, ADSC, 1); //start conversion
}

bool isConversionFinished() {
  return (ADCSRA & (1 << ADIF));
}

bool isConversionRunning() {
  return !(ADCSRA & (1 << ADIF));
}

uint8_t getConversionResult() {
  return ADCH;
}
