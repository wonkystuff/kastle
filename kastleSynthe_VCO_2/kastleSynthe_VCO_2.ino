
/*
  KASTLE VCO v 1.5


  Features
  -5 synthesis modes = phase modulation, phase distortion, tarck & hold modulation, formant synthesis, noise wtf
  -regular & alternative waveform output by 2 PWM channels
  -3 sound parameters controlled by voltage inputs
  -voltage selectable synthesis modes on weak I/O Reset pin


  Writen by Vaclav Pelousek 2017
  open source license: CC BY SA
  http://www.bastl-instruments.com


  -software written in Arduino 1.0.6 - used to flash ATTINY 85 running at 8mHz
  -created with help of the heavenly powers of internet and several tutorials that you can google out
  -i hope somebody finds this code usefull (i know it is a mess :( )

  thanks to
  -Lennart Schierling for making some work on the register access
  -Uwe Schuller for explaining the capacitance of zener diodes
  -Peter Edwards for making the inspireing bitRanger
  -Ondrej Merta for being the best boss
  -and the whole bastl crew that made this project possible
  -Arduino community and the forums for picking ups bits of code to setup the timers & interrupts
  -v1.5 uses bits of code from miniMO DCO http://www.minimosynth.com/
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
 
// F_CPU is now defined by the IDE - here as a **reminder**
#//define F_CPU 8000000  // This is used by delay.h library

#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>

#include "TR_HH_AT.h"


//defines for synth types
//all are dual oscillator setups -
#define FM     0u // aka phase modulation
#define NOISE  1u // phase distortion -
#define TAH    2u // aka track & hold modulation (downsampling with T&H)

#define LOW_THRES   (150u)
#define HIGH_THRES  (162u)

//defines for synth parameters
#define MODE   0u
#define WS_2   1u
#define PITCH  2u
#define WS_1   3u

//global variables

uint8_t mode = NOISE;
uint8_t analogValues[4];

const uint8_t analogToDigitalPinMapping[4] = {
  PORTB5, PORTB2, PORTB4, PORTB3
};

const unsigned char PROGMEM sinetable[128] = {
  0,   0,   0,   0,   1,   1,   1,   2,   2,   3,   4,   5,   5,   6,   7,   9,
  10,  11,  12,  14,  15,  17,  18,  20,  21,  23,  25,  27,  29,  31,  33,  35,
  37,  40,  42,  44,  47,  49,  52,  54,  57,  59,  62,  65,  67,  70,  73,  76,
  79,  82,  85,  88,  90,  93,  97,  100, 103, 106, 109, 112, 115, 118, 121, 124,
  128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173,
  176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215,
  218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244,
  245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255,
};

//the actual table that is read to generate the sound
uint8_t wavetable[256];

uint8_t startupRead = 0;

void
setup(void)
{
  //happends at the startup
  writeWave(0);
  digitalWrite(5, HIGH); //turn on pull up resistor for the reset pin

  //set outputs
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);

  setTimers(); //setup interrupts

  //setup ADC and run it in interrupt
  init();
  connectChannel(MODE);
  startConversion();
  _delay_us(100);
  while (startupRead < 12) {
    loop();
  }  
}

void
setTimers(void)
{
  PLLCSR |= (1 << PLLE);               // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1 << PLOCK)));    // Ensure PLL lock
  PLLCSR |= (1 << PCKE);               // Enable PLL as clock source for timer 1
  PLLCSR |= (1 << LSM);                // low speed mode 32mhz
  cli();                               // Interrupts OFF (disable interrupts globally)

  TCCR0A = 2 << COM0A0 | 2 << COM0B0 | 3 << WGM00;
  TCCR0B = 0 << WGM02 | 1 << CS00;

  //  setup timer 0 to run fast for audiorate interrupt

  TCCR1 = 0;                  //stop the timer
  TCNT1 = 0;                  //zero the timer
  GTCCR = _BV(PSR1);          //reset the prescaler
  OCR1A = 255;                //set the compare value
  OCR1C = 255;

  TIMSK =  _BV(OCIE1A);
  //start timer, ctc mode, prescaler clk/1
  TCCR1 = _BV(CTC1) | _BV(CS12);

  sei();
}

void
writeWave(uint8_t wave)
{
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
void
sineWave(void)
{
  //too costly to calculate on the fly, so it reads from the sine table. We use 128 values, then mirror them to get the whole cycle
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = pgm_read_byte_near(sinetable + i);
  }
  wavetable[128] = 255;
  for (int i = 129; i < 256; ++i) {
    wavetable[i] = wavetable[256 - i] ;
  }
}

void
sawtoothWave(void)
{
  for (int i = 0; i < 256; ++i)
  {
    wavetable[i] = i; // sawtooth
  }
}

void
triangleWave(void)
{
  for (int i = 0; i < 128; ++i)
  {
    wavetable[i] = i * 2;
  }
  int value = 255;
  for (int i = 128; i < 256; ++i)
  {
    wavetable[i] = value;
    value -= 2;
  }
}

void
squareWave(void)
{
  for (int i = 0; i < 128; ++i)
  {
    wavetable[i] = 255;
  }
  for (int i = 128; i < 256; ++i)
  {
    wavetable[i] = 1;                  //0 gives problems (offset and different freq), related to sample  = ((wavetable[phase >> 8]*amplitude)>>8);
  }
}

void
zeroWave(void)
{
  for (int i = 0; i < 256; ++i)
  {
    wavetable[i] = 1;                  //0 gives problems
  }
}

uint8_t  sample;
uint16_t _phase;
uint16_t frequency;
uint8_t  sample2;
uint16_t _phase2, _phase4, _phase5, _phase6;
uint16_t frequency2, frequency4, frequency5, frequency6;
uint8_t  _phs;
uint8_t  _phase3;

ISR(TIMER1_COMPA_vect)  // render primary oscillator in the interupt
{
  OCR0A = sample;//(sample+sample2)>>1;
  OCR0B = sample2;//_phs;// sample90;

  _phase += frequency;
  _phase2 += frequency2;
  _phase4 += frequency4;
  _phase5 += frequency5;

  if (mode == FM)
  {
    _phs = (_phase + (analogValues[WS_2] * wavetable[_phase2 >> 8])) >> 6;
    sample = (wavetable[_phs] );
  }
  else
  {
    _phase3 = _phase2 >> 5;
    _phase6 += frequency6;
  }
}

const uint8_t multiplier[3][8] = {
  {2, 2, 2, 3, 1, 2, 4, 4},
  {3, 4, 5, 2, 1, 5, 6, 8},
  {3, 8, 7, 8, 7, 6, 8, 16}
};

void
setFrequency2(uint16_t input)
{
  if (mode == NOISE)
  {
    frequency2 = (((input - 300) << 2) + 1) / 2;
  }
  else if (mode == TAH)
  {
    uint8_t multiplierIndex = analogValues[WS_2] >> 5;
    frequency2 = (input << 2) + 1;
    frequency4 = (frequency2 + 1) * multiplier[0][multiplierIndex];
    frequency5 = (frequency2 - 3) * multiplier[1][multiplierIndex];
    frequency6 = (frequency2 + 7) * multiplier[2][multiplierIndex];
  }
  else
  {
    frequency2 = (input << 2) + 1;
    frequency4 = frequency2;
    frequency5 = frequency2;
  }
}

void
setFrequency(uint16_t input)
{
  frequency = mode == NOISE ? (input - 210) : input;
  frequency <<= 2;
  frequency++;
}


void
synthesis(void)
{
  switch(mode)
  {
    case FM:
    {
      static uint8_t _saw;
      uint8_t _lastSaw;
      _lastSaw = _saw;
      _saw = (((255 - (_phase >> 8)) * (analogValues[WS_2])) >> 8);
      // uint8_t _p=(_phase4 >> 8)+128;
      sample2 = ((_saw * wavetable[_phase4 >> 8] ) >> 8) + ((wavetable[_phase5 >> 8] * (255 - analogValues[WS_2])) >> 8);

      if (_lastSaw < _saw) {
        _phase4 = 64 << 8; // hard sync for phase distortion
      }
      uint8_t shft = abs(_saw - _lastSaw);
      if (shft > 3) {
        _phase5 += shft << 8; //soft sync for previous settings of waveshape
      }
    }
    break;
    
    case NOISE:
    {
      if ((_phase >> 2) >= (analogValues[WS_2] - 100) << 5) {
        _phase = 0;
      }
      uint8_t _sample = (char)pgm_read_byte_near(sampleTable + (_phase >> 2));
      _sample = (_sample * wavetable[_phase2 >> 8]) >> 8;
      sample = _sample;
      sample2 = (wavetable[_phase3 + (_phase >> 8)]);
    }
    break;
    
    case TAH:
    {
      if ((_phase2 >> 8) > analogValues[WS_2]) {
        _phs = _phase >> 8, sample = (wavetable[_phs] );
      }
      sample2 = (wavetable[_phase2 >> 8] + wavetable[_phase4 >> 8] + wavetable[_phase5 >> 8] + wavetable[_phase6 >> 8]) >> 2;
    }
    break;
  }
}

void
loop(void)
{
  synthesis();
}

void
modeDetect(uint8_t m)
{
  mode = (m < LOW_THRES) ? NOISE : (m > HIGH_THRES) ? TAH : FM;
}

uint8_t analogChannelSequence[6] = {
  MODE, WS_2,
  MODE, PITCH,
  MODE, WS_1
};

ISR(ADC_vect)
{ // interupt triggered ad completion of ADC counter

  static uint8_t analogChannelRead = 1;
  static uint8_t analogChannelReadIndex;
  static bool    skipRead = true;
  startupRead++;

  if (!skipRead)
  { // discard alternate readings due to ADC multiplexer crosstalk

    //update values
    analogValues[analogChannelRead] = getConversionResult();

    // set controll values if relevant (value changed)
    switch(analogChannelRead)
    {
      case MODE:
        modeDetect(analogValues[MODE]);
        break;

      case PITCH:
        setFrequency(analogValues[PITCH]<<2);//constrain(mapLookup[,0,1015));
        break;

      case WS_1:
        setFrequency2(analogValues[WS_1]<<2);
        break;
    }

    analogChannelReadIndex++;
    if (analogChannelReadIndex > 5) analogChannelReadIndex = 0;
    analogChannelRead = analogChannelSequence[analogChannelReadIndex];

    //set ADC MULTIPLEXER to read the next channel
    connectChannel(analogChannelRead);

    //start the ADC - at completion the interupt will be called again
    startConversion();
  }
  else
  {
    /*
      at the first reading off the ADX (which will not used)
      something else will happen the input pin will briefly turn to output to
      discarge charge built up in passive mixing ciruit using zener diodes
      because zeners have some higher unpredictable capacitance, various voltages might get stuck on the pin
    */

    if ( mode != NOISE)
    {
      if (analogValues[analogChannelRead] < 200)
      {
        bitWrite(DDRB, analogToDigitalPinMapping[analogChannelRead], 1);
      }

      bitWrite(DDRB, analogToDigitalPinMapping[analogChannelRead], 0);
      bitWrite(PORTB, analogToDigitalPinMapping[analogChannelRead], 0);
    }
    startConversion();
  }
  skipRead = !skipRead;
}


// #### FUNCTIONS TO ACCES ADC REGISTERS
void
init(void)
{
  ADMUX  = 0;

  ADCSRA = _BV(ADEN) |   // adc enabled
           _BV(ADPS2) |  // set prescaler
           _BV(ADPS1) |  // set prescaler
           _BV(ADPS0) |  // set prescaler
           _BV(ADIE);    // enable conversion finished interupt
}


// Big assumption here is that we only feed in the ADC channel
// Set ADLAR here because we only want 8 bits of the result anyway
void
connectChannel(uint8_t number)
{
  ADMUX = _BV(ADLAR) | number;
}

void
startConversion(void)
{
  bitWrite(ADCSRA, ADSC, 1); //start conversion
}

uint8_t
getConversionResult(void)
{
  return ADCH;
}
