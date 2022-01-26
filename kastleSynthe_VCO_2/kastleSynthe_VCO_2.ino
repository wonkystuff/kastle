
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

//#define F_CPU 8000000  // This is used by delay.h library - now set by the IDE
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>

#include "TR_HH_AT.h"

//global variables

uint8_t mode;
uint8_t analogChannelRead = 1;
volatile uint8_t analogValues[4];
uint8_t lastAnalogValues[4];
uint8_t lastAnalogChannelRead;
bool firstRead = false;

uint8_t pwmIncrement, _upIncrement, _downIncrement, upIncrement, downIncrement;
bool quantizer;
const uint8_t analogToDigitalPinMapping[4] = {
  PORTB5, PORTB2, PORTB4, PORTB3
};


//defines for synth types
//all are dual oscillator setups -
#define NOISE 1 // phase distortion -
#define FM 0 //aka phase modulation
#define TAH 2 //aka track & hold modulation (downsampling with T&H)


#define LOW_THRES 150
#define HIGH_THRES 162
#define LOW_MIX 300
#define HIGH_MIX 900

//defines for synth parameters
#define PITCH 2 // Pitch
#define WS_1  3 // Timbre
#define WS_2  1 // Waveshape

// #### Macros to access ADC registers
#define initADC() (ADMUX  = 0, ADCSRA = _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) )
#define connectChannel(c) (ADMUX = _BV(ADLAR) | c)
#define startConversion() (ADCSRA |= _BV(ADSC))
#define getConversionResult() (ADCH)

const PROGMEM uint8_t  sinetable[128] = {
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
unsigned char wavetable[256];

uint8_t startupRead = 0;

// happends at the startup
void
setup(void)
{
  writeWave(0);
  digitalWrite(5, HIGH); //turn on pull up resistor for the reset pin

  //set outputs
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);

  setTimers(); //setup interrupts

  //setup ADC and run it in interrupt
  initADC();
  connectChannel(analogChannelRead);
  startConversion();

  _delay_us(100);
  while (startupRead < 12) {
    loop();
  }
}

void setTimers(void)
{
  PLLCSR |= (1 << PLLE);               // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1 << PLOCK)))
    ;    // Wait for PLL lock
  PLLCSR |= (1 << PCKE);               // Enable PLL as clock source for timer 1
  PLLCSR |= (1 << LSM); //low speed mode 32mhz
  cli();                               // Interrupts OFF (disable interrupts globally)

  //  setup timer 0 to run fast for audiorate interrupt

  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
  TCCR0B = _BV(CS00);

  TCCR1 = 0;                  // stop the timer
  TCNT1 = 0;                  // zero the timer
  GTCCR = _BV(PSR1);          // reset the prescaler
  OCR1A = 255;                // set the compare value
  OCR1C = 255;
  // OCR1C = 31;
  TIMSK =  _BV(OCIE1A);      // interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1
  TCCR1 = _BV(CTC1) | _BV(CS12) | _BV(CS10);

  sei();
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

uint8_t  sample;
volatile uint8_t  sample2;
uint16_t _phase;
uint16_t frequency;
uint16_t _phase2, _phase4, _phase5, _phase6;
int frequency2, frequency4, frequency5, frequency6;
uint8_t  _phase3;

void
fm(void)
{
  _phase += frequency;
  uint8_t _p8  = _phase >> 8;

  _phase2 += frequency2;
  uint8_t _p28 = _phase2 >> 8;

  static uint8_t _lastDownRamp;
  
  uint8_t wsr = analogValues[WS_2];
  uint8_t wsl= 255-wsr;

  uint8_t _phs = (_phase + (wsr * wavetable[_p28])) >> 6;
  sample = (wavetable[_phs] );

  uint8_t downRamp = 255 - _p8;
  
  uint8_t shft;
  // Start of ramp |\, so hard sync the start of phase4
  if (_lastDownRamp < downRamp)
  {
    _phase4 = 64 << 8; // hard sync for phase distortion
    shft = downRamp - _lastDownRamp;
  }
  else
  {
    _phase4 += frequency4;
    shft = _lastDownRamp - downRamp;
  }

  if (shft > 3)
  {
    _phase4 += shft << 8; //soft sync for previous settings of waveshape
  }

  _phase5 += frequency5;

  uint8_t _saw = (downRamp * wsr) >> 8;
  uint8_t pdSignal = ((_saw * wavetable[_phase4 >> 8] ) >> 8);
  uint8_t sinSignal = ((wavetable[_phase5 >> 8] * wsl) >> 8);
  sample2 =  pdSignal + sinSignal;

  // Remember the _saw value for next time...
  _lastDownRamp = downRamp;
}

void
noise(void)
{
  uint8_t _p8  = _phase >> 8;
  uint8_t _p28 = _phase2 >> 8;

  _phase += frequency;
  _phase2 += frequency2;

  if ((_phase >> 2) >= (analogValues[WS_2] - 100) << 5)
  {
    _phase = 0;
  }
  uint8_t _sample = (char)pgm_read_byte(sampleTable + (_phase >> 2));
  sample = (_sample * wavetable[_p28]) >> 8;
  sample2 = (wavetable[_phase3 + _p8]);
  _phase3 = _phase2 >> 5;
}

void
tah(void)
{
  uint8_t _p8  = _phase >> 8;
  uint8_t _p28 = _phase2 >> 8;
  uint8_t _p48 = _phase4 >> 8;
  uint8_t _p58 = _phase5 >> 8;
  uint8_t _p68 = _phase6 >> 8;

  _phase += frequency;
  _phase2 += frequency2;
  _phase4 += frequency4;
  _phase5 += frequency5;
  _phase6 += frequency6;

  if (_p28 > analogValues[WS_2])
  {
    sample = wavetable[_p8];
  }
  sample2 = (wavetable[_p28] + wavetable[_p48] + wavetable[_p58] + wavetable[_p68]) / 4;
}

typedef void (*synth_t)(void);

synth_t synthFunc = fm;

ISR(TIMER1_COMPA_vect)  // render primary oscillator in the interupt
{
  OCR0A = sample;
  OCR0B = sample2;

  synthFunc();
}

const uint8_t multiplier[3][8] = {
  {2, 2, 2, 3, 1, 2, 4, 4},
  {3, 4, 5, 2, 1, 5, 6, 8},
  {3, 8, 7, 8, 7, 6, 8, 16}
};

void setFrequency2(uint16_t input) {
  if (mode == NOISE)
  {
    frequency2 = (((input - 350) << 2) + 1) / 2; //sampleEnd=map(input,300,1024,0,sampleLength);//
  }
  else if (   mode == TAH)
  {
    uint8_t multiplierIndex = (analogValues[WS_2] + (1 << 4)) >> 5; // (1<<4) offset is to round the value into the 'middle' of the index-band, otherwise band 8 is hard to reach
    frequency2 = (input << 2) + 1;
    frequency4 = (frequency2 + 1) * multiplier[0][multiplierIndex]; //+analogValues[WS_2]>>4
    frequency5 = (frequency2 - 3) * multiplier[1][multiplierIndex]; //+analogValues[WS_2]>>3
    frequency6 = (frequency2 + 7) * multiplier[2][multiplierIndex]; //+analogValues[WS_2]>>2
    //frequency6 = ((frequency2 + frequency) /2) * multiplier[2][multiplierIndex]; //+analogValues[WS_2]>>2
  }
  else
  {
    frequency2 = (input << 2) + 1;
    frequency4 = frequency2;
    frequency5 = frequency2;
  }
}

void setFrequency(uint16_t input) {
  if (mode == NOISE)
  {
    frequency = ((input - 200) << 2) + 1;
  }
  else
  {
    frequency = (input << 2) + 1;
  }
}


void loop()
{
  //synthesis();  // create secondary output
  modeDetect();
}

void modeDetect() {
  if (analogValues[0] < LOW_THRES)
  {
    synthFunc = noise;
    mode = NOISE; //, incr=11,_incr=6, bitShift=2, osc2offset=270;
  }
  else if (analogValues[0] > HIGH_THRES)
  {
    synthFunc = tah;
    mode = TAH; //, incr=24,_incr=6,bitShift=4,osc2offset=255;
  }
  else
  {
    synthFunc = fm;
    mode = FM; //, incr=11,_incr=5,bitShift=4,osc2offset=255;
  }
}


uint8_t analogChannelSequence[6] = {0, 1, 0, 2, 0, 3};
uint8_t analogChannelReadIndex;

ISR(ADC_vect)
{
  // interrupt triggered at completion of ADC conversion
  startupRead++;
  if (!firstRead)
  {
    // discard first reading due to ADC multiplexer crosstalk
    //update values and remember last values
    lastAnalogValues[analogChannelRead] = analogValues[analogChannelRead];
    analogValues[analogChannelRead] = getConversionResult();

    //set ADC MULTIPLEXER to read the next channel
    lastAnalogChannelRead = analogChannelRead;
//    if (!analogChannelRead)
//    {
//      modeDetect();
//    }
    analogChannelReadIndex++;
    if (analogChannelReadIndex > 5)
    {
      analogChannelReadIndex = 0;
    }
    analogChannelRead = analogChannelSequence[analogChannelReadIndex];
 
    connectChannel(analogChannelRead);
    // set controll values (if relevant value read)

    if(lastAnalogChannelRead==PITCH)
    {
      setFrequency(analogValues[PITCH]<<2);//constrain(mapLookup[,0,1015));
    }
    else if(lastAnalogChannelRead==WS_1)
    {
      setFrequency2(analogValues[WS_1]<<2);
    }
    firstRead = true;
    //start the ADC - at completion the interupt will be called again
    startConversion();
  }
  else {
    /*
      at the first reading off the ADX (which will not used)
      something else will happen the input pin will briefly turn to output to
      discarge charge built up in passive mixing ciruit using zener diodes
      because zeners have some higher unpredictable capacitance, various voltages might get stuck on the pin
    */

    if ( mode == NOISE) { // && analogChannelRead!=PITCH){//if(analogChannelRead==0){
      //  bitWrite(PORTB,analogToDigitalPinMapping[analogChannelRead],1);
      // renderDecay();
    }

    else {
      if (analogValues[analogChannelRead] < 200)
      {
        bitWrite(DDRB, analogToDigitalPinMapping[analogChannelRead], 1);
      }
      bitWrite(DDRB, analogToDigitalPinMapping[analogChannelRead], 0);
      bitWrite(PORTB, analogToDigitalPinMapping[analogChannelRead], 0);
    }
    firstRead = false;
    startConversion();
  }
}
