/** Note: all serial output has been removed
  * as it caused some kind of interference
  * with the shift registers, causing them
  * to not function properly.
  *
  * Code for the FFT is taken from the example
  * included in the ffft library. Had to change FFT_N to 64
  * in the library to avoid overflow.
  *
  * Wave and SD card functions taken from examples
  * for the WaveHC library, on the adafruit
  * website
  **/

#include <stdint.h>
#include <SdReader.h>
#include <FatReader.h>
#include <avr/pgmspace.h>
#include "WaveHC.h"
#include "WaveUtil.h"
#include "ffft.h"

SdReader card;
FatVolume vol;
FatReader root;
FatReader f;

WaveHC wave;

#define DEBOUNCE 100

//define analog pin for input to FFT
#define IR_AUDIO 2

//define pin for thrower
#define THROWER_PIN A0
#define VIZ_PIN A1
#define MIC_PIN A2

//define pins for DAC
#define dacLCS 2
#define dacCLK 3
#define dacDI 4
#define dacLAT 5
#define dacCCS 10

//define pins for shift registers
#define latchPin1 0
#define clockPin1 1
#define dataPin1 6
#define latchPin2 7
#define clockPin2 8
#define dataPin2 9

#define CYCSTART B00001000

unsigned long timeStart;

//LEDs on shift register one
byte ledOne;

//LEDs on shift register two
byte ledTwo;

//LEDs on shift register three
byte ledThree = CYCSTART;

//array of LED states
byte ledArray[9];

//vars for normal operation
byte highPowerCell = 0;
//byte currentCyclotron = 0;
byte numCyclotronRounds = 0;
boolean equalizerMode = false;
boolean throwing = false;
boolean cycleDown = false;
boolean throwerPlaying = false;
boolean throwerOn = false;
boolean powerstart = true;

//vars for visualizer
//FFT_N = 64
volatile  byte  position = 0;
volatile  long  zero = 0;
int16_t capture[FFT_N];			/* Wave capturing buffer */
complex_t bfly_buff[FFT_N];		/* FFT buffer */
uint16_t spektrum[FFT_N/2];		/* Spectrum output buffer */

void sdErrorCheck(void)
{
  if (!card.errorCode())
    return;
  while(1);
}

void setup()
{  
  pinMode(latchPin1, OUTPUT);
  pinMode(latchPin2, OUTPUT);
  
  //init arrays
  //array for power cell
  ledArray[0] = B00000000;
  ledArray[1] = B00000001;
  ledArray[2] = B00000011;
  ledArray[3] = B00000111;
  ledArray[4] = B00001111;
  ledArray[5] = B00011111;
  ledArray[6] = B00111111;
  ledArray[7] = B01111111;
  ledArray[8] = B11111111;
  
  //array for cyclotron
//  cycArray[0] = B00001000;
//  cycArray[1] = B00000100;
//  cycArray[2] = B00000010;
//  cycArray[3] = B00000001;
    
  //output pins for DAC
  pinMode(dacLCS, OUTPUT);
  pinMode(dacCLK, OUTPUT);
  pinMode(dacDI, OUTPUT);
  pinMode(dacLAT, OUTPUT);
  
  //pin for onboard LED
  pinMode(13, OUTPUT);
  
  //pin for thrower activation switch
  pinMode(THROWER_PIN, INPUT);
  
  //pin for equalizer mode switch
  pinMode(VIZ_PIN, INPUT);
  
  //pin for microphone input
  pinMode(MIC_PIN, INPUT);
  
  //enable pull-up resistors to minimize noise
  digitalWrite(THROWER_PIN, HIGH);
  digitalWrite(VIZ_PIN, HIGH);
  
  //output pins for shift registers
  pinMode(clockPin1, OUTPUT);
  pinMode(clockPin2, OUTPUT);
  pinMode(dataPin1, OUTPUT);
  pinMode(dataPin2, OUTPUT);
  
  if(!card.init())
  {
    sdErrorCheck();
    while(1);
  }
  card.partialBlockRead(true);
  uint8_t part;
  for(part = 0; part < 5; part++)
  {
    if(vol.init(card, part))
      break;
  }
  if(part == 5)
  {
    sdErrorCheck();
    while(1);
  }
  
  if(!root.openRoot(vol))
  {
    while(1);
  } 
  
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;
  
  adcInit();
  adcCalb();
}

SIGNAL(TIMER2_OVF_vect) {
  checkEqualizerState();
  /** Keep this line disabled
    * until a switch is actually wired
    * into A1 to keep electronic noise
    * from enabling equalizer mode
    **/
  if(equalizerMode == false)
  {
    checkThrower();
  }
  else if(wave.isplaying)
  {
    wave.stop();
  }
}

void loop()
{
  if(!equalizerMode)
  {
    cyclotron();
    powerCell();
    delay(1000/16);
  }
  else
  {
    calcViz();
    doFreqs();
  }
}

void powerCell()
{
  //determine which LEDs to light
  if(highPowerCell < 8)
  {
    ledOne = ledArray[highPowerCell];
    ledTwo = ledArray[0];
  }
  else
  {
    ledOne = ledArray[8];
    ledTwo = ledArray[highPowerCell - 8];
  }
  
  //write to the shift register
  digitalWrite(latchPin1, LOW);
  shiftOut(dataPin1, clockPin1, MSBFIRST, ledOne);
  shiftOut(dataPin1, clockPin1, MSBFIRST, ledTwo);
  digitalWrite(latchPin1, HIGH);
  
  //advance to next state
  if(highPowerCell >= 16)
  {
    if(throwing)
      highPowerCell = 8;
    else
      highPowerCell = 0;
  }
  else
    highPowerCell++;  
}

void cyclotron()
{
  //see if it's time to advance
  if(numCyclotronRounds == 0)
  {
    digitalWrite(latchPin2, LOW);
    shiftOut(dataPin2, clockPin2, MSBFIRST, ledThree);
    digitalWrite(latchPin2, HIGH);
  }
  
  //update the counter
  numCyclotronRounds++;
  if(numCyclotronRounds > 3)
  {
    numCyclotronRounds = 0;
    ledThree >>= 1;
    if(ledThree < B00000001)
      ledThree = CYCSTART;
  }
}

void doFreqs()
{
  //average frequencies
  uint16_t avg = 0;
  uint16_t avgBass = 0;
  uint16_t avgCymb = 0;
  
  byte bassLed = B00000000;
  byte cymbLed = B00000000;
  
  for(byte i = (FFT_N / 2) * (0); i < (FFT_N / 2) * (1); ++i)
  {
    if(i < (double)((FFT_N / 2) * (0.2)) && i > (double)((FFT_N / 2) * (0.1)))
      avgCymb += (uint16_t)spektrum[i];
    else if(i < (double)((FFT_N / 2) * (0.1)))
      avgBass += (uint16_t)spektrum[i];
    else
      avg += (uint16_t)spektrum[i];
  }
  
  avg /= (double)((FFT_N / 2) * (0.8));
  avg /= 4;
  avgBass /= (double)((FFT_N / 2) * (0.1));
  avgBass /= 12;
  avgCymb /= (double)((FFT_N / 2) * (0.1));
  avgCymb /= 12;
  
  //determine which LEDs to light
  if(avg < 8)
  {
    ledOne = ledArray[avg];
    ledTwo = ledArray[0];
  }
  else
  {
    ledOne = ledArray[8];
    ledTwo = ledArray[avg - 8];
  }
  
  /*if(avgBass > 8)
  {
    ledThree = B00001111;
  }
  else if(avgBass > 6)
  {
    ledThree = B00001100;
  }
  else
  {
    ledThree = B00000000;
  }*/
  
  if(avgBass > 14)
  {
    bassLed = B00001010;
  }
  else if(avgBass > 11)
  {
    bassLed = B00001000;
  }
  
  if(avgCymb > 6)
  {
    cymbLed = B00000101;
  }
  else if(avgCymb > 4)
  {
    cymbLed = B00000100;
  }
  
  ledThree = bassLed | cymbLed;
  
  //write to shift registers
  digitalWrite(latchPin1, LOW);
  shiftOut(dataPin1, clockPin1, MSBFIRST, ledOne);
  shiftOut(dataPin1, clockPin1, MSBFIRST, ledTwo);
  digitalWrite(latchPin1, HIGH);
  
  digitalWrite(latchPin2, LOW);
  shiftOut(dataPin2, clockPin2, MSBFIRST, ledThree);
  digitalWrite(latchPin2, HIGH);
}


//FFT calculations to get visualizer effects
void calcViz()
{
  if (position == FFT_N)
  {
    fft_input(capture, bfly_buff);
    fft_execute(bfly_buff);
    fft_output(bfly_buff, spektrum);
    position = 0;
  }
}

//checks switch state to enable equalizer
void checkEqualizerState()
{
  int reading;
  reading = digitalRead(A1);
  if(reading == LOW)
  {
    equalizerMode = true;
  }
  else
  {
    equalizerMode = false;
    if(ledThree == B00000000 || ledThree == B00001111)
    {
      ledThree = CYCSTART;
    }
  }
}

//checks switch state to enable thrower effects
void checkThrower()
{
  int reading;
  reading = digitalRead(THROWER_PIN);
  
  if(powerstart)
  {
    if(!wave.isplaying)
    {
      playfile("power_on.WAV");
      timeStart = millis();
    }
    if(millis() - timeStart >= 3680)
    {
      powerstart = false;
      wave.stop();
    }
  }
  else if(reading == LOW)
  {
    if(throwing)
    {
      if(!wave.isplaying)
      {
        throwerPlaying = false;
        throwerOn = false;
      }
      if(!throwerPlaying && !throwerOn)
      {
        timeStart = millis();
        playfile("stream.WAV");
        throwerPlaying = true;
      }
      if((millis() - timeStart) > 9500)
      {
        timeStart = millis();
        wave.seek(0);
      }
    }
    else
    {
      playfile("throw_on.WAV");
      throwerOn = true;
      throwing = true;
    }
  }
  else if(throwing)
  {
    playfile("pow_off.WAV");
    throwerPlaying = false;
    throwing  = false;
  }
  else
  {
    if(!powerstart && !wave.isplaying)
    {
      timeStart = millis();
      playfile("hum.WAV");
    }
    else if((millis() - timeStart) > 2900)
    {
      timeStart = millis();
      wave.seek(0);
    }
  }
}

void playfile(char *name)
{
  if(wave.isplaying)
  {
    wave.stop();
  }
  if(!f.open(root, name))
  {
    return;
  }
  if(!wave.create(f))
  {
    return;
  }
  wave.play();
}

ISR(ADC_vect)
{
  if (position >= FFT_N)
    return;
  
  capture[position] = ADC + zero;
  if (capture[position] == -1 || capture[position] == 1)
    capture[position] = 0;

  position++;
}

void adcInit()
{
  /*  REFS0 : VCC use as a ref, IR_AUDIO : channel selection, ADEN : ADC Enable, ADSC : ADC Start, ADATE : ADC Auto Trigger Enable, ADIE : ADC Interrupt Enable,  ADPS : ADC Prescaler  */
  // free running ADC mode, f = ( 16MHz / prescaler ) / 13 cycles per conversion 
  ADMUX = _BV(REFS0) | IR_AUDIO; // | _BV(ADLAR); 
//  ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) //prescaler 64 : 19231 Hz - 300Hz per 64 divisions
  ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // prescaler 128 : 9615 Hz - 150 Hz per 64 divisions, better for most music
  sei();
}

void adcCalb()
{
  long midl = 0;
  // get 2 measurement at 2 sec
  // on ADC input must be NO SIGNAL!!!
  for (byte i = 0; i < 2; i++)
  {
    position = 0;
    delay(100);
    midl += capture[0];
    delay(900);
  }
  zero = -midl/2;
}
