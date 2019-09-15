#include "Wire.h"
#include <Adafruit_MCP23017.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// MCP setup
Adafruit_MCP23017 mcp;

// OLED Setup
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// Interrupts from the MCP will be handled by this PIN
//byte arduinoIntPin = 2;

// ... and this interrupt vector
//byte arduinoInterrupt = 1;


//volatile boolean awakenByInterrupt = false; // Pin state change on MCP
volatile boolean periodicInterrupt = false; // Timer interrupt for periodic polling
volatile int periodicInterruptCount = 0; // Counts how many interrupts have occured

// Pins connected to 74LS244 chip select decoder
const int G1  = 13;
const int LED = 12;

// 74HC595 Shift Register to load address bus
const int addr_DATA  = 4; // WHITE wire
const int addr_CLK   = 5; // YELLOW
const int addr_LATCH = 6; // RED

// 74HC595 Shift Register to load data bus
const int data_DATA  = 7; // ORANGE wire
const int data_CLK   = 8; // YELLOW
const int data_LATCH = 9; // RED


// Analog bus for reading pots
const int ANALOG_BUS = A0;

// Data bus
const int data[8] = {3, 4, 5, 6, 7, 8, 9, 10};

unsigned int MCP23017_GPIOAB_data = 0x0000;

// Pin for 1MHz clock to 6581
const int clkOutputPin = 11;   // Digital pin 11 = MOSI/OC2A/PCINT3) for ATmega328 boards
const int ocr2aval  = 7; // Set to 7 for 1MHz, 3 for 2MHz

// Location of SID in memory map
const int SID_BASE    = 0x0000;

// Offsets to the SID registers
const int SID_VOICE_1 = 0x00;
const int SID_VOICE_2 = 0x07;
const int SID_VOICE_3 = 0x0e;
const int SID_FILTER  = 0x15;
const int SID_MISC    = 0x19;

// The following are per-voice offsets from SID_VOICE_x
const int SID_FREQ_LO = 0x00;
const int SID_FREQ_HI = 0x01;
const int SID_PW_LO   = 0x02;
const int SID_PW_HI   = 0x03; // Bottom 4 bits only
const int SID_CTRL    = 0x04;
const int SID_AD      = 0x05;
const int SID_SR      = 0x06;

// In memory copies of the SID registers
// We need these, as the SID write registers are write only
byte SID_register[25];

const int noteFreq[95] = {
  //c     c#      d       d#      e       f       f#      g       g#      a       a#      b
  0x0112, 0x0123, 0x0134, 0x0146, 0x015a, 0x016e, 0x0184, 0x018b, 0x01b3, 0x01cd, 0x01e9, 0x0206,
  0x0225, 0x0245, 0x0268, 0x028c, 0x02b3, 0x02dc, 0x0308, 0x0336, 0x0367, 0x039b, 0x03d2, 0x040c,
  0x0449, 0x048b, 0x04d0, 0x0519, 0x0567, 0x05b9, 0x0610, 0x066c, 0x06ce, 0x0735, 0x07a3, 0x0817,
  0x0893, 0x0915, 0x099f, 0x0a32, 0x0acd, 0x0b72, 0x0c20, 0x0c9c, 0x0d9c, 0x0e6b, 0x0f46, 0x102f,
  0x1125, 0x122a, 0x133f, 0x1464, 0x159a, 0x16e3, 0x183f, 0x1981, 0x1b38, 0x1cd6, 0x1e80, 0x205e,
  0x224b, 0x2455, 0x267e, 0x28c8, 0x2b34, 0x2dc6, 0x307f, 0x3361, 0x366f, 0x39ac, 0x3d1a, 0x40bc,
  0x4495, 0x48a9, 0x4cfc, 0x518f, 0x5669, 0x5b8c, 0x60fe, 0x6602, 0x6cdf, 0x7358, 0x7a34, 0x8178,
  0x892b, 0x9153, 0x99f7, 0xa31f, 0xacd2, 0xb719, 0xc1fc, 0xcd85, 0xd98d, 0xe6b0, 0xf467
};

void setup() {
  // ----------------------------------------------------------------------
  // Initialise screen
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3D (for the 128x64)
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Siduino");
  display.display();

  // ----------------------------------------------------------------------
  // I/O setup
  digitalWrite(G1, LOW);
  pinMode(G1, OUTPUT);
  pinMode(addr_DATA, OUTPUT);
  pinMode(addr_CLK, OUTPUT);
  digitalWrite(addr_LATCH, LOW);
  pinMode(addr_LATCH, OUTPUT);
  pinMode(data_DATA, OUTPUT);
  pinMode(data_CLK, OUTPUT);
  digitalWrite(data_LATCH, LOW);
  pinMode(data_LATCH, OUTPUT);

  // Start data bus in write mode
  for (int i = 0; i < 8; i++) {
    pinMode(data[i], OUTPUT);
  }

  // Initialise serial
  Serial.begin(31250);
  // Flush MIDI
  while (Serial.available()) {
    Serial.read();
  }

  // Setup clock output pin to drive SID chip
  pinMode(clkOutputPin, OUTPUT);

  // ----------------------------------------------------------------------
  // TIMER1 for polling analog input surface

  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 1250;            // compare match register 1 sec = 16MHz/256 = 62,500 Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  pinMode(12, OUTPUT);
  interrupts();             // enable all interrupts

  // ----------------------------------------------------------------------
  // TIMER2 for generating 1MHz clock for 6581 on digital pin 11 (OCR2A)
  // CTC mode with no prescaling.  OC2A toggles on compare match
  //
  // WGM22:0 = 010: CTC Mode, toggle OC
  // WGM2 bits 1 and 0 are in TCCR2A,
  // WGM2 bit 2 is in TCCR2B
  // COM2A0 sets OC2A (arduino pin 11 on Uno or Duemilanove) to toggle on compare match
  //
  TCCR2A = ((1 << WGM21) | (1 << COM2A0));

  // Set Timer 2  No prescaling  (i.e. prescale division = 1)
  //
  // CS22:0 = 001: Use CPU clock with no prescaling
  // CS2 bits 2:0 are all in TCCR2B
  TCCR2B = (1 << CS20);

  // Make sure Compare-match register A interrupt for timer2 is disabled
  TIMSK2 = 0;
  // This value determines the output frequency
  OCR2A = ocr2aval;


  // ----------------------------------------------------------------------
  // MCP23017 setup
  // This will be used to control the SID_CTRL registers
  //pinMode(arduinoIntPin, INPUT);

  mcp.begin();
  //mcp.setupInterrupts(true, false, LOW);

  for (int i = 0; i < 16; i++) {
    mcp.pinMode(i, INPUT);
    mcp.pullUp(i, HIGH);  // turn on a 100K pullup internally
    //mcp.setupInterruptPin(i, CHANGE);
  }

  //MCP23017_GPIOAB_data = mcp.readGPIOAB();

  //attachInterrupt(digitalPinToInterrupt(arduinoIntPin), intCallBack, FALLING);

  // ----------------------------------------------------------------------
  // Set up iitial SID voices
  sidInit();

  //sidRegisterWrite(SID_VOICE_1 + SID_CTRL, 0b01000000); // Initialise to PW
  updateCtrlParams();

  // These should be switched
  sidRegisterWrite(SID_FILTER + 0x02, 0b00000001); // RES | EXT V3 V2 V1
  sidRegisterWrite(SID_FILTER + 0x03, 0b00011111); // 3OFF HP BP LP | Volume

  updateFilterParams();
  updateVoiceParams();

  noteOff(SID_VOICE_1); //just in case something is wierd

  //setADSR(SID_VOICE_1, 0x0, 0x8, 0xa, 0x5);

  statusDisplay();
}

ISR(TIMER1_COMPA_vect) {
  //digitalWrite(LED, digitalRead(LED) ^ 1);   // toggle LED pin
  periodicInterrupt = true;
}

void handlePeriodicInterrupt() {

  periodicInterruptCount++;
  // Fast updates
  updateFilterParams();

  // Slower updates
  // probably not altering these so much in real time
  if (!(periodicInterruptCount % 20)) {
    updateVoiceParams();
    updateCtrlParams();
  }

  periodicInterrupt = false;
}

void updateFilterParams()
{
  // ADC is 10 bits
  setFC(readPot(0) << 1);  // Cutoff frequency is 11 bits
  setRES(readPot(1) >> 6); // Resonance is 4 bits
  setVOL(readPot(2) >> 6); // Volume is 4 bits
}

void updateVoiceParams() {
  // No voice parameter for now, as we are working only with voice 1
  // Each ADSR parameter is 4 bits
  setADSR(SID_VOICE_1, readPot(4) >> 6, readPot(5) >> 6, readPot(6) >> 6, readPot(7) >> 6);
  // PW is 12 bits
  setPW(SID_VOICE_1, readPot(3) << 2);
}

void updateCtrlParams() {
  // Don't mess with GATE
  int c = mcp.readGPIOAB();
  byte gate = sidRegisterRead(SID_VOICE_1 + SID_CTRL) & 1;
  setCTRL(SID_VOICE_1, (c & 0b11111110) | gate);
}


void statusDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);

  // Display cutoff freq as one binary string high byte and low 3 bits
  //display.print(sidRegisterRead(SID_FILTER + 0x01), BIN); display.println(sidRegisterRead(SID_FILTER + 0x00), BIN);

  //display.println(sidRegisterRead(SID_FILTER + 0x02), BIN);
  //display.println(sidRegisterRead(SID_FILTER + 0x03), BIN);

  display.println(sidRegisterRead(SID_VOICE_1 + SID_AD), HEX);
  display.println(sidRegisterRead(SID_VOICE_1 + SID_SR), HEX);
  display.println(sidRegisterRead(SID_VOICE_1 + SID_CTRL), BIN);
  display.display();
}

void data_write(byte d) {
  digitalWrite(data_LATCH, LOW);
  shiftOut(data_DATA, data_CLK, MSBFIRST, d);
  digitalWrite(data_LATCH, HIGH);
}

/*
   Write an address onto the address bus
   This is done by shifting it out usign a shift register
*/
void address_write(byte a) {
  digitalWrite(addr_LATCH, LOW);
  shiftOut(addr_DATA, addr_CLK, MSBFIRST, a);
  digitalWrite(addr_LATCH, HIGH);
}


/*
  // The int handler will just signal that the int has happen
  // we will do the work from the main loop.
  void intCallBack() {
  awakenByInterrupt = true;
  }


  void handleInterrupt() {

  // Get more information from the MCP from the INT
  //uint8_t pin = mcp.getLastInterruptPin();
  //uint8_t val = mcp.getLastInterruptPinValue();

  //Serial.print("The change was ");
  //Serial.print("Pin: "); Serial.print(pin, HEX); Serial.println();
  //Serial.print("Val: "); Serial.print(val, HEX); Serial.println();

  //irqcount++;
  MCP23017_GPIOAB_data = mcp.readGPIOAB();
  //updateCtrlParams();

  //statusDisplay();

  //cleanInterrupts();
  }

  // handy for interrupts triggered by buttons
  // normally signal a few due to bouncing issues
  void cleanInterrupts() {
  EIFR = 0x01;
  awakenByInterrupt = false;
  }
*/
/*
   Generic poke function to write value on to the data bus
   and reg obnto the address bus.
*/
void poke(int reg, int value) {
  digitalWrite(G1, LOW);
  address_write(reg);
  data_write(value);
  digitalWrite(G1, HIGH);
  delay(1);
  digitalWrite(G1, LOW);
}

// SID chip functions -------------------------------

// Use sidWriteRegister and sidReadRegister to access
// SID chip registers. This keeps local copy of the
// registers up to date. Direct poking will cause the
// local copy to get out of date so don't do it.
// The registers are indexed from 0x00, i.e. the bottom
// of the SID chip memory map.

byte sidRegisterRead(int r) {
  return SID_register[r];
}

void sidRegisterWrite(int r, byte b) {
  SID_register[r] = b;
  poke(SID_BASE + r, b); // This is the only place that SID_BASE should be used.

}

void sidInit() {
  for (int i = 0; i < 24; i++) {
    sidRegisterWrite(i, 0x00);
  }
}

void noteOn(int v, int f) {
  setFreq(v, f);
  setGate(v);
}

void setFreq(int v, int f) {
  sidRegisterWrite(v + SID_FREQ_LO, lowByte(f));
  sidRegisterWrite(v + SID_FREQ_HI, highByte(f));
}

void setGate(int v) {
  byte ctrl = sidRegisterRead(v + SID_CTRL);
  sidRegisterWrite(v + SID_CTRL, ctrl | 0b00000001);
}

void noteOff(int v) {
  byte ctrl = sidRegisterRead(v + SID_CTRL);
  sidRegisterWrite(v + SID_CTRL, ctrl & 0b11111110);
}

void setCTRL(int v, byte c) {
  // Note will also change status of GATE
  // Caller needs to manage GATE
  sidRegisterWrite(v + SID_CTRL, c);
}

void setADSR(int v, byte a, byte d, byte s, byte r) {
  a &= 0xf;
  d &= 0xf;
  s &= 0xf;
  r &= 0xf;

  sidRegisterWrite(v + SID_AD, (a << 4) | d);
  sidRegisterWrite(v + SID_SR, (s << 4) | r);
}

void setPW(int v, int pw) {
  sidRegisterWrite(v + SID_PW_LO, pw & 0xfff);
  sidRegisterWrite(v + SID_PW_HI, (pw >> 8) & 0xf);
}

int readPot(int i) {
  byte base = 0x40; // 0b0100 0000
  address_write(base + i);
  digitalWrite(G1, HIGH);
  int pot = analogRead(A0);
  delayMicroseconds(10);
  digitalWrite(G1, LOW);
  return pot;
}

void setFC(int FC) {
  sidRegisterWrite(SID_FILTER + 0x00, FC & 0b111);
  sidRegisterWrite(SID_FILTER + 0x01, FC >> 3);
}

void setRES(int RES) {
  RES = RES & 0b1111;
  byte r = sidRegisterRead(SID_FILTER + 0x02);
  sidRegisterWrite(SID_FILTER + 0x02, (RES << 4) | (r & 0b1111));
}

void setVOL(int VOL) {
  byte r = sidRegisterRead(SID_FILTER + 0x03);
  sidRegisterWrite(SID_FILTER + 0x03, (r & 0b11110000) | (VOL & 0b1111));
}



// ---------------------------------------------------------------------------------
// Channel voice messages
// MIDI commnds are the high nibble. Low nibble is the midi channel number
#define MIDIcvMask      0b1         // All commands in this category start with this
#define MIDInoteOff     0b10000000  // Note on
#define MIDInoteOn      0b10010000  // Note off
#define MIDIpkp         0b10100000  // Polyphonic key pressure
#define MIDIcc          0b10110000  // Control change
#define MIDIpc          0b11000000  // Program change
#define MIDIcp          0b11010000  // Channel pressure
#define MIDIpbc         0b11100000  // Pitch bend change

// Channel mode messages
#define MIDIcmMask      0b1111      // All commands in this category start with this
#define MIDIsysex       0b11110000  // System exclusive
#define MIDItcqf        0b11110001  // Time code quarter frame
#define MIDIspp         0b11110010  // Song position pointer
#define MIDIss          0b11110011  // Song select
#define MIDIundef1      0b11110100  // undefined
#define MIDIundef2      0b11110101  // undefined
#define MIDItunereq     0b11110110  // Tune request
#define MIDIsysexEnd    0b11110111  // System exclusive end of transmnission

// System real time messages
#define MIDIsrtMask     0b11111     // All commands in this category start with this
#define MIDIclock       0b11111000  // Timing clock sent 24 times per quarter note
#define MIDIundef3      0b11111001  // undefined
#define MIDIstart       0b11111010  // Start
#define MIDIcontinue    0b11111011  // Continue
#define MIDIstop        0b11111100  // Stop
#define MIDIundef4      0b11111101  // undefined
#define MIDIas          0b11111110  // Active sensing
#define MIDIreset       0b11111111  // Reset

const byte MIDIparamLength[3][8] = {
  {2, 2, 2, 2, 1, 1, 2, 0},
  {0, 1, 2, 1, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};

enum MIDImessageType {MIDIcv, MIDIcm, MIDIsrt};

byte commandByte;
byte midiBuffer[2];
int midiBufferIndex = 0;

int midiMessageType = -1; // -1 means invalid
int midiParamLength = -1;
int midiChannel     = -1;

byte currentNoteByte = 0x00; // We will use note zero as a "no note" value
byte oldNoteByte     = 0x00;


void handleMIDI() {

  byte noteByte;
  byte velocityByte;

  /*
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("MIDI: ");
    display.print(commandByte, HEX); display.print(" ");
    for (int i = 0; i < midiParamLength; i++) {
      display.print(midiBuffer[i], HEX); display.print(" ");
    }
    display.display();
  */

  if (commandByte >> 4 == MIDInoteOn >> 4) { // NOTE ON

    noteByte     = midiBuffer[0];
    velocityByte = midiBuffer[1];

    if ((noteByte != currentNoteByte) && (noteByte != oldNoteByte)) {
      // Only push if the note is note already playing
      oldNoteByte = currentNoteByte; // push old note onto stack
      currentNoteByte = noteByte;    // This is the new note
    }

    // (re)trigger note
    noteOn(SID_VOICE_1, noteFreq[currentNoteByte]);

  } else if (commandByte >> 4 == MIDInoteOff >> 4) { // NOTE OFF

    noteByte     = midiBuffer[0];
    velocityByte = midiBuffer[1];

    if (noteByte == currentNoteByte) {
      currentNoteByte = oldNoteByte; // pop old note off stack
    }
    if (noteByte = oldNoteByte) { // delete old note from stack
      oldNoteByte = 0x00;

    }

    if (currentNoteByte) { // There was an old note still playing. Reactivate it.
      noteOn(SID_VOICE_1, noteFreq[currentNoteByte]);
    } else { // There was nothing on the stack, so turn voice off
      noteOff(SID_VOICE_1);
      currentNoteByte = 0;
    }
  } else if ((commandByte >> 4 == MIDIpbc >> 4) && currentNoteByte) { // PITCH BEND (only if a note is playing)

    int pb = ((midiBuffer[1] << 7) | midiBuffer[0]);
    int currentFreq = noteFreq[currentNoteByte];
    int wholeTone;
    float fracTone = ((float)pb - 8192.0) / 8192.0; // [-1, +1]

    if (fracTone > 0) { // bend up
      wholeTone = noteFreq[currentNoteByte + 2] - currentFreq;
    } else if (fracTone < 0) { // bend down
      wholeTone = currentFreq - noteFreq[currentNoteByte - 2]; 
    }

    setFreq(SID_VOICE_1, round(currentFreq + (float)wholeTone * fracTone));

  } else { // catch all

  }
}

void loop() {


  if (periodicInterrupt) {
    //handlePeriodicInterrupt();
  }

  if (Serial.available() > 0) {
    byte incomingByte = Serial.read(); // read next byte

    if (incomingByte >> 7) { // New command received

      commandByte = incomingByte;
      midiBufferIndex = 0;
      // Check command category
      if (commandByte >> 3 == MIDIsrtMask) {        // System real time command

        midiMessageType = MIDIsrt;
        midiParamLength = MIDIparamLength[midiMessageType][commandByte & 0b111];

      } else if (commandByte >> 4 == MIDIcmMask) {  // Channel mode command

        midiMessageType = MIDIcm;
        midiParamLength = MIDIparamLength[midiMessageType][commandByte & 0b111];

      } else {                                      // Channel voice command

        midiMessageType = MIDIcv;
        midiChannel     = commandByte & 0b1111;
        midiParamLength = MIDIparamLength[midiMessageType][(commandByte >> 4) & 0b111];

      }

      if (MIDIparamLength == 0) {
        handleMIDI();
      }

    } else if (commandByte != MIDIsysex) { // Parameter data, skipping system exclusive

      if (midiBufferIndex < 2) { // Only handling 1 and 2 byte data, i.e. no System Exclusive messages
        midiBuffer[midiBufferIndex++] = incomingByte; // Parameter data
      }

      // Check if we have a complete message
      if (midiParamLength == midiBufferIndex) {
        handleMIDI();
        midiBufferIndex = 0;
      }

    }


  }

}
