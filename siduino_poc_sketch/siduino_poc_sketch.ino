#include "Wire.h"
#include <Adafruit_MCP23017.h>

// MCP setup
Adafruit_MCP23017 mcp;

// Interrupts from the MCP will be handled by this PIN
byte arduinoIntPin = 2;

// ... and this interrupt vector
//byte arduinoInterrupt = 1;

volatile boolean awakenByInterrupt = false;

/*
    Test the 74LS244 3 bit to 8 line binary decoder
    Assumes G2A and G2B lines are hard wired LOW

    G1 LOW  = All Yi HIGH
    G1 HIGH = Yi according to table below
      ---------
      C B A   i - Yi goes LOW, all others stay HIGH
      ---------
      0 0 0 | 0
      0 0 1 | 1
      0 1 0 | 2
      0 1 1 | 3
      1 0 0 | 4
      1 0 1 | 5
      1 1 0 | 6
      1 1 1 | 7
      ---------
*/
// Pins connected to 74LS244 chip select decoder
int G1 = 13;

// 74HC595 Shift Register to load address bus
int addr_DATA  = 4; // WHITE wire
int addr_CLK   = 5; // YELLOW
int addr_LATCH = 6; // RED

// 74HC595 Shift Register to load data bus
int data_DATA  = 7; // ORANGE wire
int data_CLK   = 8; // YELLOW
int data_LATCH = 9; // RED


// Analog bus for reading pots
int ANALOG_BUS = A0;

// Data bus
int data[8] = {3, 4, 5, 6, 7, 8, 9, 10};

unsigned int MCP23017_GPIOAB_data = 0xffff;

// Pin for 1MHz clock to 6581
const int clkOutputPin = 11;   // Digital pin 11 = MOSI/OC2A/PCINT3) for ATmega328 boards
const int ocr2aval  = 7; // Set to 7 for 1MHz, 3 for 2MHz

// Location of SID in memory map
const int SID_BASE = 0x0000;
const int SID_VOICE_1 = SID_BASE + 0x00;
const int SID_VOICE_2 = SID_BASE + 0x07;
const int SID_VOICE_3 = SID_BASE + 0x0e;
const int SID_FILTER  = SID_BASE + 0x15;
const int SID_MISC    = SID_BASE + 0x19;

// The following are per-voice offsets from SID_VOICE_x
const int SID_FREQ_LO = 0x00;
const int SID_FREQ_HI = 0x01;
const int SID_PW_LO   = 0x02;
const int SID_PW_HI   = 0x03; // Bottom 4 bits only
const int SID_CTRL    = 0x04;
const int SID_AD      = 0x05;
const int SID_SR      = 0x06;

void setup() {
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
  Serial.begin(9600);

  // Setup clouck output pin to drive SID chip
  pinMode(clkOutputPin, OUTPUT);
  // Set Timer 2 CTC mode with no prescaling.  OC2A toggles on compare match
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

  pinMode(arduinoIntPin, INPUT);

  mcp.begin();
  mcp.setupInterrupts(true, false, LOW);


  for (int i = 0; i < 16; i++) {
    mcp.pinMode(i, INPUT);
    mcp.pullUp(i, HIGH);  // turn on a 100K pullup internally
    mcp.setupInterruptPin(i, CHANGE);
  }

  attachInterrupt(digitalPinToInterrupt(arduinoIntPin), intCallBack, FALLING);
}

/*
  byte data_read() {
  byte d = 0;

  for (int i = 7; i >= 0; i--) {
    d <<= 1;
    d |= digitalRead(data[i]);
  }
  return d;
  }
*/

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



// The int handler will just signal that the int has happen
// we will do the work from the main loop.
void intCallBack() {
  awakenByInterrupt = true;
}

void handleInterrupt() {

  // Get more information from the MCP from the INT
  uint8_t pin = mcp.getLastInterruptPin();
  uint8_t val = mcp.getLastInterruptPinValue();

  Serial.print("The change was ");
  Serial.print("Pin: "); Serial.print(pin, HEX); Serial.println();
  Serial.print("Val: "); Serial.print(val, HEX); Serial.println();

  MCP23017_GPIOAB_data = mcp.readGPIOAB();

  cleanInterrupts();
}

// handy for interrupts triggered by buttons
// normally signal a few due to bouncing issues
void cleanInterrupts() {
  EIFR = 0x01;
  awakenByInterrupt = false;
}

void poke(int reg, int value) {
  digitalWrite(G1, LOW);
  //Serial.println("writing address");
  address_write(reg);
  //Serial.println("writing data");
  data_write(value);
  //Serial.println("chip enable");
  digitalWrite(G1, HIGH);
  delay(1);
  //Serial.println("chip disable");
  digitalWrite(G1, LOW);
}

void loop() {

  /*
    // 4051 analog multiplexer device 2 on chip select circuit
    byte base = 0x40; // 0b0100 0000
    for (byte i = 0; i < 8; i++) {
      address_write(base + i);
      digitalWrite(G1, HIGH);
      int pot = analogRead(A0);
      digitalWrite(G1, LOW);
      Serial.println(pot);
      delay(50);
    }

    if (awakenByInterrupt) {
      detachInterrupt(digitalPinToInterrupt(arduinoIntPin));
      Serial.println("IRQ!");
      handleInterrupt();
      attachInterrupt(digitalPinToInterrupt(arduinoIntPin), intCallBack, FALLING);
    }

    Serial.println();
    Serial.println(MCP23017_GPIOAB_data, HEX);
    Serial.println();
  */

  int chord[3] = {0x1125, 0x159a, 0x1981};

  poke(SID_FILTER + 0x03, 0x0f); // max volume & voice 3 off

  // Make a noise
  for (int i = 0; i < 3; i++) {
    Serial.println("Writing to SID...BEEP");

    poke(SID_VOICE_1 + SID_FREQ_LO, lowByte(chord[i]));
    poke(SID_VOICE_1 + SID_FREQ_HI, highByte(chord[i]));
    poke(SID_VOICE_1 + SID_AD, 0x0f);
    poke(SID_VOICE_1 + SID_SR, 0xf0);
    poke(SID_VOICE_1 + SID_CTRL, 0b00100001);
    Serial.println("complete");
    delay(1000);
    Serial.println("Writing to SID...OFF");
    poke(SID_VOICE_1 + SID_CTRL, 0b00000000);
    Serial.println("complete");
    delay(500);
  }
}
