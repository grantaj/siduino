//
// Use of timer2 to generate a signal for a particular frequency on pin 11
//
// davekw7x
// https://forum.arduino.cc/index.php?topic=62964.0

const int clkOutputPin = 11;   // Digital pin 11 = MOSI/OC2A/PCINT3) for ATmega328 boards
//const int freqOutputPin = 10; // OC2A output for Mega boards

const int clkOutputPin2 = 12; // For busy loop clock

// Constants are computed at compile time

// If you change the prescale value, it affects CS22, CS21, and CS20
// For a given prescale value, the eight-bit number that you
// load into OCR2A determines the frequency according to the
// following formulas:
//
// With no prescaling, an ocr2val of 3 causes the output pin to
// toggle the value every four CPU clock cycles. That is, the
// period is equal to eight slock cycles.
//
// With F_CPU = 16 MHz, the result is 2 MHz.
//
// Note that the prescale value is just for printing; changing it here
// does not change the clock division ratio for the timer!  To change
// the timer prescale division, use different bits for CS22:0 below
const int prescale  = 1;
const int ocr2aval  = 7; // Set to 7 for 1MHz, 3 for 2MHz
// The following are scaled for convenient printing
//

// Period in microseconds
const float period    = 2.0 * prescale * (ocr2aval+1) / (F_CPU/1.0e6);

// Frequency in Hz
const float freq      = 1.0e6 / period;

void setup()
{
   pinMode(clkOutputPin, OUTPUT);
   pinMode(clkOutputPin2, OUTPUT);
   pinMode(LED_BUILTIN, OUTPUT);
   
   Serial.begin(9600);

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

   Serial.print("Period    = ");
   Serial.print(period);
   Serial.println(" microseconds");
   Serial.print("Frequency = ");
   Serial.print(freq);
   Serial.println(" Hz");
}




void loop() {
  // Busy loop clock output
  digitalWrite(clkOutputPin2, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);  
  delay(50);                    
  digitalWrite(clkOutputPin2, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);                      
}
