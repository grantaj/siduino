
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

byte commandByte;
byte noteByte;
byte velocityByte;
int row = 0;

bool note = false; // MIDI note on=true, off=false

void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.clear();

  Serial.begin(31250);

  cli();//stop interrupts

  //set timer2 interrupt every 128us
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 7.8khz increments
  OCR2A = 255;// = (16*10^6) / (7812.5*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS11 bit for 8 prescaler
  TCCR2B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();//allow interrupts

}


ISR(TIMER2_COMPA_vect) {//checks for incoming midi every 128us
cli();

  switch (Serial.available()) {
    case 1:
      commandByte = Serial.read();
      lcd.setCursor(0, row); row += 1; row %= 2;
      lcd.print(commandByte, HEX); lcd.print(" ");
      break;

    case 2:
      noteByte = Serial.read();//read next byte
      velocityByte = Serial.read();//read final byte
      lcd.setCursor(0, row); row += 1; row %= 2;
      lcd.print(noteByte, HEX); lcd.print(" ");
      lcd.print(velocityByte, HEX);
      break;

    case 3:
      commandByte = Serial.read();
      noteByte = Serial.read();//read next byte
      velocityByte = Serial.read();//read final byte
      lcd.setCursor(0, row); row += 1; row %= 2;
      lcd.print(commandByte, HEX); lcd.print(" ");
      lcd.print(noteByte, HEX); lcd.print(" ");
      lcd.print(velocityByte, HEX);
      break;

    default:
      break;
  }

sei();

}

void loop() {
  //do whatever here
}
