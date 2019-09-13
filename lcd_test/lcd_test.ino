/*
   LCD driver for HD44780
*/
const int D4 = 4; // 4-bit interface
const int D5 = 5;
const int D6 = 6;
const int D7 = 7;
const int RS = 8; // Register select (0=command, 1=data)
const int E  = 9; // Enable

const int lcdData[4] = {D4, D5, D6, D7};

void setup() {
  // put your setup code here, to run once:

  pinMode(D4, OUTPUT); // lowest bit
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT); // highest bit
  pinMode(RS, OUTPUT);
  pinMode(E, OUTPUT);

  digitalWrite(E, HIGH);

  commandMode();
  // Ensure 4-bit mode
  putNibble(0b0011);
  putNibble(0b0011);
  putNibble(0b0011);

  putNibble(0b0010);

  // Two line mode
  putByte(0b00101000);
  
  Serial.begin(9600);
}

byte lowNibble(byte b) {
  return b & 0b1111;
}

byte highNibble(byte b) {
  return b >> 4;
}

void putNibble(byte b) {
  Serial.print("Writing nibble: ");
  Serial.println(b, HEX);

  for (int i = 0; i < 4; i++) {
    digitalWrite(lcdData[i], b & 1);
    b >>= 1;
  }
  toggleEnable();
}

void putByte(byte b) {
  putNibble(highNibble(b));
  putNibble(lowNibble(b));

}

void toggleEnable() {
  digitalWrite(E, LOW);
  delay(2);
  digitalWrite(E, HIGH);
}

void commandMode() {
  digitalWrite(RS, LOW);
}
void dataMode() {
  digitalWrite(RS, HIGH);
}

void clearScreen() {
  commandMode();
  putByte(0b00000001);
}

void cursorHome() {
  commandMode();
  putByte(0b00000010);
}

void writeChar(char x) {
  dataMode();
  putByte(x);
}

void writeString(char* x) {
  int i=0;
  while (x[i]) writeChar(x[i++]); 
}

void loop() {
  //cursorHome();
  clearScreen();
  writeString("Hello");
  delay(1000);

}
