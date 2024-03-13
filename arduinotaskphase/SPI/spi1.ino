#include <SPI.h>
const int LEDpin = 3;
const int buttonpin = 2;
volatile byte Slavereceived, Slavesend;

int buttonvalue;  
bool received=1;
bool send;
bool ledst;



void setup() {
  Serial.begin(9600);
  pinMode(buttonpin, INPUT_PULLUP); 
  pinMode(LEDpin, OUTPUT);          
  pinMode(MISO, OUTPUT); 
  pinMode(MOSI,INPUT); 
  pinMode(SS,INPUT);          
  SPCR |= _BV(SPE);
  
  SPI.attachInterrupt();
  SPDR=255;  

}


ISR(SPI_STC_vect) 
{
  Slavereceived = SPDR; 

  
  if (send)
    SPDR=255;
  else SPDR=0;
  
}

void loop() {

 send=digitalRead(buttonpin); 
  
    ledst=Slavereceived&128;
    digitalWrite(LEDpin,ledst);
    Serial.println(received); 
}
