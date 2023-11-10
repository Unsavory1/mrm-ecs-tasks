const int numberofBits=3; 
const int lowestBit=7; 
int btt=4;
int bttprevState=LOW; 
int bttcurrState=LOW;
int led1=9;
int led2=7;
int led3=8; 
int bits[numberofBits];

void setup()
{ 
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
  pinMode(led3,OUTPUT); 
  pinMode(btt,INPUT_PULLUP);
} 
void loop()
{ 
  bttcurrState = digitalRead(btt);
  if(bttcurrState == HIGH && bttcurrState != bttprevState)
  {
    addOne();
    displayBits();
  }
  bttprevState=bttcurrState;
}
void addOne()
{
  int carry=1; 
  for(int i=0; i<=numberofBits;i++)
  {
    carry+=bits[i];
    bits[i]=carry%2;
    carry/=2;
  }
}
  void displayBits() 
  { 
for(int i=0;i<=numberofBits;i++)
{ 
  digitalWrite(lowestBit + i,bits[i]);
}
  }
