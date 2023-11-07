// C++ code
//
int led=9;
int brightness=0; 
int fadeAmt=5;
void setup()
{
  pinMode(led, OUTPUT);
  
}

void loop()
{
  analogWrite(led, brightness);
  brightness=brightness+fadeAmt; 
  if(brightness<=0 || brightness>=255) 
  {
    fadeAmt=-fadeAmt; 
  } 
  	
  delay(5);
  
}