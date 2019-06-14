int pin = 13; 
void setup() {
  // put your setup code here, to run once:
  
 
  pinMode(pin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
int i=0;
digitalWrite(pin,LOW);
delay(2000);
digitalWrite(pin,HIGH);
delay(2000);
digitalWrite(pin,LOW);
delay(2000);
digitalWrite(pin,HIGH);
delay(2000);
digitalWrite(pin,LOW);
  for(i=0;i<5;i++)
  {
    digitalWrite(pin,LOW);
    delay(500);
    digitalWrite(pin,HIGH);
    delay(500);
  }
}
