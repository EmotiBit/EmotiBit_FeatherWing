int pin = 19; 
void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  pinMode(pin,OUTPUT);
  digitalWrite(pin,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  int i=0;
  digitalWrite(pin,HIGH);
  for(i=0;i<=30;i++)
  {
    Serial.println(i);
    delay(500);
  }
  digitalWrite(pin,LOW);
  delay(500);
  digitalWrite(pin,HIGH);
  delay(500);
  digitalWrite(pin,LOW);
  delay(500);
  digitalWrite(pin,HIGH);
  delay(500);
  digitalWrite(pin,LOW);
}
