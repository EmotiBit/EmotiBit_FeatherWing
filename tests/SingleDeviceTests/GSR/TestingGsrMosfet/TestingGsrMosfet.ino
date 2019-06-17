/// NEW GSR
int vlow = 5;
int vhigh = 28;
float vl=0,vh=0;
float sumvl=0,sumvh=0;
int Navg=64;

int MOS = 27;//gpio pin assigned ot the mosfet
void setup(){
  Serial.begin(9600);
  Serial.println("starting GSR Setup");
  pinMode(vlow,INPUT);
  pinMode(vhigh,INPUT);
  pinMode(MOS,OUTPUT);
  digitalWrite(MOS,LOW);// Switch is ON. hence, The GSR is powered
}
void loop(){
//   Serial.println("trying out the mosfet");
//  Serial.println("MOSFET ON");   
  digitalWrite(MOS,LOW);  
  delay(1000); 
    for(int k=0;k<20;k++)
    {    
        for(int i=0;i<Navg;i++)
        {
          
          vl = analogRead(vlow);
          vh = analogRead(vhigh);
          
          
          vl = (vl/1024.0)*3.3;
          vh = (vh/1024.0)*3.3;
          
          sumvl+= vl;sumvh+= vh;
        }
        
        vl = sumvl/Navg;
        vh = sumvh/Navg;
        
        sumvh=0;sumvl=0;
      
      // for Plotter
      
        Serial.print(vl);Serial.print(",");Serial.print(vh);Serial.print(",");Serial.println();
//        Serial.print("3.3");Serial.print(",");Serial.print("0");Serial.println();
        delay(50);
    }
    Serial.println("MOSFET OFF");
    digitalWrite(MOS,HIGH);
    
    for(int k=0;k<20;k++)
    {    
        for(int i=0;i<Navg;i++)
        {
          
          vl = analogRead(vlow);
          vh = analogRead(vhigh);
          
          
          vl = (vl/1024.0)*3.3;
          vh = (vh/1024.0)*3.3;
          
          sumvl+= vl;sumvh+= vh;
        }
        
        vl = sumvl/Navg;
        vh = sumvh/Navg;
        
        sumvh=0;sumvl=0;
      
      // for Plotter
      
        Serial.print(vl);Serial.print(",");Serial.print(vh);Serial.print(",");
        Serial.print("3.3");Serial.print(",");Serial.print("0");Serial.println();
        delay(50);
    }
}
  


