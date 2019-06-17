#define BOARD_ADAFRUIT_FEATHER_M0
//#define BOARD_ADAFRUIT_FEATHER_NRF52

//int vfol = 2;
int vlow = A3;
int vhigh = A4;
//int vbias = 28;
float vl=0,vh=0,vf=0,vb=0;
float sumvf=0,sumvl=0,sumvh=0,sumvb=0;
int Navg=64;

#if defined(BOARD_ADAFRUIT_FEATHER_M0)
  int MOS = 5 ;//gpio pin assigned ot the mosfet
#elif defined(BOARD_ADAFRUIT_FEATHER_NRF52)
  int MOS = 27;//gpio pin assigned ot the mosfet
#endif


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("starting");
  //pinMode(vfol,INPUT);
  pinMode(vlow,INPUT);
  pinMode(vhigh,INPUT);
  pinMode(MOS,OUTPUT);
  digitalWrite(MOS,LOW);// Switch is ON. hence, The GSR is powered

  
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i=0;i<Navg;i++)
  {
    //vf = analogRead(vfol);
    vl = analogRead(vlow);
    vh = analogRead(vhigh);
    //vb = analogRead(vbias);
    //vf = (vf/1024.0)*3.3;
    vl = (vl/1024.0)*3.3;
    vh = (vh/1024.0)*3.3;
    //vb = (vb/1024.0)*3.3;
    //sumvf+= vf;
    sumvl+= vl;
    sumvh+= vh;
    //sumvb+= vb;
  }
  //vf = sumvf/Navg;
  vl = sumvl/Navg;
  vh = sumvh/Navg;
  //vb = sumvb/Navg;
  //sumvf=0;
  //sumvb=0;
  sumvh=0;
  sumvl=0;
// Printing on monitor
//  Serial.print("vbias=");Serial.print(vb);
//  Serial.print("\t vfol = ");Serial.print(vf);
//  Serial.print("\t vlow = ");Serial.print(vl);
//  Serial.print("\t vhigh = ");Serial.println(vh);

// for Plotter

//Serial.print(vb);Serial.print(",");Serial.print(vf);Serial.print(",");
Serial.print(vl);Serial.print(",");
Serial.print(vh);Serial.print(",");
Serial.print("3.3");Serial.print(",");Serial.print("0");Serial.println();
//Serial.println(vh);  
}
