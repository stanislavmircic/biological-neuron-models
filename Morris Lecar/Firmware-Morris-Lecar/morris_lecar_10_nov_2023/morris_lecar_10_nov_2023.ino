//
// Made for Computational SpikerBox V0.1
// 20. Nov. 2023
// BackyardBrains
// Written: Stanislav Mircic
//
//
// Simulation frequency
// 2.96kHz - with text at baud rate 230400
//
// Inputs:
//        - analog input jack (0-5V)
//
// Outputs:
//        - TTL pulse whenever V crosses 30mV
//        - serial communication @ 2Mbaud (text values of V) 
//
// Potentiometers:
//        Pot R1 - pin 4
//        Pot R2 - pin 5
//        Pot R3 - pin 6
//
//        Analog input 1 - pin 1
//        Analog input 2 - pin 2
//
//        Analog out (LP filtered) 1  - pin 13
//        Analog out (LP filtered) 1  - pin 14
//

//#define USE_SPIKE_RECORDER
#define DEVICE_NAME_CHAR 25
uint8_t		deviceName[DEVICE_NAME_CHAR] = {255, 255, 1, 1, 128, 255, 'H', 'W', 'T', ':', 'M', 'U', 'S', 'C', 'L', 'E', 'S', 'S', ';', 255, 255, 1, 1, 129, 255};
uint8_t outputFrameBuffer[4];

//Time step
double  dt = 1;

 
double minf;
double tau_n;
double ninf;

//initial values 
double   Iapp = 65;
double   V = 0;
double   n = 0;

//Reversal potentials
double   ECa = 120; //mV
double   EK = -84; //mV
double   EL = -60; //mV
double   Cm = 20; //microF/cm2

double   gL = 2; //mS/cm2
double   gK = 8;
double   gKCa = 0.75;

double   V1 = -1.2;
double   V2 = 18;
 
//SNLC - default
double   V3 = 12;
double   V4 = 17.4;
double   phi = 0.067;
double   gCa = 4;

// //Hopf
//double V3 = 2;
//double V4 = 30;
//double phi = 0.04;
//double gCa = 4.4;

// //Homoclinic
//double V3 = 12;
//double V4 = 17.4;
//double phi = 0.23;
//double gCa = 4;


void setup() 
{
  Serial.begin(500000);
  //set the resolution to 12 bits (0-4096)
  analogReadResolution(10);

  // //debug pin
  pinMode(12, OUTPUT);
}

//
// Create a random integer from 0 - 65535
//
unsigned int rng() {
  static unsigned int y = 0;
  y += micros(); // seeded with changing number
  y ^= y << 2; y ^= y >> 7; y ^= y << 7;
  return (y);
}

int inputCounter = 0;
bool somethingArrivedOnSerial = false;

void loop() 
{
    digitalWrite(12, HIGH);
    Iapp = 0;

    double analogInput = 0;
    //get current from pot
    inputCounter = inputCounter +1;
    if(inputCounter>1000)
    {
      inputCounter = 0;
    }
    if(inputCounter>500)
    {
        analogInput = 60;
        
    }
    Iapp = Iapp + analogInput;

    minf = 0.5*(1+tanh((V-V1)/V2));
    tau_n = 1.0/cosh((V-V3)/(2*V4));
    ninf = 0.5*(1+tanh((V-V3)/V4));


    V = V + ((-gL*(V-EL) - gK*n*(V-EK) - gCa*minf*(V-ECa)+Iapp)/Cm)*dt;
    n = n + phi*((ninf-n)/tau_n)*dt;
    

    // while (Serial.available() > 0) 
    // {
    //   // read the incoming byte:
    //   int incomingByte = Serial.read();
    //   somethingArrivedOnSerial = true;
    // }

    // if(somethingArrivedOnSerial)
    // {
    //   somethingArrivedOnSerial = false;
    //   Serial.write(deviceName, DEVICE_NAME_CHAR);
    // }

    //test output
  #ifdef USE_SPIKE_RECORDER
    uint16_t voltage = (V+100)*5;
    //convert data to frame according to protocol
    outputFrameBuffer[0]= (voltage>>7)| 0x80;           
    outputFrameBuffer[1]=  voltage & 0x7F;   
    Serial.write(outputFrameBuffer,2);  
  #else
    Serial.println(V);
  #endif
    digitalWrite(12, LOW);
}
