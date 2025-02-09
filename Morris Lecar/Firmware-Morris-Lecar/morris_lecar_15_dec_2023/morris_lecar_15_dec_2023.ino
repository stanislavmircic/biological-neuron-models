//
// Made for Computational SpikerBox V0.1
// 15. Dec. 2023
// BackyardBrains
// Written: Stanislav Mircic
//
//
// Simulation frequency
// Baud rate 230400
// 6.88kHz - with Spike Recorder one channel (V) with one input (force) - no potentiometers
// 3.04kHz - with Spike Recorder one channel (V) with one input (force) and 3 potentiometers
// 6.86kHz - with Spike Recorder one channel (V) with one input (force) and 3 time-multiplexed potentiometers
// 6.61kHz - with Spike Recorder one channel (V) with one input (force) and 3 time-multiplexed potentiometers, dig outputs
// 3.69kHz - with Spike Recorder one channel (V) with one input (force) and 3 time-multiplexed potentiometers, dig outputs, 2 analog inputs
//
// Inputs:
//        - analog input jack (0-5V)
//
// Outputs:
//        - TTL pulse whenever V crosses 30mV
//        - serial communication with Spike Recorder
//
// Potentiometers:
//        Pot R1 - pin GPIO4
//        Pot R2 - pin GPIO5
//        Pot R3 - pin GPIO6
//
//        Analog input 1 - pin GPIO1
//        Analog input 2 - pin GPIO2
//
//        Digital out (LP filtered) 1  - pin GPIO13
//        Digital out (LP filtered) 1  - pin GPIO14
//


#define USE_SPIKE_RECORDER
//#define USE_ANALOG_INPUTS

#define SENSOR_INPUT_PIN 3
#define SENSITIVITY_SENSOR_INPUT (0.05)

#define OUT1_PIN 13
#define OUT2_PIN 14

#define SENSITIVITY_ANALOG_INPUTS (0.1)
#define INPUT1_PIN 1
#define INPUT2_PIN 2

//gL - R1
#define gL_POT_PIN 4
//range of the parameter
#define gL_POT_MIN_VALUE  (0.0)
#define gL_POT_MAX_VALUE  (4.0)
float gLPotSensitivity = 1;

//gCa - R2
#define gCa_POT_PIN 5
//range of the parameter
#define gCa_POT_MIN_VALUE  (2.0)
#define gCa_POT_MAX_VALUE  (6.0)
float gCaPotSensitivity = 1;

//gK - R3
#define gK_POT_PIN 6
//range of the parameter
#define gK_POT_MIN_VALUE  (7.0)
#define gK_POT_MAX_VALUE  (8.0)
float gKPotSensitivity = 1;


#include <HardwareSerial.h>
#include "driver/uart.h"

// #define TXD1                GPIO_NUM_17
// #define RXD1                GPIO_NUM_18
// #define RTS1                GPIO_NUM_19
// #define CTS1                GPIO_NUM_20

#define TXD0                GPIO_NUM_43
#define RXD0                GPIO_NUM_44
#define DEVICE_NAME_CHAR 25
#define SPIKE_THRESHOLD 30
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

int32_t           length = 0;
const uart_port_t uart_num = UART_NUM_0;   //UART_NUM_0
uint8_t           receiveBuffer[1000];

void setup() 
{
  // put your setup code here, to run once:
  uart_config_t uart_config = {
    .baud_rate = 230400,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,//UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 125,
  };

  // Setup UART buffered IO with event queue
  const int     uart_buffer_size_rx = 1000;
  const int     uart_buffer_size_tx = 32000;
  QueueHandle_t uart_queue;
  
  // Install UART driver using an event queue here
  ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size_rx, uart_buffer_size_tx, 10, &uart_queue, 0));
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

  // Set UART pins(TX, RX, RTS, CTS)
  //ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD1, RXD1, RTS1, CTS1));
  ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));


  pinMode(SENSOR_INPUT_PIN,INPUT);

  pinMode(gL_POT_PIN, INPUT);
  gLPotSensitivity = (gL_POT_MAX_VALUE-gL_POT_MIN_VALUE)/1024.0;

  pinMode(gCa_POT_PIN, INPUT);
  gCaPotSensitivity = (gCa_POT_MAX_VALUE-gCa_POT_MIN_VALUE)/1024.0;

  pinMode(gK_POT_PIN, INPUT);
  gKPotSensitivity = (gK_POT_MAX_VALUE-gK_POT_MIN_VALUE)/1024.0;

  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
  pinMode(OUT1_PIN, OUTPUT);
  pinMode(OUT2_PIN, OUTPUT);

  pinMode(INPUT1_PIN, INPUT);
  pinMode(INPUT2_PIN, INPUT);

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
double sensorInput = 0;
double   lastVoltageValue = 0;
void loop() 
{
    digitalWrite(12, HIGH);
    Iapp = 0;

    //multiplex in time ADC measurement. We don't need fast measurement 
    //and ADC will significantly slow down loop. We will measure just one thing per loop
    switch (inputCounter) {
      case 0:
        sensorInput = (1024-analogRead(SENSOR_INPUT_PIN))*SENSITIVITY_SENSOR_INPUT;
      break;
      case 1:
        gL = gL_POT_MIN_VALUE + analogRead(gL_POT_PIN)*gLPotSensitivity;
      break;
      case 2:
        gK = gK_POT_MIN_VALUE + analogRead(gK_POT_PIN)*gLPotSensitivity;
      break;
      case 3:
        gCa = gCa_POT_MIN_VALUE + analogRead(gCa_POT_PIN)*gCaPotSensitivity;
      break;
      default:
        inputCounter = 0;
      break;
    }
    inputCounter++;
    if(inputCounter>3)
    {
      inputCounter = 0;
    }

    Iapp = Iapp + sensorInput;

    #ifdef USE_ANALOG_INPUTS
    //we don't time-multiplex analog inputs since we can expect spikes on inputs.
    //The spikes are fast, we don't want to miss them.
      Iapp = Iapp + SENSITIVITY_ANALOG_INPUTS*(analogRead(INPUT1_PIN)-512);
      Iapp = Iapp + SENSITIVITY_ANALOG_INPUTS*(analogRead(INPUT2_PIN)-512);
    #endif


    minf = 0.5*(1+tanh((V-V1)/V2));
    tau_n = 1.0/cosh((V-V3)/(2*V4));
    ninf = 0.5*(1+tanh((V-V3)/V4));

    V = V + ((-gL*(V-EL) - gK*n*(V-EK) - gCa*minf*(V-ECa)+Iapp)/Cm)*dt;
    n = n + phi*((ninf-n)/tau_n)*dt;
    

  //set outputs HIGH if voltage crosses threashold
  //also set if voltage in previous step crossed 
  //the threshold (to prolongate the spike)
  if((V>SPIKE_THRESHOLD) || (lastVoltageValue>SPIKE_THRESHOLD))
  {
      digitalWrite(OUT1_PIN, HIGH);
      digitalWrite(OUT2_PIN, HIGH);
  }
  else
  {
      digitalWrite(OUT1_PIN, LOW);
      digitalWrite(OUT2_PIN, LOW);
  }
  lastVoltageValue = V;

  //send voltage information through serial/USB
  ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
  length = uart_read_bytes(uart_num, receiveBuffer, length, 10);   
  if(length > 0)     
  {
    uart_tx_chars(uart_num, (const char*)deviceName, DEVICE_NAME_CHAR);
    uart_tx_chars(uart_num, (const char*)deviceName, DEVICE_NAME_CHAR);
    uart_tx_chars(uart_num, (const char*)deviceName, DEVICE_NAME_CHAR);
    somethingArrivedOnSerial = true;
  }
    //test output
  #ifdef USE_SPIKE_RECORDER
    //transform voltage to fit in 10 bit uint (0-1024)
    //make it positive (+100) and stretch over full interval (*5)
    uint16_t voltage = (V+100)*5;
    //convert data to frame according to protocol
    outputFrameBuffer[0]= (voltage>>7)| 0x80;           
    outputFrameBuffer[1]=  voltage & 0x7F;   
    if(somethingArrivedOnSerial)
    {
      uart_tx_chars(uart_num, (const char*)outputFrameBuffer, 2);
    }
  #else
    Serial.println(V);
  #endif
    digitalWrite(12, LOW);
}
