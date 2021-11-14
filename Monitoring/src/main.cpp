#include <SPI.h>     //Import SPI librarey
#include <RH_RF95.h> // RF95 from RadioHead Librarey
#include <PZEM004Tv30.h>
#include <Arduino.h>
#include <LoRa.h>
#include <ModbusMaster.h>

#define MAX485_DE 48
#define MAX485_RE 49

double sensorValue1 = 0;
double sensorValue2 = 0;
int crosscount = 0;
int climb_flag = 0;
int val[100];
int max_v = 0;
double VmaxD = 0;
double VeffD = 0;
double Veff = 0;

static uint8_t pzemSlaveAddr = 0x01;
static uint16_t NewshuntAddr = 0x0000;

// /* ModbusMaster node;                     /* activate modbus master codes*/
// float PZEMVoltage = 0;                 /* Declare value for DC voltage */
// float PZEMCurrent = 0;                 /* Declare value for DC current*/
// float PZEMPower = 0;                   /* Declare value for DC Power */
// float PZEMEnergy = 0;                  /* Declare value for DC Energy */
// unsigned long startMillisPZEM;         /* start counting time for LCD Display */
// unsigned long currentMillisPZEM;       /* current counting time for LCD Display */
// const unsigned long periodPZEM = 1000; // refresh every X seconds (in seconds) in LED Display. Default 1000 = 1 second
// int page = 1;                          /* display different pages on LCD Display*/

// /* 2 - LCD Display  */
// /* refresh every X seconds (in seconds) in LED Display. Default 1000 = 1 second */
// int ResetEnergy = 0;                     /* reset energy function */
// unsigned long startMillisEnergy;         /* start counting time for LCD Display */
// unsigned long currentMillisEnergy;       /* current counting time for LCD Display */
// const unsigned long periodEnergy = 1000; // refresh every X seconds (in seconds) in LED Display. Default 1000 = 1 second */

int counter = 0;

PZEM004Tv30 pzem(12, 14); // Software Serial pin 5 (RX) & 4 (TX)

float DataVoltage;
float DataCurrent;
float DataPower;
float DataEnergy;
float DataFrequency;
float DataPowerfactor;

void preTransmission() /* transmission program when triggered*/
{

  /* 1- PZEM-017 DC Energy Meter */

  digitalWrite(MAX485_RE, 1); /* put RE Pin to high*/
  digitalWrite(MAX485_DE, 1); /* put DE Pin to high*/
  delay(1);                   // When both RE and DE Pin are high, converter is allow to transmit communication
}

void postTransmission() /* Reception program when triggered*/
{

  /* 1- PZEM-017 DC Energy Meter */

  delay(3);                   // When both RE and DE Pin are low, converter is allow to receive communication
  digitalWrite(MAX485_RE, 0); /* put RE Pin to low*/
  digitalWrite(MAX485_DE, 0); /* put DE Pin to low*/
}

// void setShunt(uint8_t slaveAddr) //Change the slave address of a node
// {

//   /* 1- PZEM-017 DC Energy Meter */

//   static uint8_t SlaveParameter = 0x06;     /* Write command code to PZEM */
//   static uint16_t registerAddress = 0x0003; /* change shunt register address command code */

//   uint16_t u16CRC = 0xFFFF;                 /* declare CRC check 16 bits*/
//   u16CRC = crc16_update(u16CRC, slaveAddr); // Calculate the crc16 over the 6bytes to be send
//   u16CRC = crc16_update(u16CRC, SlaveParameter);
//   u16CRC = crc16_update(u16CRC, highByte(registerAddress));
//   u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
//   u16CRC = crc16_update(u16CRC, highByte(NewshuntAddr));
//   u16CRC = crc16_update(u16CRC, lowByte(NewshuntAddr));

//   Serial.println("Change shunt address");
//   preTransmission(); /* trigger transmission mode*/

//   Serial2.write(slaveAddr); /* these whole process code sequence refer to manual*/
//   Serial2.write(SlaveParameter);
//   Serial2.write(highByte(registerAddress));
//   Serial2.write(lowByte(registerAddress));
//   Serial2.write(highByte(NewshuntAddr));
//   Serial2.write(lowByte(NewshuntAddr));
//   Serial2.write(lowByte(u16CRC));
//   Serial2.write(highByte(u16CRC));
//   delay(10);
//   postTransmission(); /* trigger reception mode*/
//   delay(100);
//   while (Serial2.available()) /* while receiving signal from Serial2 from meter and converter */
//   {
//     Serial.print(char(Serial2.read()), HEX); /* Prints the response and display on Serial Monitor (Serial)*/
//     Serial.print(" ");
//   }
// }

void changeAddress(uint8_t OldslaveAddr, uint8_t NewslaveAddr) //Change the slave address of a node
{

  /* 1- PZEM-017 DC Energy Meter */

  static uint8_t SlaveParameter = 0x06;        /* Write command code to PZEM */
  static uint16_t registerAddress = 0x0002;    /* Modbus RTU device address command code */
  uint16_t u16CRC = 0xFFFF;                    /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, OldslaveAddr); // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));

  Serial.println("Change Slave Address");
  preTransmission(); /* trigger transmission mode*/

  Serial2.write(OldslaveAddr); /* these whole process code sequence refer to manual*/
  Serial2.write(SlaveParameter);
  Serial2.write(highByte(registerAddress));
  Serial2.write(lowByte(registerAddress));
  Serial2.write(highByte(NewslaveAddr));
  Serial2.write(lowByte(NewslaveAddr));
  Serial2.write(lowByte(u16CRC));
  Serial2.write(highByte(u16CRC));
  delay(10);
  postTransmission(); /* trigger reception mode*/
  delay(100);
  while (Serial2.available()) /* while receiving signal from Serial2 from meter and converter */
  {
    Serial.print(char(Serial2.read()), HEX); /* Prints the response and display on Serial Monitor (Serial)*/
    Serial.print(" ");
  }
}

void bacaPZEM()
{
 /* DataVoltage = pzem.voltage();
  if (DataVoltage != NAN)
  {
    Serial.print("Voltage: ");
    Serial.print(DataVoltage);
    Serial.println("V");
  }
  else
  {
    Serial.println("Error reading voltage");
  } 

   Serial.println("Polinema-K1AC : "); */

  for ( int i = 0; i < 100; i++ ) {
    sensorValue1 = analogRead(A0);
    if (analogRead(A0) > 511) {
      val[i] = sensorValue1;
    }
    else {
      val[i] = 0;
    }
    delay(1);
  }

  max_v = 0;

  for ( int i = 0; i < 100; i++ )
  {
    if ( val[i] > max_v )
    {
      max_v = val[i];
    }
    val[i] = 0;
  }
  if (max_v != 0) {


    VmaxD = max_v;
    VeffD = VmaxD / sqrt(2);
    Veff = (((VeffD - 420.76) / -90.24) * -210.2) + 210.2;
  }
  else {
    Veff = 0;
  }
  Serial.print("Voltage: ");
  Serial.print(Veff);
  Serial.println("V");
  VmaxD = 0; 

  DataCurrent = pzem.current();
  if (DataCurrent != NAN)
  {
    Serial.print("Current: ");
    Serial.print(DataCurrent);
    Serial.println("A");
  }
  else
  {
    Serial.println("Error reading current");
  }

  DataPower = pzem.power();
  if (DataCurrent != NAN)
  {
    Serial.print("Power: ");
    Serial.print(DataPower);
    Serial.println("W");
  }
  else
  {
    Serial.println("Error reading power");
  }

  DataEnergy = pzem.energy();
  if (DataCurrent != NAN)
  {
    Serial.print("Energy: ");
    Serial.print(DataEnergy, 4);
    Serial.println("kWh");
  }
  else
  {
    Serial.println("Error reading energy");
  }

  DataFrequency = pzem.frequency();
  if (DataCurrent != NAN)
  {
    Serial.print("Frequency: ");
    Serial.print(DataFrequency, 1);
    Serial.println("Hz");
  }
  else
  {
    Serial.println("Error reading frequency");
  }

  DataPowerfactor = pzem.pf();
  if (DataCurrent != NAN)
  {
    Serial.print("PF: ");
    Serial.print(DataPowerfactor);
    Serial.println("VA");
  }
  else
  {
    Serial.println("Error reading power factor");
  }
}

void setup()
{

  // Serial.begin(9600); /* to display readings in Serial Monitor at 9600 baud rates */

  // /* 1- PZEM-017 DC Energy Meter */

  // // setShunt(0x01);                          // Delete the "//" to set shunt rating (0x01) is the meter address by default
  // // resetEnergy(0x01);                       // By delete the double slash symbol, the Energy value in the meter is reset. Can also be reset on the LCD Display
  // startMillisPZEM = millis();            /* Start counting time for run code */
  // Serial2.begin(9600, SERIAL_8N2);       /* To assign communication port to communicate with meter. with 2 stop bits (refer to manual)*/
  //                                        // By default communicate via Serial2 port: pin 14 (Tx) and pin 15 (Rx)
  // node.begin(pzemSlaveAddr, Serial2);    /* Define and start the Modbus RTU communication. Communication to specific slave address and which Serial port */
  // pinMode(MAX485_RE, OUTPUT);            /* Define RE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  // pinMode(MAX485_DE, OUTPUT);            /* Define DE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  // digitalWrite(MAX485_RE, 0);            /* Arduino create output signal for pin RE as LOW (no output)*/
  // digitalWrite(MAX485_DE, 0);            /* Arduino create output signal for pin DE as LOW (no output)*/
  //                                        // both pins no output means the converter is in communication signal receiving mode
  // node.preTransmission(preTransmission); // Callbacks allow us to configure the RS485 transceiver correctly
  // node.postTransmission(postTransmission);
  // changeAddress(0XF8, 0x01); // By delete the double slash symbol, the meter address will be set as 0x01.
  //                            // By default I allow this code to run every program startup. Will not have effect if you only have 1 meter

  // delay(1000); /* after everything done, wait for 1 second */
  //LoRa.setFrequency(433E6);
  Serial.begin(9600);
  while (!Serial)
    ;

  Serial.println("LoRa Sender");

  if (!LoRa.begin(915E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
}

// void bacaPZEMDC()
// {
//   currentMillisPZEM = millis();                          /* count time for program run every second (by default)*/
//   if (currentMillisPZEM - startMillisPZEM >= periodPZEM) /* for every x seconds, run the codes below*/
//   {
//     uint8_t result;                              /* Declare variable "result" as 8 bits */
//     result = node.readInputRegisters(0x0001, 6); /* read the 9 registers (information) of the PZEM-014 / 016 starting 0x0000 (voltage information) kindly refer to manual)*/
//     Serial.println(result);
//     if (result != 0) /* If there is a response node.ku8MBSuccess*/
//     {
//       uint32_t tempdouble = 0x00000000;                     /* Declare variable "tempdouble" as 32 bits with initial value is 0 */
//       PZEMVoltage = node.getResponseBuffer(0x0000) / 100.0; /* get the 16bit value for the voltage value, divide it by 100 (as per manual) */
//                                                             // 0x0000 to 0x0008 are the register address of the measurement value
//       PZEMCurrent = node.getResponseBuffer(0x0001) / 100.0; /* get the 16bit value for the current value, divide it by 100 (as per manual) */

//       tempdouble = (node.getResponseBuffer(0x0003) << 16) + node.getResponseBuffer(0x0002); /* get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
//       PZEMPower = tempdouble / 10.0;                                                        /* Divide the value by 10 to get actual power value (as per manual) */

//       tempdouble = (node.getResponseBuffer(0x0005) << 16) + node.getResponseBuffer(0x0004); /* get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
//       PZEMEnergy = tempdouble;

//       Serial.print(PZEMVoltage, 1); /* Print Voltage value on Serial Monitor with 1 decimal*/
//       Serial.print("V   ");
//       Serial.print(PZEMCurrent, 3);
//       Serial.print("A   ");
//       Serial.print(PZEMPower, 1);
//       Serial.print("W  ");
//       Serial.print(PZEMEnergy, 0);
//       Serial.print("Wh  ");
//       Serial.println();

//       if (pzemSlaveAddr == 2) /* just for checking purpose to see whether can read modbus*/
//       {
//         Serial.println();
//       }
//     }
//     else
//     {
//       Serial.println("Failed to read modbus");
//     }
//     startMillisPZEM = currentMillisPZEM; /* Set the starting point again for next counting time */
//   }
// }

void loop()
{
  /* bacaPZEMDC();
  String voltdc = String(PZEMVoltage);
  String ampdc = String(PZEMCurrent);
  String wattdc = String(PZEMPower);
  String whdc = String(PZEMEnergy);
  String dataDC = "Polinema-K1DC;V;" + voltdc + ";A;" + ampdc + ";W;" + wattdc + ";wh;" + whdc; */
  bacaPZEM();
  String volt = String(DataCurrent);
  String amp = String(DataCurrent);
  String watt = String(DataPower);
  String kwh = String(DataEnergy);
  String hz = String(DataFrequency);
  String pf = String(DataPowerfactor);
  String dataAC = "Polinema-K1AC;V;" + volt + ";A;" + amp + ";W;" + watt + ";kwh;" + kwh + ";hz;" + hz + ";pf;" + pf;
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print(dataAC);
  //LoRa.print(dataDC);
  LoRa.endPacket();

  counter++;
  if (counter == 1000)
    counter = 0;

  delay(1000);
}

/* int counter = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setTxPower(20);
  
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  delay(3000);

 */

/* #define RFM95_CS 53 //CS if Lora connected to pin 10
#define RFM95_RST 9 //RST of Lora connected to pin 9
#define RFM95_INT 2 //INT of Lora connected to pin 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

PZEM004Tv30 pzem(12, 14); // Software Serial pin 5 (RX) & 4 (TX)

float DataVoltage;
float DataCurrent;
float DataPower;
float DataEnergy;
float DataFrequency;
float DataPowerfactor;

int period = 2000; //interval simpan data ke db
int time_now = 0;

void setup() 
{
 
//Initialize Serial Monitor
  Serial.begin(9600);
  
// Reset LoRa Module 
  pinMode(RFM95_RST, OUTPUT); 
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

//Initialize LoRa Module
  while (!rf95.init()) {
    Serial.println("Initializing...");
    while (1);
  }
  Serial.println("Succeded");
  

 //Set the default frequency 434.0MHz
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set FREQ to: ");
  Serial.println(RF95_FREQ);

  rf95.setTxPower(18);
}

char value = 48;
void bacaPZEM()
{
   float voltage = pzem.voltage();
   if(voltage != NAN){
       Serial.print("Voltage: ");
       Serial.print(voltage);
       Serial.println("V");
   } else {
       Serial.println("Error reading voltage");
   }

   float current = pzem.current();
   if(current != NAN){
       Serial.print("Current: ");
       Serial.print(current);
       Serial.println("A");
   } else {
       Serial.println("Error reading current");
   }

   float power = pzem.power();
   if(current != NAN){
       Serial.print("Power: ");
       Serial.print(power);
       Serial.println("W");
   } else {
       Serial.println("Error reading power");
   }

   float energy = pzem.energy();
   if(current != NAN){
       Serial.print("Energy: ");
       Serial.print(energy,3);
       Serial.println("kWh");
   } else {
       Serial.println("Error reading energy");
   }

   float frequency = pzem.frequency();
   if(current != NAN){
       Serial.print("Frequency: ");
       Serial.print(frequency, 1);
       Serial.println("Hz");
   } else {
       Serial.println("Error reading frequency");
   }

   float pf = pzem.pf();
   if(current != NAN){
       Serial.print("PF: ");
       Serial.println(pf);
   } else {
       Serial.println("Error reading power factor");
   }

    if (millis() >= time_now){
    String volt = String(DataVoltage);
    String amp = String(DataCurrent);
    String watt = String(DataPower);
    String kwh = String(DataEnergy);
    String hz = String(DataFrequency);
    String pf = String(DataPowerfactor);
    String data = volt + amp + watt + kwh + hz + pf;
    char d[72];
    data.toCharArray(d, 10); //String to char array
    Serial.println("Sending to rf95_server");

    rf95.send(d, sizeof(d));
    rf95.waitPacketSent();
    }

   Serial.println();
   delay(2000);
   Serial.println("Send: ");
  char radiopacket[1] = {char(value)};
  rf95.setModeRx();
  rf95.send((uint8_t *)radiopacket, 1);
  
    
  delay(1000);
  rf95.setModeTx();
  delay(2000);
  value++;
  if (value > '9'){
  value = 48;}
   */

/* if (millis() >= time_now + period)
  {
    Serial.print("Testing3:");
    Serial.println(millis());
    time_now += period;
    String volt = String(DataVoltage);
    String amp = String(DataCurrent);
    String watt = String(DataPower);
    String kwh = String(DataEnergy);
    String hz = String(DataFrequency);
    String pf = String(DataPowerfactor);
    String data = volt + amp + watt + kwh + hz + pf;
    char d[72];
    data.toCharArray(d, 5); //String to char array
    Serial.println("Sending to rf95_server");

    rf95.send(d, sizeof(d));
    rf95.waitPacketSent();  
  } */
//}

/* void loop()
{
  
  bacaPZEM(); 
  

 */

/* Serial.print("TimeNow:");
  Serial.println(time_now);
  DataVoltage = pzem.voltage();
  if (DataVoltage != NAN)
  {
    Serial.print("Voltage: ");
    Serial.print(DataVoltage);
    Serial.println("V");
  }
  else
  {
    Serial.println("Error reading voltage");
  }

  DataCurrent = pzem.current();
  if (DataCurrent != NAN)
  {
    Serial.print("Current: ");
    Serial.print(DataCurrent);
    Serial.println("A");
  }
  else
  {
    Serial.println("Error reading current");
  }

  DataPower = pzem.power();
  if (DataCurrent != NAN)
  {
    Serial.print("Power: ");
    Serial.print(DataPower);
    Serial.println("W");
  }
  else
  {
    Serial.println("Error reading power");
  }

  DataEnergy = pzem.energy();
  if (DataCurrent != NAN)
  {
    Serial.print("Energy: ");
    Serial.print(DataEnergy, 3);
    Serial.println("kWh");
  }
  else
  {
    Serial.println("Error reading energy");
  }

  DataFrequency = pzem.frequency();
  if (DataCurrent != NAN)
  {
    Serial.print("Frequency: ");
    Serial.print(DataFrequency, 1);
    Serial.println("Hz");
  }
  else
  {
    Serial.println("Error reading frequency");
  }

  DataPowerfactor = pzem.pf();
  if (DataCurrent != NAN)
  {
    Serial.print("PF: ");
    Serial.println(DataPowerfactor);
  }
  else
  {
    Serial.println("Error reading power factor");
  }

  delay(2000);
  Serial.println("Testing1");
  if (millis() >= time_now + period)
  {
    Serial.print("Testing3:");
    Serial.println(millis());
    time_now += period;
    String volt = String(DataVoltage);
    String amp = String(DataCurrent);
    String watt = String(DataPower);
    String kwh = String(DataEnergy);
    String hz = String(DataFrequency);
    String pf = String(DataPowerfactor);
    String data = volt + amp + watt + kwh + hz + pf;
    char d[72];
    data.toCharArray(d, 5); //String to char array
    Serial.println("Sending to rf95_server");

    rf95.send(d, sizeof(d));
    rf95.waitPacketSent();
  }
  Serial.println("Testing2"); */
//}