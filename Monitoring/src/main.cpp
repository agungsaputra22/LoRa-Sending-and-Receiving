// PZEM-017 DC Energy Meter with LCD By Solarduino

// Note Summary
// Note :  Safety is very important when dealing with electricity. We take no responsibilities while you do it at your own risk.
// Note :  This DC Energy Monitoring Code needs PZEM-017 DC Energy Meter to measure values and Arduio Mega / UNO for communication and display.
// Note :  This Code monitors DC Voltage, current, Power, and Energy.
// Note :  The values shown in LCD Display is refreshed every second.
// Note :  The values are calculated internally by energy meter and function of Arduino is only to read the value and for further calculation.
// Note :  The first step is need to select shunt value and change the value accordingly. look for line "static uint16_t NewshuntAddr = 0x0000; "
// Note :  You need to download and install (modified) Modbus Master library at our website (https://solarduino.com/pzem-014-or-016-ac-energy-meter-with-arduino/ )
// Note :  The Core of the code was from EvertDekker.com 2018 which based on the example from http://solar4living.com/pzem-arduino-modbus.htm
// Note :  Solarduino only amend necessary code and integrate with LCD Display Shield.

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/ ////////////*/

/* 1- PZEM-017 DC Energy Meter */



#include <SPI.h>     //Import SPI librarey
#include <RH_RF95.h> // RF95 from RadioHead Librarey
#include <PZEM004Tv30.h>
#include <SPI.h>
#include <LoRa.h>

int counter = 0;

PZEM004Tv30 pzem(12, 14); // Software Serial pin 5 (RX) & 4 (TX)

float DataVoltage;
float DataCurrent;
float DataPower;
float DataEnergy;
float DataFrequency;
float DataPowerfactor;

void bacaPZEM()
{
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

}

void setup()
{
  //LoRa.setFrequency(433E6);
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(433E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
}

void loop()
{
  bacaPZEM();
  String volt = String(DataVoltage );
  String amp = String(DataCurrent );
  String watt = String(DataPower );
  String kwh = String(DataEnergy );
  String hz = String(DataFrequency );
  String pf = String(DataPowerfactor );
  String data = volt + "," + amp + "," + watt + "," + kwh + "," + hz + "," + pf + ";";
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print(data);
  LoRa.endPacket();

  counter++;

  delay(1500);
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