/*
 # Editor : liji
 # Ver    : 1.0
 # Product: analog sensor meter Pro
*/
#include <OneWire.h>
#include <SPI.h>
#include <SFE_CC3000.h>
#include <SFE_CC3000_Client.h>
#include <PubSubClient.h>

// Pin
#define CC3000_INT      7   // Needs to be an interrupt pin (D2/D3)
#define CC3000_EN       5   // Can be any digital pin
#define CC3000_CS       10  // Preferred is pin 10 on Uno

#define publishInterval 10000
#define samplingInterval 20 // 采样频率
#define ArrayLenth  40 
#define Offset -0.27  //deviation compensate
#define PHSensorPin A2    //pH meter Analog output to Arduino Analog Input 2
byte ECSensorPin = A1;    //EC Meter analog output,pin on analog 1

// Global Variables
SFE_CC3000 wifi = SFE_CC3000(CC3000_INT, CC3000_EN, CC3000_CS);
SFE_CC3000_Client ethClient = SFE_CC3000_Client(wifi);
// Mqtt
PubSubClient client(ethClient);
char username[] = "root";  
char password[] = "fisher";
char clientId[] = "fisher01-";
IPAddress server(106, 185, 35, 79);

char ap_ssid[] = "jiluyou_lj";                  // SSID of network
char ap_password[] = "Jianngh@123";          // Password of network
unsigned int ap_security = WLAN_SEC_WPA2; 
unsigned int timeout = 30000;

//Sensor
int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2
//Temperature chip i/o
OneWire ds(DS18S20_Pin);
float temperature,ECcurrent,ph;
char chartemp[20]; // save message temp
static unsigned long printTime = millis();
static unsigned long lastSendTime = millis();
static unsigned long samplingTime = millis();

int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0; 
int ecArray[ArrayLenth];   //Store the average value of the sensor feedback
int ecArrayIndex=0; 

void setup() {
  // put your setup code here, to run once:
//  Serial.begin(9600);
  // init mqtt
  client.setServer(server, 1883);
  client.setCallback(callback);
  // connect wifi
  connectWifi();
  // Allow the hardware to sort itself out
  delay(1500);
  
}

void loop() {
  static float pHValue,phVoltage,ecVoltage;
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  // Temperature
  temperature = getTemp(); 
  // sample data
  if(millis()-samplingTime > samplingInterval) {
      // =========PH==========
      pHArray[pHArrayIndex++] = analogRead(PHSensorPin);
      if(pHArrayIndex == ArrayLenth) 
        pHArrayIndex=0;
      phVoltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5 * phVoltage + Offset;
      // =========EC==========
      ecArray[ecArrayIndex++] = analogRead(ECSensorPin);
      if(ecArrayIndex == ArrayLenth) 
        ecArrayIndex=0;
      ecVoltage = avergearray(ecArray, ArrayLenth);
      samplingTime = millis();
  }
  if(millis()-lastSendTime > publishInterval)
  {
//      Serial.println(temperature);
      load(temperature);
      client.publish("Temperature", chartemp);
      load(pHValue);
      client.publish("PH", chartemp);
      load(ecVoltage);
      client.publish("TDS", chartemp);
      lastSendTime=millis();
  }


//  delay(1000); //just here to slow down the output so it is easier to read
  client.loop();  
}


void connectWifi(){
  if (wifi.init()){
//      Serial.println("CC3000 initialization complete");
  } else{
//      Serial.println("Something went wrong during CC3000 init!"); 
  }
  if(!wifi.connect(ap_ssid, ap_security, ap_password, timeout)) {
//    Serial.println("Error: Could not connect to AP");
  }  
}


void reconnect() {
  // conenct wifi / maybe wifi is disconnectd
  connectWifi();
  // Loop until we're reconnected
  while (!client.connected()) {
//    Serial.print("Try Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId, username, password)) {
//      Serial.println("mqtt connected");
//      client.subscribe("inTopic");
    } else {
//      Serial.print("failed, rc=");
//      Serial.print(client.state());
//      Serial.println(" try again in 5 seconds");
//      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
//  Serial.print("Message arrived [");
//  Serial.print(topic);
//  Serial.print("] ");
//  for (int i=0;i<length;i++) {
////    Serial.print((char)payload[i]);
//  }
//  float myFloat = atof((char)payload[i]);
}

// ===========================sensor===========================
float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];
  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }
  if ( OneWire::crc8( addr, 7) != addr[7]) {
//      Serial.println("CRC is not valid!");
      return -1000;
  }
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
//      Serial.print("Device is not recognized");
      return -1000;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }  
  ds.reset_search();
  byte MSB = data[1];
  byte LSB = data[0];
  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  return TemperatureSum;
}


double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
//    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

void load(float value){
  String stemp = String(clientId);
  stemp += value;
  stemp.toCharArray(chartemp, 20);
}



