//library for lora
#include <SPI.h>
#include <LoRa.h>
//library oled 
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#include <elapsedMillis.h>
#include <SoftwareSerial.h>
#include <DHT.h>

#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

#define BAND 866E6

#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define soil 34
#define DHTPin 2          
#define DHTTYPE DHT21      
DHT dht(DHTPin, DHTTYPE);  

//relay
const int relay1 = 22; //lampu
const int relay2 = 21; //kipas
const int relay3 = 23; //selonoid
const int relay4 = 17; //pompa

elapsedMillis readSensorMillis;
elapsedMillis readUltrasonicMillis;
elapsedMillis readSoilMillis;
elapsedMillis readDHTMillis;
elapsedMillis readNPKMillis;
elapsedMillis OLEDPrintMillis;

int distance, SoilMoistValue, SoilMoistPercent, dataConduct, datapH, dataNitrogen, dataPhosphorus, dataPotassium, Dis;
float Hum, Temp, dataSoil, dataTemp;
String Moistmsg, Tempmsg, Hummsg, pHmsg, Ecmsg, Nitromsg, Phosmsg, Kalmsg, Dismsg;

unsigned char data_buffer[4] = {0};
unsigned char CS;
//unsigned long readSensorInterval = 6000;
unsigned long readUltrasonicInterval = 100;
unsigned long OLEDPrintInterval = 2000;
unsigned long readSoilInterval = 500;
unsigned long readDHTInterval = 700;
unsigned long readNPKInterval = 400; 

SoftwareSerial NPKSerial(35, 25);  // RX, TX
SoftwareSerial UltrasonicSerial(13, 0);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

void setup()
{
  Serial.begin(115200);
  NPKSerial.begin(4800);
  UltrasonicSerial.begin(9600);

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  initOLED();
  initLoRa();
  initSensor();
}

void loop(){
  int rssis = LoRa.packetRssi();
  String rssiss = String(rssis);
  dataSender(Dismsg, Moistmsg, Hummsg, Tempmsg, Nitromsg, Phosmsg, Kalmsg, Ecmsg, pHmsg, rssiss);

  if (readUltrasonicMillis >= readUltrasonicInterval){
    initUltrasonic();
    kondisiUltrasonik();
    readUltrasonicMillis = 0; 
  }

  if (readSoilMillis >= readSoilInterval){
    initSoil();
    kondisiSoil();
    readSoilMillis = 0;
  }

  if (readDHTMillis >= readDHTInterval){
    initDHT();
    kondisiDHT();
    readDHTMillis = 0;
  }

  if (readNPKMillis >= readNPKInterval){
    initNPK();
    readNPKMillis = 0;
  }

  if (OLEDPrintMillis >= OLEDPrintInterval){
    OLEDPrint(); 
    OLEDPrintMillis = 0;
  }

  Serial.print("Soil Moist = ");
  Serial.print(SoilMoistPercent);
  Serial.println("%");

  Serial.print("Humidity: ");
  Serial.print(Hum);
  Serial.println("%");
  Serial.print("Temperature: ");
  Serial.print(Temp);
  Serial.println(" Celsius");

  Serial.print("Soil Conductivity: ");
  Serial.println(dataConduct);
  Serial.print("Soil pH: ");
  Serial.println(datapH);
  Serial.print("Nitrogen: ");
  Serial.println(dataNitrogen);
  Serial.print("Phosphorus: ");
  Serial.println(dataPhosphorus);
  Serial.print("Potassium: ");
  Serial.println(dataPotassium);

  Serial.print("distance: ");
  Serial.print(Dis);
  Serial.println(" cm");
  Serial.println("           ");
}

void initSensor(){
  dht.begin();
  pinMode(soil, INPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
}

void initLoRa(){
 //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print("LoRa Initializing OK!");
  display.display();
  delay(2000);
}

void initOLED(){
 Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(30, 30);
  display.print("LORA SENDER ");
  display.display();

  Serial.println("LoRa Sender");
}

void initSoil(){
  SoilMoistValue = analogRead(soil); // put Sensor insert into soil
  SoilMoistPercent = map(SoilMoistValue,  2775, 4095, 100, 0);
  Moistmsg = String (SoilMoistPercent);
}

void initDHT(){
  Hum = dht.readHumidity();
  Temp = dht.readTemperature();
  Hummsg = String (Hum);
  Tempmsg = String (Temp);
}

void initNPK(){
  byte queryData[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
  byte receivedData[19];

  NPKSerial.write(queryData, sizeof(queryData));  // Send the query data to the NPK sensor
  delay(1000);  // Wait for 1 second

  if (NPKSerial.available() >= sizeof(receivedData)) {  // Check if there are enough bytes available to read
    NPKSerial.readBytes(receivedData, sizeof(receivedData));  // Read the received data into the receivedData array

    // Parse and print the received data in decimal format
    unsigned int soilHumidity = (receivedData[3] << 8) | receivedData[4];
    unsigned int soilTemperature = (receivedData[5] << 8) | receivedData[6];
    unsigned int soilConductivity = (receivedData[7] << 8) | receivedData[8];
    unsigned int soilPH = (receivedData[9] << 8) | receivedData[10];
    unsigned int nitrogen = (receivedData[11] << 8) | receivedData[12];
    unsigned int phosphorus = (receivedData[13] << 8) | receivedData[14];
    unsigned int potassium = (receivedData[15] << 8) | receivedData[16];

// Perhitungan hasil kalibrasi sensor
    dataSoil = soilHumidity / 10.0;
    dataTemp = soilTemperature / 10.0;
    dataConduct = soilConductivity;
    datapH = soilPH / 10.0;
    dataNitrogen = nitrogen;
    dataPhosphorus = phosphorus / 2 ;
    dataPotassium = potassium; 

    Ecmsg = String (dataConduct);
    pHmsg = String (datapH);
    Nitromsg = String (dataNitrogen);
    Phosmsg = String (dataPhosphorus);
    Kalmsg = String (dataPotassium);
  }
}

void initUltrasonic(){
    if (UltrasonicSerial.available() > 0) {
 
    delay(4);
 
    // Check for packet header character 0xff
    if (UltrasonicSerial.read() == 0xff) {
      // Insert header into array
      data_buffer[0] = 0xff;
      // Read remaining 3 characters of data and insert into array
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = UltrasonicSerial.read();
      }
 
      //Compute checksum
      CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
      // If checksum is valid compose distance from data
      if (data_buffer[3] == CS) {
        distance = (data_buffer[1] << 8) + data_buffer[2];
        Dis = distance/10;
        
        Dismsg = String (Dis);
      }
    }
  }
}

void dataSender(String sensor1, String sensor2, String sensor3, String sensor4, String sensor5, String sensor6, String sensor7, String sensor8,
                String sensor9, String sensor10){
  LoRa.beginPacket ();
  LoRa.print (sensor1);
  LoRa.print ("#");
  LoRa.print (sensor2);
  LoRa.print ("#");
  LoRa.print (sensor3);
  LoRa.print ("#");
  LoRa.print (sensor4);
  LoRa.print ("#");
  LoRa.print (sensor5);
  LoRa.print ("#");
  LoRa.print (sensor6);
  LoRa.print ("#");
  LoRa.print (sensor7);
  LoRa.print ("#");
  LoRa.print (sensor8);
  LoRa.print ("#");
  LoRa.print (sensor9);
  LoRa.print ("#");
  LoRa.print (sensor10);
  LoRa.print ("#");
  LoRa.endPacket ();
}

void OLEDPrint(){
  display.clearDisplay(); // clear display        // set text size

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Distance: ");
  display.print(Dis);
  display.println(" Cm");
  display.display();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.print("Soil Moist: ");
  display.print(SoilMoistPercent);
  display.println(" %");

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 20);
  display.print("Humidity: ");
  display.print(Hum);
  display.println(" %");

  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("Temperature: ");
  display.print(Temp);
  display.println(" C");
  display.display();

  display.setTextSize(1);
  display.setCursor(0, 45);
  display.print("N: ");
  display.print(dataNitrogen);
  display.display();
  
  display.setTextSize(1);
  display.setCursor(45, 45);
  display.print("P: ");
  display.print(dataPhosphorus);
  display.display();

  display.setTextSize(1);
  display.setCursor(90, 45);
  display.print("K: ");
  display.print(dataPotassium);
  display.display();

  display.setTextSize(1);
  display.setCursor(15, 55);
  display.print("EC: ");
  display.print(dataConduct);
  display.display();

  display.setTextSize(1);
  display.setCursor(75, 55);
  display.print("pH: ");
  display.print(datapH);
  display.display();
}

void kondisiDHT() {
  //suhu
  if (Temp <= 39) {
    digitalWrite(relay1, LOW);
    //digitalWrite(relay2, HIGH);
  }
  else if (Temp >= 42) {
    digitalWrite(relay2, HIGH);
    //digitalWrite(relay1, HIGH);
  }
  //humidity
  if (Hum <= 75) {
    digitalWrite(relay2, LOW);
  }
  else if (Hum >= 85) {
    digitalWrite(relay2, HIGH);
  }
}

void kondisiSoil(){
  //kelembapan tanah
  if (SoilMoistPercent < 60) {
    digitalWrite(relay3, LOW);
  }
  else if (SoilMoistPercent > 80) {
    digitalWrite(relay3, HIGH);
  }
}

void kondisiUltrasonik(){
  //ultrasonik
  if (Dis > 65) {
    digitalWrite(relay4, LOW);
  }
  else if (Dis < 25) {
    digitalWrite(relay4, HIGH);
  }
}
