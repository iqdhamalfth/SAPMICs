#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <elapsedMillis.h>
#include <Adafruit_SSD1306.h>
#include <LiquidCrystal_I2C.h>

//define the pins used by the LoRa transceiver module
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

// Alamat LCD 20x4
LiquidCrystal_I2C lcd(0x27,20,4); //LCD address to 0x27

elapsedMillis UltrasonicPrintMillis;
elapsedMillis SoilPrintMillis;
elapsedMillis DHTPrintMillis;
elapsedMillis NPKPrintMillis;

unsigned long NPKPrintInterval = 10000;
unsigned long UltrasonicPrintInterval = 18000;
unsigned long SoilPrintInterval = 25000;
unsigned long DHTPrintInterval = 32000;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

String Dis, Moist, Hum, Temp, Nitro, Phos, Kal, Ec, pH, rssi; 

void setup() { 
  //initialize Serial Monitor
  Serial.begin(115200);
  
  lcd.init();
  lcd.backlight();  
  initLCD();

  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  
  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA RECEIVER ");
  display.display();

  Serial.println("LoRa Receiver Test");
  
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0,10);
  display.println("LoRa Initializing OK!");
  display.display();  
}

void loop() {
  if (NPKPrintMillis >= NPKPrintInterval){
    LcdPrintNPK();    
    NPKPrintMillis = 0;  
  } 

  if (UltrasonicPrintMillis >= UltrasonicPrintInterval){
    LcdPrintUltrasonic();    
    UltrasonicPrintMillis = 0;  
  }

  if (SoilPrintMillis >= SoilPrintInterval){
    LcdPrintSoil();    
    SoilPrintMillis = 0;  
  }

  if (DHTPrintMillis >= DHTPrintInterval){
    LcdPrintDHT();    
    DHTPrintMillis = 0;  
  }
 
  //try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.println("Received packet ");
    while (LoRa.available()) {
      Dis = LoRa.readStringUntil('#');
      Moist = LoRa.readStringUntil('#');
      Hum = LoRa.readStringUntil('#');
      Temp = LoRa.readStringUntil('#');
      Nitro = LoRa.readStringUntil('#');
      Phos = LoRa.readStringUntil('#');
      Kal = LoRa.readStringUntil('#'); 
      Ec = LoRa.readStringUntil('#');
      pH = LoRa.readStringUntil('#');
      rssi = LoRa.readStringUntil('#');
    }
    //print RSSI of packet
    int rssi = LoRa.packetRssi(); 
  }
}

void initLCD(){
    lcd.setCursor(0,0);
    lcd.print("    LoRa Receiver   ");
    lcd.setCursor(0,1);
    lcd.print("   Waiting For The  ");
    lcd.setCursor(0,2);
    lcd.print("       Package      ");
    lcd.setCursor(0,3);
    lcd.print("                    ");
}

void LcdPrintUltrasonic(){
// LCD Print Sensor Ina219
  lcd.setCursor(0,0);
  lcd.print(" Sensor Ultrasonic  ");
  lcd.setCursor(0,1);
  lcd.print("Distance :"); lcd.print(Dis); lcd.print(" Cm"); lcd.print("    ");
  lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("                    ");
}

void LcdPrintSoil(){
// LCD Print Sensor Ina219
  lcd.setCursor(0,0);
  lcd.print(" Sensor Soil Moist  ");
  lcd.setCursor(0,1);
  lcd.print("Soil Moist :"); lcd.print(Moist); lcd.print(" %"); lcd.print(" ");
  lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("                    ");
}

void LcdPrintDHT(){
// LCD Print Sensor Ina219
  lcd.setCursor(0,0);
  lcd.print("     Sensor DHT    ");
  lcd.setCursor(0,1);
  lcd.print("Humidity   :"); lcd.print(Hum); lcd.print(" %"); lcd.print("  ");
  lcd.setCursor(0,2);
  lcd.print("Temperature:"); lcd.print(Temp); lcd.print(" C"); //lcd.print(" ");
  lcd.setCursor(0,3);
  lcd.print("                    ");
}

void LcdPrintNPK(){
// LCD Print Sensor Ina219
  lcd.setCursor(0,0);
  lcd.print("     Sensor NPK    ");
  lcd.setCursor(0,1);
  lcd.print("N  :"); lcd.print(Nitro); lcd.print("     P  :"); lcd.print(Phos); lcd.print("   ");
  lcd.setCursor(0,2);
  lcd.print("K  :"); lcd.print(Kal); lcd.print("    EC :"); lcd.print(Ec); lcd.print("  ");
  lcd.setCursor(0,3);
  lcd.print("pH :"); lcd.print(pH); lcd.print("             ");
}