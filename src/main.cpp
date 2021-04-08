#include <Arduino.h>
//Sample using LiquidCrystal library
#include <LiquidCrystal.h>
#include <Adafruit_BMP280.h>
// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

int l=1;

Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();


// read the buttons
int read_LCD_buttons()
{
 adc_key_in = analogRead(0);      // read the value from the sensor
 // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
 // we add approx 50 to those values and check to see if we are close
 if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
 // For V1.1 us this threshold
 if (adc_key_in < 50)   return btnRIGHT;
 if (adc_key_in < 250)  return btnUP;
 if (adc_key_in < 450)  return btnDOWN;
 if (adc_key_in < 650)  return btnLEFT;
 if (adc_key_in < 850)  return btnSELECT;

 // For V1.0 comment the other threshold and use the one below:
/*
 if (adc_key_in < 50)   return btnRIGHT;
 if (adc_key_in < 195)  return btnUP;
 if (adc_key_in < 380)  return btnDOWN;
 if (adc_key_in < 555)  return btnLEFT;
 if (adc_key_in < 790)  return btnSELECT;
*/


 return btnNONE;  // when all others fail, return this...
}

void setup()
{
 pinMode(10,OUTPUT);
 digitalWrite(10,HIGH); 
 pinMode(3,OUTPUT);
 digitalWrite(3,LOW);
 lcd.begin(16, 2);             
 lcd.setCursor(0,0);
 lcd.print("Push "); 
   if (!bmp.begin(0x76)) {
    lcd.print(F("ERROR"));
    while (1) delay(10);
  }
 bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, // Режим работы
                   Adafruit_BMP280::SAMPLING_X2, // Точность изм. температуры
                   Adafruit_BMP280::SAMPLING_X16, // Точность изм. давления
                   Adafruit_BMP280::FILTER_X16, // Уровень фильтрации
                   Adafruit_BMP280::STANDBY_MS_500); // Период просыпания, мСек   
}

void loop()
{
 lcd.setCursor(9,1);            // move cursor to second line "1" and 9 spaces over
 lcd.print(millis()/1000);      // display seconds elapsed since power-up
  
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  if (temp_event.temperature>24){digitalWrite(3,LOW);}
  if (temp_event.temperature<23){digitalWrite(3,HIGH);}


 lcd.setCursor(10,0);
 //Serial.print(temp_event.temperature);
 lcd.print(temp_event.temperature);




 lcd.setCursor(0,1);            // move to the begining of the second line
 lcd_key = read_LCD_buttons();  // read the buttons

 switch (lcd_key)               // depending on which button was pushed, we perform an action
 {
   case btnRIGHT:
     {
     lcd.print("RIGHT ");
     break;
     }
   case btnLEFT:
     {
     lcd.print("LEFT   ");
     break;
     }
   case btnUP:
     {
     lcd.print("UP    ");
     break;
     }
   case btnDOWN:
     {
     lcd.print("DOWN  ");
     break;
     }
   case btnSELECT:
     {
     lcd.print("SELECT");
     l==1?l=0:l=1;
     digitalWrite(10,l);
     break;
     }
     case btnNONE:
     {
     lcd.print("NONE  ");
     break;
     }
 }

}
 





/*
#include <Adafruit_BMP280.h> // Библиотека для работы с датчиком BMP280

Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 Starting"));
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);
  if (!bmp.begin(0x76)) {
    Serial.println(F("BMP280 ERROR!!"));
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, // Режим работы
                   Adafruit_BMP280::SAMPLING_X2, // Точность изм. температуры
                   Adafruit_BMP280::SAMPLING_X16, // Точность изм. давления
                   Adafruit_BMP280::FILTER_X16, // Уровень фильтрации
                   Adafruit_BMP280::STANDBY_MS_500); // Период просыпания, мСек  
 //delay(500);
}


void loop() {
sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure/1.33);
  Serial.println(" mm");

  Serial.println();

  
  if (temp_event.temperature>23){
    digitalWrite(13,HIGH);
    digitalWrite(2,LOW);
    Serial.println("ON");
  }
  else{
    digitalWrite(13,LOW);
    digitalWrite(2,HIGH);
    Serial.println("OFF");
  }
  delay(2000);
}*/