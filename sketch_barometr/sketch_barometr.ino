#include <Wire.h>
#include <SPI.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>  // предоставляет специальные процедуры Wi-Fi для ESP8266, которые мы вызываем для подключения к сети
#include <WiFiUdp.h>      // обрабатывает отправку и получение пакетов UDP
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <iarduino_RTC.h>

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args) write(args);
#else
#define printByte(args) print(args, BYTE);
#endif
uint8_t bell[8] = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};
uint8_t note[8] = {0x2, 0x3, 0x2, 0xe, 0x1e, 0xc, 0x0};
uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
uint8_t duck[8] = {0x0, 0xc, 0x1d, 0xf, 0xf, 0x6, 0x0};
uint8_t check[8] = {0x0, 0x1, 0x3, 0x16, 0x1c, 0x8, 0x0};
uint8_t cross[8] = {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0};
uint8_t retarrow[8] = {0x1, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4};
float altuf;

boolean wake_flag, move_arrow;
int sleep_count, bat, hh, humidity, pressurehPa, angle, delta, last_angle = 90;
String line;
float windspeed, a, b, tempC, tempK, pres, k = 0.8;
unsigned long aver_pressure, pressure_array[6],
    time_array[6];
  double pressure, aver_altu, altu,altu_sens;
unsigned long sumX, sumY, sumX2, sumXY, delayTime;

const char* ssid = "SSID";       //ИМЯ
const char* password = "PASS";//ПАРОЛЬ

const char* host = "api.openweathermap.org";
int stat = 20;
bool pusk = true;
bool WIFI = true;
bool status;
bool jkl = 0;
String hh1;
int mm;
int ss;

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

iarduino_RTC watch(RTC_DS3231);
Adafruit_BME280 bme;
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 14400, 60000);
String arr_days[] = {"Sunday",   "Monday", "Tuesday", "Wednesday",
                     "Thursday", "Friday", "Saturday"};
String date_time;

void setup() {
  Serial.begin(115200);
  timeClient.begin();
  watch.begin();
  lcd.init();
  lcd.backlight();

  Serial.println(F("BME280 test"));
  lcd.setCursor(0, 0);
  lcd.print("I2C test");
  delay(500);

  status = bme.begin();
  if (!status) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("check I2C");
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  lcd.clear();
  lcd.print("success");
  delay(500);
  pinMode(A0, INPUT);

  lcd.clear();

  lcd.print("WIFI connection");

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    if (stat > 0) {
      delay(500);
      Serial.print(".");
      stat = stat - 1;
    } else {
      break;
    }
    if (stat == 0) {
      WIFI = 0;

    }
  }
 
  if (WIFI == 1) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    lcd.clear();
    lcd.print("success");
    delay(500);
    jsonGet();
  }
  if (WIFI == 0) {
    Serial.println("NO CONNECTION WIFI");
    lcd.clear();
    lcd.print("NO CONNECTION WIFI");
  }
  pressure =  aver_sens();        // найти текущее давление по среднему арифметическому
  for (byte i = 0; i < 6; i++) {  // счётчик от 0 до 5
    pressure_array[i] = pressure;  // забить весь массив текущим давлением
    time_array[i] = i;  // забить массив времени числами 0 - 5
  }
  lcd.clear();
 


    altuf = bme.readAltitude(SEALEVELPRESSURE_HPA);
}
void loop() {
  String ss1 = watch.gettime("s");
  String uio = watch.gettime("i");
  pressure =  aver_sens();
  if (uio == "10") {
    jkl = 1;
  }
  if (uio == "20") {
    jkl = 1;
  }
  if (uio == "30") {
    jkl = 1;
  }
  if (uio == "40") {
    jkl = 1;
  }
  if (uio == "50") {
    jkl = 1;
  }
  if (uio == "00") {
    jkl = 1;
  }

  if (jkl == 1) {
    Serial.println("pusk");

    if (WIFI == 0) {
      Serial.println();
      Serial.println();
      Serial.print("Connecting to ");
      Serial.println(ssid);

      WiFi.begin(ssid, password);

      stat = 20;
    }
    while (WiFi.status() != WL_CONNECTED) {
      if (stat > 0) {
        delay(500);
        Serial.print(".");
        stat = stat - 1;
      } else {
        break;
      }
    }
    if (stat == 0) {
      WIFI = 0;
    }
    if (stat > 1) {
      WIFI = 1;
    }
    if (WIFI == 1) {
      jsonGet();

      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    }
      // найти текущее давление по среднему арифметическому
    for (byte i = 0; i < 5;
         i++) {  // счётчик от 0 до 5 (да, до 5. Так как 4 меньше 5)
      pressure_array[i] = pressure_array[i + 1];  // сдвинуть массив давлений
      // КРОМЕ ПОСЛЕДНЕЙ ЯЧЕЙКИ на
      // шаг назад
      Serial.println(pressure_array[i]);
    }
    pressure_array[5] =
        pressure;  // последний элемент массива теперь - новое давление
    Serial.println(pressure);
    sumX = 0;
    sumY = 0;
    sumX2 = 0;
    sumXY = 0;
    for (int i = 0; i < 6; i++) {  // для всех элементов массива
      sumX += time_array[i];
      sumY += (long)pressure_array[i];
      sumX2 += time_array[i] * time_array[i];
      sumXY += (long)time_array[i] * pressure_array[i];
    }
    a = 0;
    a = (long)6 * sumXY;  // расчёт коэффициента наклона приямой
    a = a - (long)sumX * sumY;
    a = (float)a / (6 * sumX2 - sumX * sumX);
    delta = a * 6;  // расчёт изменения давления
    jkl = 0;
    delay(60000);
  }

  Serial.print("delta: ");
  Serial.println(delta);

  StaticJsonBuffer<2000> jsonBuffer;  /// буфер на 2000 символов
  JsonObject& root = jsonBuffer.parseObject(line);  // скармиваем String
  if (pusk == true) {
    if (!root.success()) {
      Serial.println("parseObject() failed");  // если ошибка, сообщаем об этом
      jsonGet();
      WIFI = 0;
      lcd.clear();
      lcd.setCursor(0, 0);

      lcd.print("NO CONNECT WIFI");
      delay(50);
    }
    pusk = false;
  }
  timeClient.update();

  hh1 = watch.gettime("H");

  if (hh1 == "00") {
    lcd.noBacklight();
  }
  if (hh1 == "01") {
    lcd.noBacklight();
  }
  if (hh1 == "02") {
    lcd.noBacklight();
  }
  if (hh1 == "03") {
    lcd.noBacklight();
  }
  if (hh1 == "04") {
    lcd.noBacklight();
  }
  if (hh1 == "05") {
    lcd.noBacklight();
  }
  if (hh1 == "06") {
    lcd.backlight();
  }
  if (hh1 == "07") {
    lcd.backlight();
  }
  if (hh1 == "08") {
    lcd.backlight();
  }
  if (hh1 == "09") {
    lcd.backlight();
  }
  if (hh1 == "10") {
    lcd.backlight();
  }
  if (hh1 == "11") {
    lcd.backlight();
  }
  if (hh1 == "12") {
    lcd.backlight();
  }
  if (hh1 == "13") {
    lcd.backlight();
  }
  if (hh1 == "14") {
    lcd.backlight();
  }
  if (hh1 == "15") {
    lcd.backlight();
  }
  if (hh1 == "16") {
    lcd.backlight();
  }
  if (hh1 == "17") {
    lcd.backlight();
  }
  if (hh1 == "18") {
    lcd.backlight();
  }
  if (hh1 == "19") {
    lcd.backlight();
  }
  if (hh1 == "20") {
    lcd.backlight();
  }
  if (hh1 == "21") {
    lcd.backlight();
  }
  if (hh1 == "22") {
    lcd.noBacklight();
  }
  if (hh1 == "23") {
    lcd.noBacklight();
  }

  Serial.print("BUTTON:");
  if (analogRead(A0) > 1000) {
    Serial.println("pressed");
    bat = bat + 1;
    lcd.clear();
  }
  if (analogRead(A0) < 500) {
    Serial.println("not pressed");
  }

  if (bat > 2) {
    bat = 0;
  }

  Serial.print("кнопка нажата:");
  Serial.println(bat);
  Serial.println();

  hh = timeClient.getHours();
  mm = timeClient.getMinutes();
  ss = timeClient.getSeconds();

  if (WIFI == 1) {
    if (ss1 == "00") {
      watch.settime(ss, mm, hh);
    }

    Serial.println("Данные с интернета:");
    Serial.print("Время:");
    Serial.print(timeClient.getFormattedTime());
    Serial.print(" ");
    int day = timeClient.getDay();
    Serial.println("'" + arr_days[day] + "'");

    String name = root["name"];  // достаем имя,
    Serial.print("Город:");
    Serial.println(name);

    tempK = root["main"]["temp"];  // достаем температуру из структуры main
    tempC = tempK - 273.15;  // переводим кельвины в цельси
    Serial.print("Темпераутра: ");
    Serial.print(tempC);  // отправляем значение в сериал
    Serial.println(" C");

    pressurehPa = root["main"]["pressure"];
    pres = pressurehPa / 1.333;
    Serial.print("Давление: ");
    Serial.print(pres);
    Serial.println(" mmHc");

    humidity = root["main"]["humidity"];
    Serial.print("Влажность: ");
    Serial.print(humidity);
    Serial.println(" %");

    windspeed = root["wind"]["speed"];
    Serial.print("Скорость ветра: ");
    Serial.print(windspeed);
    Serial.println(" m/s");
  }
  Serial.println();

  Serial.println("Данные с бортовых датчиков:");
  Serial.print("Время:");
  Serial.println(watch.gettime("d-m-Y, H:i:s, D"));

  Serial.print("Температура: ");
  Serial.print(bme.readTemperature());
  Serial.println(" C");

//  pressure = aver_sens();
  Serial.print("Давление: ");
  Serial.print(pressure / 133.33);
  Serial.println(" mmHc");

unsigned long filter_timer;




  //altu = 0;
  //for (byte i = 0; i < 50; i++) {
  //  altu = (bme.readAltitude(SEALEVELPRESSURE_HPA) + altu);
 //}
 //altuf = altu / 50;
 //Serial.println(altu);

 if (millis() - filter_timer > 5) {
    filter_timer = millis();    // просто таймер
    // читаем значение (не обязательно с аналога, это может быть ЛЮБОЙ датчик)
    altu = bme.readAltitude(SEALEVELPRESSURE_HPA);
     //основной алгоритм фильтрации. Внимательно прокрутите его в голове, чтобы понять, как он работает
    altuf = (altu * 0.03) + (altuf * (1 - 0.03));
    // для примера выведем в порт
   // Serial.println(val_f);
  }
 
  Serial.print("Высота над уровнем моря: ");
  Serial.print(altuf, 2);
  Serial.println(" m");

  Serial.print("Влажность: ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();

  if (bat == 1) {
    lcd.setCursor(0, 1);
    lcd.print("H:");
    lcd.print(altuf, 2);
    lcd.print("m ");
    lcd.print("W:");
    lcd.print(delta);
    lcd.setCursor(0, 0);
    lcd.print(watch.gettime("H:i:s"));
    lcd.print(" P:");
    lcd.print((bme.readPressure() / 133.33), 0);
    lcd.print("mm ");
  }

  if (bat == 2) {
    if (WIFI == 1) {
      lcd.setCursor(0, 0);
      lcd.print("T:");
      lcd.print(tempC, 0);
      lcd.printByte(223);
      lcd.print("C ");
      lcd.print("F:");
      lcd.print(humidity);
      lcd.print("% ");
      lcd.setCursor(0, 1);
      lcd.print("S:");
      lcd.print(windspeed, 0);
      lcd.print("m/c ");
      lcd.print("P:");
      lcd.print(pres, 0);
      lcd.print("mm");
    }
    if (WIFI == 0) {
      bat = 0;
    }
  }
  if (bat == 0) {
    lcd.setCursor(0, 0);
    lcd.print(watch.gettime("d/m/Y H:i"));
    lcd.setCursor(0, 1);
    lcd.printByte(165);
    lcd.print("T:");
    lcd.print(bme.readTemperature(), 0);
    lcd.printByte(223);
    lcd.print("C ");
    lcd.printByte(165);
    lcd.print("F:");
    lcd.print(bme.readHumidity(), 0);
    lcd.print("%");
  }
  // delay(1000);
}



long aver_sens() {
  pressure = 0;
  for (byte i = 0; i < 10; i++) {
    pressure += bme.readPressure();
  }
  aver_pressure = pressure / 10;
  return aver_pressure;
}

void jsonGet() {
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  client.println(
      "GET /data/2.5/weather?id=479123&appid=26d75c796a87d886bd32e262707a10d7 "   //ID ГОРОДА
      "HTTP/1.1");
  client.println("Host: api.openweathermap.org");
  client.println("Connection: close");
  client.println();
  delay(1500);
  while (client.available()) {
    line = client.readStringUntil('\r');
  }
  Serial.print(line);
  Serial.println();
  Serial.println("closing connection");
}
