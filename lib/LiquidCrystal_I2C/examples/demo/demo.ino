#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);

void setup()
{
  lcd.init();      
  lcd.backlight();
}


void loop()
{
  lcd.clear();
  lcd.setCursor(16+4,0);
  lcd.print("Welcome!");
  lcd.setCursor(16+1,1);
  lcd.print("Store duino.ru");
  scroll_left(16, 100);       // выталкиваем текст в видимую область экрана
  scroll_left(16, 50);        // выталкиваем текст за пределы экрана

  scroll_right(16, 100);        // выталкиваем текст вправо с паузой между шагами 100мс
  blink_led(3);                 // помигать подсветкой 3 раза
  scroll_left(16, 50);          // выталкиваем текст влево за пределы экрана
}

/***********************************************/
void scroll_left(uint8_t col, uint8_t del)
{
    for (int i=0; i<col; i++)
  {
    lcd.scrollDisplayLeft();  //толкнуть текст на 1 позицию влево
    delay(del);
  }
  delay(1000);
}

void scroll_right(uint8_t col, uint8_t del)
{
    for (int i=0; i<col; i++)
  {
    lcd.scrollDisplayRight(); //толкнуть текст на 1 позицию вправо
    delay(del);
  }
  delay(1000);
}

void blink_led(uint8_t num)
{
 for (int i=0; i<num; i++)
  {
    lcd.noBacklight();          //выключить подсветку
      delay(300);
    lcd.backlight();            //включить подсветку
      delay(500);
  }   
}

