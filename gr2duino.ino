// GR2DUINO - скетч для превращения гидрометрической вертушки ГР-21
// в настоящий измеритель скорости потока!
// Автор - к.г.н. Всеволод Морейдо
// (с) 2019

// библиотека для работы с дисплеем
#include <TroykaTextLCD.h>
// библиотека для работы с датчиком тока (Troyka-модуль)
#include <TroykaCurrent.h>
// создаем объект для работы с дисплеем
TroykaTextLCD lcd; 
// создаём объект для работы с датчиком тока
// и передаём ему номер пина выходного сигнала
ACS712 sensorCurrent(A0);
//Пин подключения кнопки начала измерения
#define RES 11 
// продолжительность измерения 10000 мс =  1 сек
const unsigned long sampleTime = 100000;
// переменная для состояния кнопки
boolean butt;
// переменная для памяти состояния кнопки
boolean butt_flag = 0;

void setup() {
      // открываем последовательный порт
      Serial.begin(9600);
      // устанавливаем количество столбцов и строк экрана
      lcd.begin(16, 2);
      // устанавливаем контрастность в диапазоне от 0 до 63
      lcd.setContrast(27);
      // устанавливаем яркость в диапазоне от 0 до 255
      lcd.setBrightness(200);
      // инициализация кнопки
      pinMode(RES, INPUT_PULLUP);
  
}

void loop() {
  delay(100);
  // считываем текущее положение кнопки
  butt = !digitalRead(11); 

  if (butt == 1 && butt_flag == 1){ // если кнопка нажата
    butt_flag = 1;
    float current_speed = getSpeed(); // запускаем измерение
    lcd.clear();
    displaySpeed(current_speed);      // выводим показания на дисплей
  } else if (butt == 0 && butt_flag == 0) { // если кнопка не нажата и никогда не была (первое включение), то выводим начальный экран
    butt_flag = 1;                         
    lcd.clear();
    lcd.setCursor(0, 0);
    // печатаем первую строку
    lcd.print(F("Press button to"));
    // печатаем вторую строку
    lcd.setCursor(0, 1);
    lcd.print(F("start measuring!"));
  }
}
// функция измерения тока на клеммах вертушки, обрабатывает изменение тока любой продолжительности (медленное течение) как один оборот
float getSpeed(){
  int count = 0;
  boolean countFlag = LOW;
  unsigned long currentTime = 0;
  unsigned long startTime = millis();
  while (currentTime <= sampleTime){ // цикл по времени измерения
    if (sensorCurrent.readCurrentDC() > 0.5)
    {
      countFlag = HIGH;
      Serial.println(sensorCurrent.readCurrentDC());
    }
    if (sensorCurrent.readCurrentDC() <= 0.5 && countFlag == HIGH)
    {
      count++;
      countFlag=LOW;
      Serial.println(count);
    }
    currentTime = millis() - startTime;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    // печатаем первую строку
    lcd.print(F("Measuring..."));
    // печатаем вторую строку
    lcd.setCursor(0, 1);
    lcd.print(currentTime / 1000);
    lcd.setCursor(10, 1);
    lcd.print(count);
  }
  
  float countRpm = (float(count) * 20 / float(sampleTime)) * 1000.; // расчет оборотов в секунду
  float flowSpeed = 0.004 + 0.219 * countRpm; // расчет скорости течения по уравнению конкретной вертушки
  Serial.print(F("RPM count: "));
  Serial.println(countRpm);
  Serial.print(F("Flow speed: "));
  Serial.println(flowSpeed);
  return flowSpeed;
}
// функция вывода измерения на дисплей
void displaySpeed(float flowSpeed) {
  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.print(F("Flow speed: "));
  lcd.setCursor(0, 1);
  lcd.print(flowSpeed, 3);
  lcd.setCursor(10, 1);
  lcd.print(F(" m/s"));
}
