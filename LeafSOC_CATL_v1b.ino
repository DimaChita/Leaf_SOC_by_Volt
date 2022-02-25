
// Дисплей для отображения остатка заряда по напряжению батареи в процентах, кВт и планируемого пробега в км для Nissan Leaf в кузове ze0 с пперепакованной ВВБ на модули CATL
// =======================================
// Основано на проектах "CANa Display for Nissan LEAF"  https://ev-olution.yolasite.com/CANa.php © Copyright EV-OLUTION и "LeafSOCdisplay" https://github.com/PaulKennett/LeafSOCdisplay
// =======================================
// DimaChita
// =======================================


#include <U8g2lib.h>                    // Объявление библиотеки для работы с дисплеем на Arduino, скачать библиотеку https://github.com/olikraus/u8g2 
#include <mcp_can.h>                    // Объявление библиотеки для работы с MCP2515 на Arduino, скачать библиотеку https://github.com/coryjfowler/MCP_CAN_lib 
#include <EEPROM.h>                     // Объявление стандартной библиотеки Arduino IDE для работы с EEPROM
//#include <SPI.h>                        // Объявление стандартной библиотеки Arduino IDE для работы с SPI
#include "battery_large.h"              // Большое контурное иозображение батареи (должно быть скопировано в папку с данным скетчем)
#include "battery_solid.h"              // Маленькое контурное иозображение батареи (должно быть скопировано в папку с данным скетчем)

#define VERSION " SOC by Volt "         // Информация выводимая на стартовый экран
#define MODEL   " NISSAN LEAF "
#define DATE    " ver 22.02.22"
#define KM_PER_KWH 6.5F                 // Расход км на кВтч по умолчанию
#define CAN0_INT 2                      // Определяем номер пина для MCP2515 на Arduino
#define LINE1 16                        // Определяем положение на вертикальной оси для первой строки на дисплее
#define LINE2 38                        // Определяем положение на вертикальной оси для второй строки на дисплее
#define LINE3 60                        // Определяем положение на вертикальной оси для третьей строки на дисплее
#define LINE4 64                        // Определяем положение на вертикальной оси для четвёртой строки на дисплее
#define V_TURTLE  321.62F               // Уровень напряжения ВВБ при выходе черепахи SOC 0%
#define V_SOC_5   334.70F               // Уровень напряжения ВВБ при SOC 5%
#define V_SOC_10  338.17F               // Уровень напряжения ВВБ при SOC 10%
#define V_SOC_25  346.94F               // Уровень напряжения ВВБ при SOC 25%
#define V_SOC_50  354.54F               // Уровень напряжения ВВБ при SOC 50%
#define V_SOC_75  371.83F               // Уровень напряжения ВВБ при SOC 75%
#define V_SOC_90  384.00F               // Уровень напряжения ВВБ при SOC 90%
#define V_SOC_100 393.81F               // Уровень напряжения ВВБ при SOC 100%
#define WT 36000                        // Ёмкость ВВБ в Ваттах доступная от полного заряда до черепахи, равна количеству Втч вошедших по счетчику - 10% потерь

long unsigned int rxId;                 // ID CAN сообщения
unsigned char len = 0;                  // Длинна данных CAN
unsigned char rxBuf[8];                 // Буфер для данных CAN

float km_per_kWh;                       // Расход км на кВт
byte km_per_dWh = KM_PER_KWH * 10;      // Расход км на кВтч в ватт часах для сохранения в EEPROM
float range;                            // Планируемый пробег в км
char buffer[4];                         // Символьный массив для сохранения результата преобразования
int i;                                  // Индекс
int Page = 1;                           // Номер страницы дисплея по умолчанию перед четением EEPROM
byte EEPROMaddr0 = 0;                   // Адроес для хранения номера страницы в EEPROM
byte EEPROMaddr1 = 1;                   // Адроес для хранения значения расхода в км на кВт в EEPROM
byte rawCCStatus1;                      // Признак включенного климат контроля для режима изменения настроек
byte rawCCVentIntake;                   // Признак включенной рециркуляции воздуха климат контроля для режима изменения настроек
byte rawCCVentTarget;                   // Признак включенного обдува в ноги для режима изменения настроек
byte rawCCFanSpeed;                     // Используется для смены страниц дисплея для режима изменения настроек
byte rawCCFanSpeed2;                    // Переменная для сравнения с rawCCFanSpeed и отслеживания нажатия
byte rawCCButtonPress;                  // Признак нажатия на кнопку климат контроля для режима изменения настроек
byte rawCCButtonPress2;                 // Переменная для сравнения с CCButtonPress и отслеживания нажатия
byte rawCCSetpoint;                     // Используется для установки настроек по умолчанию в EEPROM
byte rawGearPos = 1;                    // Признак селектора, по умолчанию Парковка P (1), используется для выбора опции настроек для редактирования в EEPROM
byte rawGearPos2 = 1;                   // Признак селектора для сравнения, по умолчанию Парковка P (1), используется для выбора опции настроек для редактирования в EEPROM
byte rawECOselected = 4;                // Признак селектора в режиме ECO, по умолчанию 4 выкл, используется для расчета расхода км на кВтч
byte ECOselected = 4;                   // Признак селектора в режиме ECO, по умолчанию 4 выкл, используется для сравнения при расчете расхода км на кВтч
byte  onHeater = 0;                     // Признак включенного отопителя салона для расчета расхода км на кВтч, по умолчанию 0, диапазон настройки 5.9 - 8.7 км на кВтч.
long CANInterval = 500;                 // Интервал повторения сообщения 0x79b, по умолчанию 500мс
long CANMillis = 0;                     // Время последнего отправленного сообщения 0x79b для запроса температуры батареи
long LCDMillis = 0;                     // Время последнего обновления дисплея в милисикундах
long LCDInterval = 1000;                // Интервал обновления дисплея
float BattVolts;                        // Общее напряжение батареи
//float prevBattVolts;                    // Предыдущее общее напряжение батареи
int16_t Amp;                            // Данные для расчета тока разряда/заряда батареи в А
float ActSocPct;                        // Уровень заряда батаери в процентах, рассчитанный по напряжению самой слабой ячейки
float WtPct;                            // Количество Ватт в одном проценте SOC
float Amps;                             // Текущий ток разряда/заряда батареи в А
//float prevAmps;                         // Предыдущий ток разряда/заряда батареи в А
float ActSockWh;                        // Остаток заряда в кВтч, рассчитанный по напряжению самой слабой ячейки
float kW;                               // Текущее потребление/расход батареи в кВтч
int BattTemp1;                          // Показания первого датчика температуры батареи
int BattTemp2;                          // Показания вторго датчика температуры батареи
int BattTemp3;                          // Показания третьего датчика температуры батареи
int BattTemp4;                          // Показания четвертого датчика температуры батареи
int MinBattTemp;                        // Минимальная температура батареи по датчикам
float BattTempFactor1 = 1;              // Коэфициэнт пересчета ёмкости батареи в зависимости от температуры
//float BattTempFactor2 = 0;              // Коэфициэнт снижения напряжения батареи в зависимости от температуры
int CANSt = 1;                          // Признак получения данных о температуре из CAN шины
//int CANSv = 0;                          // Признак получения данных о напряжении и токе ВВБ из CAN шины
//uint8_t RcvFrIdx;                       // Признак вывода на экран для отправки сообщения в CAN для запроса температуры
uint16_t CPVaverage;                    // Расчетное среднее напряжениее на элементах ВВБ
//float intResistance = 0;                // Расчетное внутреннее споротивление ВВБ
float VsocPct5;                         // Доля в вольтах от общего напряжения в одном проценте SOC в интервале от черепахи до 5%
float VsocPct10;                        // Доля в вольтах от общего напряжения в одном проценте SOC в интервале от 5% до 10%
float VsocPct25;                        // Доля в вольтах от общего напряжения в одном проценте SOC в интервале от 10% до 25%
float VsocPct50;                        // Доля в вольтах от общего напряжения в одном проценте SOC в интервале от 25% до 50%
float VsocPct75;                        // Доля в вольтах от общего напряжения в одном проценте SOC в интервале от 50% до 75%
float VsocPct90;                        // Доля в вольтах от общего напряжения в одном проценте SOC в интервале от 75% до 90%
float VsocPct100;                       // Доля в вольтах от общего напряжения в одном проценте SOC в интервале от 90% до 100%
float prevActSocPct = 0;                // Предыдущее значение % SOC
int intSocPct = 10000;                  // Минимальный интервал изменения % SOC в милисикундах, мнимальное время за которое ввб разрядится на 1%, может быть получено и расчитано из x1DC или из объёма ВВБ и максимального возможного потребления
long SocMillis = 0;                     // Время последнего расчета % SOC в милисикундах,для котроля интервала изменения

MCP_CAN CAN0(10);                               // Установка 10 пина для MCP CS

U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/ 7, /* dc=*/ 6, /* reset=*/ 5);      // Строка инициализации 1.3" OLED дисплея на чипе SH1106
// -------------------------------------------------------------------------------------------------


void(* resetFunc) (void) = 0;


void setup() 
{
  //SPI.begin();
  u8g2.begin();
  u8g2.enableUTF8Print();                       // Включение UTF8 шрифтов для вывода текста кириллицы на экран Arduino
  u8g2.setFont(u8g2_font_10x20_t_cyrillic);     // Установка шрифта с кириллицей
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) 
  {
    u8g2.firstPage();
    do {
      u8g2.setCursor(0, LINE1); 
      u8g2.print(VERSION);
      u8g2.setCursor(0, LINE2); 
      u8g2.print(MODEL);
      u8g2.setCursor(0, LINE3); 
      u8g2.print(DATE);
    } while ( u8g2.nextPage() );
    delay(1000);                                // Задержка для инициализации MCP2515
  } else {
    u8g2.firstPage();
    do {
      u8g2.setFontMode(0);                      // Инверсировать шрифт
      u8g2.setDrawColor(1);                     // Белый цвет экрана
      u8g2.drawBox(0, 0, 128, 64);              // Рисование белого прямоугольника
      u8g2.setDrawColor(0);
      u8g2.setCursor(31, 18); 
      u8g2.print("MCP2515");
      u8g2.setCursor(3, 40); 
      u8g2.print("(CAN module)");
      u8g2.setCursor(15, 61); 
      u8g2.print("ошибка MCP2515");
    } while ( u8g2.nextPage() );
    delay(10000);
  }
  
  CAN0.init_Mask(0, 0, 0x07ff0000);
  CAN0.init_Filt(0, 0, 0x07bb0000);
  CAN0.init_Filt(1, 0, 0x07bb0000);

  CAN0.init_Mask(1, 0, 0x07ff0000);
  CAN0.init_Filt(2, 0, 0x01db0000);
  CAN0.init_Filt(3, 0, 0x055b0000);
  CAN0.init_Filt(4, 0, 0x05bc0000);
  CAN0.init_Filt(5, 0, 0x05bc0000);
  CAN0.init_Filt(6, 0, 0x054a0000);
  CAN0.init_Filt(7, 0, 0x054b0000);
  CAN0.init_Filt(8, 0, 0x011a0000);
  
  CAN0.setMode(MCP_NORMAL);                     // Установка нормального режима MCP2515 для отправки и получения данных
  pinMode(CAN0_INT, INPUT);

  if (EEPROM.read(EEPROMaddr0) > 6 ) 
  {
    EEPROM.update(EEPROMaddr0, Page);
  }
  if (EEPROM.read(EEPROMaddr1) > 30  && EEPROM.read(EEPROMaddr1) < 100 ) 
  {
    km_per_kWh = (EEPROM.read(EEPROMaddr1)) / 10.0F;
  } else {
    EEPROM.update(EEPROMaddr1, km_per_dWh);     // Используется только при первом запуске этого кода на новом Arduino
  }
 
} //end setup loop


void loop() {

  if (!digitalRead(CAN0_INT)) 
  {                                             // Читаем и собираем данные из EV-CAN шины
    CAN0.readMsgBuf(&rxId, &len, rxBuf);       

   if ((rxId & 0x40000000) != 0x40000000) 
    {                                           // Читаем все данные кроме удалённых запросов и выбераем сообщения CAN, которые нас интересуют
     if (rxId == 0x54b) 
      {                                         // Читаем данные о настройках климат контроля (100ms)
        rawCCStatus1 = (rxBuf[0]);
        rawCCVentTarget = (rxBuf[2]);
        rawCCVentIntake  = (rxBuf[3]);
        rawCCFanSpeed = (rxBuf[4] >> 3);
        rawCCButtonPress = (rxBuf[7]);
      } 
      if (rxId == 0x11a) 
      {                                         // Читаем позицию селектора (10ms)
        rawGearPos = (rxBuf[0] >> 4);           // 1 = Парковка, 2 = Задняя, 3 = Нейтраль, 4 = Движение
        rawECOselected = (rxBuf[1] >> 4);       // 4 = ECO выкл 5 = ECO вкл
        if (rawECOselected == 5)
        { 
          km_per_kWh = (EEPROM.read(EEPROMaddr1)) / 10.0F;
          km_per_kWh = km_per_kWh + 1;
        }
        else if (rawECOselected == 4)
        {  
           km_per_kWh = (EEPROM.read(EEPROMaddr1)) / 10.0F;
        }
      }
      if (rxId == 0x54a) 
      {                                         // Читаем данные о настройках температуры климат контроля (100ms)
        rawCCSetpoint = (rxBuf[4]);             // 18.0C = 36, 18.5C = 37 ... 31.5.0C = 63, 32.0C = 64
      }  
    } 
    
   Setup_mode_check();
   
   if (Page >=1 && Page <= 5)
    { 
      byte data[8] = {0x02, 0x21, 0x04, 0xff, 0xff, 0xff, 0xff, 0xff};
      byte data2[8] = {0x30, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff};
      byte data3[8] = {0x02, 0x21, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff};
                                     
     if ((millis() - CANMillis) > CANInterval)
      {                                         //Проверяем, выдержан ли интервал повторения отправки сообщения и прочитаны ли данные по предыдущему запросу, делаем запрос и читаем данные о температуре батареи и вычисляем температурный коэффициэнт
        byte sndStat = CAN0.sendMsgBuf(0x79b, 0, 8, data);
        CANMillis = millis();
        CANSt = 0;
      }
      else                                 
      {
        if (rxId == 0x7bb && rxBuf[0] == 0x10)              
        {
          BattTemp1 = rxBuf[6];
          MinBattTemp = BattTemp1;
          byte sndStat = CAN0.sendMsgBuf(0x79b, 0, 8, data2);
        }
        else if (rxId == 0x7bb && rxBuf[0] == 0x21) 
        {
          BattTemp2 = rxBuf[2];
          BattTemp3 = rxBuf[5];
          if (MinBattTemp > BattTemp2) MinBattTemp = BattTemp2;
          if (MinBattTemp > BattTemp3) MinBattTemp = BattTemp3;
          byte sndStat = CAN0.sendMsgBuf(0x79b, 0, 8, data2);
        }
        else if (rxId == 0x7bb && rxBuf[0] == 0x22) 
        {
          BattTemp4 = rxBuf[1];
          CANSt = 1;                                 // Признак получения данных о температуре из CAN шины
          if (MinBattTemp > BattTemp4) MinBattTemp = BattTemp4;
          
          if (MinBattTemp > 25)      BattTempFactor1 = 1.02F;
          if (MinBattTemp <= 25)     BattTempFactor1 = 1.00F;
          if (MinBattTemp <= 10)     BattTempFactor1 = 0.95F;     
          if (MinBattTemp <= 0)      BattTempFactor1 = 0.85F;
          if (MinBattTemp < -10)     BattTempFactor1 = 0.75F; 
         }
       } 
    if (Page >=1 && Page <= 4)
    { 
     if (rxId == 0x1db)                             // Читаем данные о текущем напряжении и токе разряда/заряда батареи
     {                  
        BattVolts = (rxBuf[2] << 2) | (rxBuf[3] >> 6);
        BattVolts = BattVolts / 2.0F;
        Amp = (rxBuf[0] << 3) | (rxBuf[1] >> 5);
        if (Amp & 0x0400) Amp |= 0xf800; 
        Amps = -(Amp / (2.0F));
        kW = (Amps * BattVolts)/ 1000.0F;
        CPVaverage = BattVolts/0.096F;
        //CANSv = 1;                                   // Признак получения данных о напряжении и токе ВВБ из CAN шины
        // if ((prevBattVolts > BattVolts) && (BattVolts > 0) && (Amps > prevAmps) && (prevAmps >= 0)) intResistance = (prevBattVolts - BattVolts)/ (Amps - prevAmps); // Если напряжение снизилось и получены необходимые данные, рассчитываем внутреннее сопротивление
        // prevBattVolts = BattVolts;
        // prevAmps = Amps; 
     }
         

     if (Amps >= -1 && Amps <= 12)                                                    // Расчитываем % SOC, только при условии тока заряда/разряда от -1 до 12 Ампер
    { 
        //if (Amps > 2 && Amps <= 12) BattVolts = BattVolts + ((Amps - 2) * 0.03F);     // Корректировка просадки напряжения на 0,03 Вольт на каждый 1 Ампер в диапазоне тока от 2 до 12 Ампер
        WtPct = WT/100.0F;                                                            // Рассчитаем количество ватт в одном проценте SOC
        VsocPct5    = (V_SOC_5 - V_TURTLE)/5.0F;                                      // Рассчитаем долю в вольтах от общего напряжения в одном проценте SOC в каждом интервале от черепахи до 100%...
        VsocPct10   = (V_SOC_10 - V_SOC_5)/5.0F;
        VsocPct25   = (V_SOC_25 - V_SOC_10)/15.0F;
        VsocPct50   = (V_SOC_50 - V_SOC_25)/25.0F;
        VsocPct75   = (V_SOC_75 - V_SOC_50)/25.0F;
        VsocPct90   = (V_SOC_90 - V_SOC_75)/15.0F;
        VsocPct100  = (V_SOC_100 - V_SOC_90)/10.0F;
        
         if (BattVolts > V_SOC_90) 
         {
           ActSocPct = ((BattVolts - V_SOC_90) / VsocPct100 + 90) / 1.0F;
         }
         else if (BattVolts > V_SOC_75) 
         {
           ActSocPct = ((BattVolts - V_SOC_75) / VsocPct90 + 75) / 1.0F;
         }
         else if (BattVolts > V_SOC_50) 
         {
           ActSocPct = ((BattVolts - V_SOC_50) / VsocPct75 + 50) / 1.0F;
         }
         else if (BattVolts > V_SOC_25) 
         {
           ActSocPct = ((BattVolts - V_SOC_25) / VsocPct50 + 25) / 1.0F;
         }
         else if (BattVolts > V_SOC_10) 
         {
           ActSocPct = ((BattVolts - V_SOC_10) / VsocPct25 + 10) / 1.0F;
         }
         else if (BattVolts > V_SOC_5) 
         {
           ActSocPct = ((BattVolts - V_SOC_5) / VsocPct10 + 5) / 1.0F;
         } 
         else if (BattVolts > V_TURTLE) 
         {
           ActSocPct = ((BattVolts - V_TURTLE) / VsocPct5) / 1.0F;
         }
         else 
         {
           ActSocPct = 0;
         }
         
         unsigned long currentSocMillis = millis() - SocMillis;
         if (currentSocMillis > intSocPct && prevActSocPct != ActSocPct && prevActSocPct != 0)   //Проверяем, выдержан ли интервал изменения процента SOC в милисикундах и изменился ли % SOC
         { 
            float maxChangePct = currentSocMillis / intSocPct;
            if (maxChangePct >= +(prevActSocPct - ActSocPct))                                    //Проверяем, соблюдается ли условие изменения SOC максимум на 1% за 10 секунд
            {                                       
              SocMillis = millis();
              prevActSocPct = ActSocPct;
            }
            else 
            {
              ActSocPct = prevActSocPct;
            }
         }
         else if (prevActSocPct == 0)
         {
            prevActSocPct = ActSocPct;
         }  
         else 
         {
            ActSocPct = prevActSocPct;
         } 
         ActSockWh = ActSocPct * WtPct * BattTempFactor1 / 1000.00F; 
         range = km_per_kWh * ActSockWh;
       }      
      }
     }
    }
  
  // Если были прочитаны данные, была нажата кнопка климат контроля или изменено положение селектора, то выводятся данные на экран
 // if (CANSt == 1  || (rawCCButtonPress2 != rawCCButtonPress)  || (rawGearPos2 != rawGearPos)) 
 // {
  if ((millis() - LCDMillis) > LCDInterval)
  {                                                                             //Проверяем, выдержан ли интервал обновления информации на дисплее
    LCDMillis = millis();  
    switch (Page) 
    { 
      case 0:                               
        EEPROM.update(EEPROMaddr0, 1);                                            // Перезагрузка Arduino
        resetFunc();
      case 1:  
        u8g2.firstPage();
        do 
        {
          u8g2.drawXBMP( 0, 0, bitmap_width, bitmap_height, battery_large_bits);  // Прорисовка батареи
          u8g2.setFont(u8g2_font_logisoso24_tr);                                  // Установка шрифта
          u8g2.setCursor(41, 33);
          dtostrf(ActSocPct, 3, 0, buffer);
          u8g2.print((ActSocPct <= 0) ? "---" : buffer);
          u8g2.setFont(u8g2_font_10x20_t_cyrillic);                               // Установка шрифта с кириллицей
          u8g2.setCursor(36, LINE3);
          dtostrf(ActSockWh, 3, 1, buffer);
          u8g2.print((ActSockWh <= 0) ? "---" : buffer);
          u8g2.print(" кВтч");
        } while ( u8g2.nextPage() );
      case 2:
        u8g2.firstPage();
        do 
        {
          u8g2.drawXBM( 0, 40, 56, 24, battery_solid_bits);                          // Прорисовка батареи
          u8g2.setCursor(79, 27);  
          dtostrf(range, 3, 0, buffer);
          u8g2.print((range <= 0) ? "---" : buffer);
          u8g2.setCursor(0, 27);   
          u8g2.print("ОСТАТОК:");
          u8g2.setCursor(110, 27); 
          u8g2.print("км");
          u8g2.setCursor(5, LINE3);
          u8g2.setFontMode(1);                                                       // Чёрный тескт
          u8g2.setDrawColor(2);                                                      // Белый фон
          dtostrf(ActSocPct, 3, 0, buffer);
          u8g2.print((ActSocPct <= 0) ? "---" : buffer);
          u8g2.print("%"); 
          u8g2.setFontMode(0);
          u8g2.setCursor(70, LINE3);
          if (ActSockWh >= 10) u8g2.setCursor(60, LINE3);
          dtostrf(ActSockWh, 3, 1, buffer);
          u8g2.print((ActSockWh <= 0) ? "---" : buffer);
          u8g2.print("кВтч");    
        } while ( u8g2.nextPage() );
      case 3:
        u8g2.firstPage();
        do 
        {
          u8g2.setCursor(0, LINE1);
          u8g2.print("При:");  
          u8g2.print(km_per_kWh, 1);
          u8g2.print("км/кВч"); 
          u8g2.setFontMode(1);                          
          u8g2.setDrawColor(2);                          
          u8g2.drawBox(79, 20, 128, 21);
          u8g2.setCursor(0, LINE2); 
          u8g2.print("Остаток:");
          dtostrf(range, 3, 0, buffer);
          u8g2.print((range <= 0) ? "---" : buffer);
          u8g2.print("км");
          u8g2.setCursor(0, LINE3);
          u8g2.print("ECO режим:"); 
          u8g2.print((rawECOselected == 5) ? "ON" : "OFF");
        } while ( u8g2.nextPage() );
      case 4:
        u8g2.firstPage();
        do 
        {
          u8g2.drawHLine(0,  0, 128);
          u8g2.drawHLine(0, 21, 128);
          u8g2.drawHLine(0, 42, 128);
          u8g2.drawHLine(0, 63, 128);
          u8g2.drawVLine(0,   0, 63);
          u8g2.drawVLine(63,  0, 63);
          u8g2.drawVLine(127, 0, 63);
          u8g2.setCursor(2, 19);
          dtostrf(BattVolts, 3, 1, buffer); 
          u8g2.print((BattVolts <= 0) ? "---" : buffer); 
          u8g2.print("В"); 
          u8g2.setCursor(67, 19); 
          dtostrf(Amps, 3, 1, buffer);
          u8g2.print(Amps); 
          u8g2.print("А");
          u8g2.setCursor(2, 40);
          dtostrf(CPVaverage, 3, 0, buffer); 
          u8g2.print((CPVaverage <= 0) ? "---" : buffer);  
          u8g2.print("мВ"); 
          u8g2.setCursor(67, 40);
          dtostrf(kW, 3, 1, buffer); 
          u8g2.print((kW <= 0) ? "---" : buffer); 
          u8g2.print("кВ");
          u8g2.setCursor(2, 61);
          dtostrf(ActSockWh, 3, 1, buffer); 
          u8g2.print((ActSockWh <= 0) ? "---" : buffer); 
          u8g2.print("кВ"); 
          u8g2.setCursor(77, 61); 
          dtostrf(MinBattTemp, 3, 0, buffer);
          u8g2.print((MinBattTemp <= 0) ? "---" : buffer); 
          u8g2.print("°С");
       } while ( u8g2.nextPage() );
     case 5:
      u8g2.firstPage();
      do 
      {
        u8g2.drawHLine(0,  21, 128);
        u8g2.drawHLine(0, 41, 128);
        u8g2.drawHLine(0, 63, 128);
        u8g2.drawVLine(0,   21, 63);
        u8g2.drawVLine(63,  21, 63);
        u8g2.drawVLine(127, 21, 63);
        u8g2.setCursor(0, 20); 
        u8g2.print("Темп-ура ввб:");
        u8g2.setCursor(12, 39);
        dtostrf(BattTemp1, 3, 0, buffer);  
        u8g2.print(buffer);
        u8g2.print("°С"); 
        u8g2.setCursor(80, 39);
        dtostrf(BattTemp2, 3, 0, buffer); 
        u8g2.print(BattTemp2);
        u8g2.print("°С");
        u8g2.setCursor(12, 61); 
        dtostrf(BattTemp3, 3, 0, buffer); 
        u8g2.print((BattTemp3 <= 0) ? "---" : buffer);
        u8g2.print("°С"); 
        u8g2.setCursor(70, 61);
        dtostrf(BattTemp4, 3, 0, buffer); 
        u8g2.print((BattTemp4 <= 0) ? "---" : buffer);
        u8g2.print("°С"); 
      } while ( u8g2.nextPage() );
    case 6:
      u8g2.firstPage();
      do 
      {
        u8g2.setFontMode(0);
        u8g2.setCursor(0, LINE1); 
        u8g2.print("Настр.расхода");
        if (rawCCStatus1 == 0 && rawCCVentTarget == 152 && rawCCVentIntake == 9 && (rawGearPos == 2 ||rawGearPos == 3)) 
        {   
          switch (rawGearPos)  
          {
            case 2: onHeater = 1;                     // Если включена задняя скорость, настраивается зиминий расход с учетом потребления отопителя и можно редактировать расход в диапазоне 3.6 - 6.4 км/кВтч, без учета корректировки на ECO режим
            case 3: onHeater = 0;                     // Если включена нейтраль,  можно редактировать расход в диапазоне 6.5 - 9.3 км/кВтч, без учета корректировки на ECO режим
          }
          int kHeater;
          kHeater = ((onHeater == 1)? -6 : 23);
          km_per_dWh = rawCCSetpoint + kHeater;
          EEPROM.update(EEPROMaddr1, km_per_dWh);        
          km_per_kWh = (EEPROM.read(EEPROMaddr1)) / 10.0F;
          u8g2.setFontMode(1);                          
          u8g2.setDrawColor(2);                          
          u8g2.drawBox(0, 20, 127, 21);
          u8g2.setCursor(0, LINE2); 
          u8g2.print("км/кВтч: ");  
          u8g2.print(km_per_kWh, 1);
        } 
        else 
        {
          u8g2.setFontMode(0);                         
          u8g2.setCursor(0, LINE2); 
          u8g2.print("км/кВтч: ");  
          u8g2.print(km_per_kWh, 1);
        }
      } while ( u8g2.nextPage() );
     default:
        u8g2.setCursor(0, LINE1);
        u8g2.print(" № Дисплея");  
        u8g2.setCursor(0, LINE2); 
        u8g2.print("не определён");
        u8g2.setCursor(0, LINE3);
        u8g2.print("вкл. обдув +/-"); 
    }
  }  
} // END MAIN LOOP


void Setup_mode_check() 
{
  // Использование климат контроля для смены старниц дисплея
  // Включите климат и выключите кондиционер, включите рециркуляцию и направление в ноги, затем меняйте страницы с помощью регулировки скорости вентилятора
  // На седьмой странице Arduino будет постоянно перезагружатся, пока не смените страницу
  if (rawCCStatus1 == 0 && rawCCVentTarget == 152 && rawCCVentIntake == 9 && rawCCFanSpeed > 0 && rawCCFanSpeed < 7) 
  {
    Page = rawCCFanSpeed;
    EEPROM.update(EEPROMaddr0, rawCCFanSpeed);
    Draw_dotted_box();
    u8g2.setCursor(3, LINE4); 
    //u8g2.print(Page);
  }
  else if (rawCCStatus1 == 0 && rawCCVentTarget == 152 && rawCCVentIntake == 9 && rawCCFanSpeed == 7) 
  {
    Page = 0;
    EEPROM.update(EEPROMaddr0, 0);
  }
  else 
  {
    Page = EEPROM.read(EEPROMaddr0);
  }
}

void Draw_dotted_box() 
{
  for (byte i = 0; i < 63; i += 5) 
  {
    u8g2.drawPixel(0, i);
    u8g2.drawPixel(127, i);
  }
  for (byte i = 0; i < 127; i += 5) 
  {
    u8g2.drawPixel(i, 0);
    u8g2.drawPixel(i, 63);
  }
}

void Clear_EEPROM() 
{
  if (EEPROM.read(5) == 255) 
  {
    for (i = 6; i <= 94; i++) 
    {
      EEPROM.update(i, 0);
    }
  }
}
