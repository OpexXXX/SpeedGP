/*
 * GpsHadler.h
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */

#ifndef NMEAPARSER_H_
#define NMEAPARSER_H_
#include <stdint.h>

namespace NMEA_UART {

typedef enum {
    GPS_NULL = 0,
    GPS_PRMC = 1,
    GPS_NRMC,
    GPS_PGGA,
    GPS_NGGA,
    GPS_PGSV,
    GPS_LGSV,
    GPS_PVTG,
    GPS_NVTG,
    GPS_GPGSA,
    GPS_GNGSA,
} GPS_MESSEGE_TYPE;

typedef struct {
    uint8_t Status;
    uint32_t Time; //время
    float SLatitude;  //Широта
    float SLongitude;         //Долгота
    float altitude;	//Высота
    uint32_t CourseTrue;                // курс
    uint32_t Speed; 	//скорость
} gpsMessege;

//Конвертация String в Int, строка без точки, "154" -> 154
int asciiToInt(char* s);

//Конвертация String в Int, возвращает целую часть до точки "151.6654684" -> 154
uint32_t asciiBeforeDotToInt(char* s);

//Конвертация String в Int, возвращает дробную часть после точки, "151.6654684" -> 6654684
uint32_t asciiAfterDotToInt(char* s);

//Конвертация String в Int, возвращает число без точки, "151.6654684" -> 1516654684
uint32_t asciiRemoveDotToInt(char* s);

//Конвертация String в float, возвращает число , "151.6654684" -> 151.6654684
float stringToFloat(char *string);

//Возвращает разницу времени в секундах
uint32_t getDifTime(uint32_t startTime, uint32_t stopTime);

//TODO Дописать функцию преобразования коо
float convertStrDegToDecimal(char *coor);

class Parser {
public:
    Parser();
    gpsMessege getMessege();               //Подготовить посылку
    GPS_MESSEGE_TYPE charParser(unsigned char);		//Парсер пакета UART
    virtual
    ~Parser();
//private:
    //RMC  Recommended Minimum Specific GNSS Data
    char UTCtime[10];           //время в формате ччммсс.сс по UTC;время в формате ччммсс.сс по UTC
    char Status[2];             //статус, А если данные достоверны или V если не достоверны
    char SLatitude[12];         //широта в формате ddmm.mmmm
    char NS[2];                 //полушарие, N для северного, S для южного
    char SLongitude[12];        //долгота в формате ddmm.mmmm
    char EW[2];                 //полушарие, W  для западного, E для восточного
    char SpeedKnot[9];          //скорость относительно земли в узлах (1 узел = 1.852 км/ч)
    char CourseOverGround[9];  //азимут направления движения в градусах
    char UTCdate[8];           //дата в формате ddmmyy
    //Не используется           //магнитное склонение в градусах
    //Не используется           //направление склонения, W для западного, E для восточного
    char ModeIndicator[2];      //Индикатор режима

    //VTG Course Over Ground and Ground Speed
    char CourseTrue[9];         //Курс на истинный полюс (в градусах), затем следует буква Т
    char CourseMagnetic[9];     //Курс на магнитный полюс (так же в градусах), затем следует буква М
    //    SpeedKnot[8];         //Скорость относительно земли в узлах, затем следует буква N
    char SpeedKmh[9];           //Скорость относительно земли в км/ч, затем следует буква К
    //   ModeIndicator[3];      //Индикатор режима, согласно рассмотренным ранее значениям

    //GGA – Global Positioning System Fix Data
    //   char UTCtime[12]       //Время определения координат в формате ччммсс.сс по UTC
    //   Не используется -      //широта в формате ddmm.mmmm
    //   Не используется -      //полушарие, N для северного, S для южного
    //   Не используется -      //долгота в формате ddmm.mmmm
    //   Не используется -      //полушарие, W для западного, E для восточного
    char QualityIndicator[2];   //режим работы приемника (о значениях позже)
    char SatCount[3];           //количество спутников, использованных для получения координат
    //   Не используется -      //    HDOP;
    char AltitudeMSL[12];       //Высота над уровнем моря в метрах, далее следует буква М
    char GeoidalSeparation[12]; //Не используется -      //Высота над геоидом в метрах, далее следует буква М
    //   Не используется -      //Возраст дифференциальных поправок (в моем случае пусто)
    //   Не используется -      //Режимы работы приемника

    // GSA – GNSS DOP and Active Satellites
    char GSAMode [2];           //Режим переключения 2D/3D, А – автоматический, М – ручной
    char FixType[2];            //  Режим: 1 – нет решения, 2 – 2D, 3- 3D
    //   Не используется -      //ID номера спутников, используемых в нахождении координат (1-32 для GPS, 65-96 для ГЛОНАСС)
    char PDOP[8];               //PDOP (снижение точности по местоположению)
    char HDOP[8];               //HDOP (снижение точности в горизонтальной плоскости)
    char VDOP[8];               //VDOP (снижение точности в вертикальной плоскости)

    //GSV – GNSS Satellites in View содержат информацию о видимых спутниках, в каждом сообщении может содержаться информация максимум о 4 спутниках. Строки содержат данные:
    //   Не используется -      //Общее количество сообщений (в нашем случае 3);
    //   Не используется -      //Номер текущего сообщения (обратите внимание на каждую строку, эти значения идут по порядку);
    char ViewSatellite[4];            //Общее количество видимых спутников (во всех трех сообщениях это значение одинаково);
    //   Не используется -      //ID номер спутника;
    //   Не используется -      //Угол места в градусах (макс. 90);
    //   Не используется -      //Азимут в градусах (0-359);
    //   Не используется -      //SNR (00-99 дБГц)4
    //*****Последние 4 значения встречаются в строке 4 раза подряд, если строка содержит информацию о 4 спутниках. Если строка содержит информацию менее чем о 4 спутниках, то нулевые поля (,,,,) не используются.*/

    char UNUSED[32]; /*//мусорка, тут все данные, которые не нужны*/

    unsigned char GLONAS_COUNT;
    unsigned char GPS_COUNT;

    char * const RMC[13];
    char * const VTG[10];
    char * const GGA[15];
    char * const GSA[18];
    char * const GSV[20];
};

}

#endif /* NMEAPARSER_H_ */
