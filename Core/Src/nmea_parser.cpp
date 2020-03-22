/*
 * NMEA_PARSER.cpp
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */

#include "nmea_parser.h"
#include "string.h"
namespace NMEA_UART {

Parser::Parser() :
    //RMC , hhmmss.sss,      x,  llll.lll,  a,  yyyyy.yyy,  a,       x.x,               u.u,  xxxxxx,       ,       ,             v*hh
    RMC      { UTCtime, Status, SLatitude, NS, SLongitude, EW, SpeedKnot, CourseOverGround , UTCdate, UNUSED, UNUSED,ModeIndicator , UNUSED},

    //VTG,       x.x,      T,            y.y,      M,       u.u,       N,     v.v,      K,             m*hh
    VTG { CourseTrue, UNUSED, CourseMagnetic, UNUSED, SpeedKnot,  UNUSED,SpeedKmh, UNUSED, ModeIndicator, UNUSED},

    //GGA, hhmmss.ss,   llll.lll,  a, yyyyy.yyy,  a,                x,       uu,    v.v,         w.w,      M,               x.x,      M,       ,   zzzz*hh
    GGA    { UTCtime,  SLatitude, NS,SLongitude, EW, QualityIndicator, SatCount,   HDOP, AltitudeMSL, UNUSED, GeoidalSeparation, UNUSED, UNUSED, UNUSED, UNUSED},//+2

    //GSA,      a,      x,     xx,     xx,     xx,    xx,     xx,     xx,     xx,     xx,     xx,     xx,     xx,     xx,    u.u,    v.v,   z.z*hh
    GSA { GSAMode,FixType, UNUSED, UNUSED, UNUSED,UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED,   PDOP,   HDOP,  VDOP, UNUSED},

    //GSV,     x,      u,            xx,     uu,     vv,    zzz,     ss,     uu,     vv,    zzz,     ss,     uu,     vv,    zzz,     ss,     uu,     vv,    zzz,     ss*hh
    GSV { UNUSED, UNUSED, ViewSatellite, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED}

{
    GLONAS_COUNT = 0;
    GPS_COUNT = 0;

}

Parser::~Parser() {
}

gpsMessege Parser::getMessege() {

    gpsMessege gpsPack;
    gpsPack.Status = Status[0];
    gpsPack.Time = asciiRemoveDotToInt(UTCtime) * 10;
    float tempSpeed = stringToFloat(SpeedKnot) * 1.852;
    gpsPack.Speed = tempSpeed;
    gpsPack.SLatitude = convertStrDegToDecimal(SLatitude);
    gpsPack.SLongitude = convertStrDegToDecimal(SLongitude);
    gpsPack.CourseTrue = stringToFloat(CourseOverGround );
    return (gpsPack);
}

/*//Парсер посылки UART 1 байт*/
GPS_MESSEGE_TYPE Parser::charParser(unsigned char data) {
    static unsigned char ByteCount = 0xff;
    static unsigned int MsgType;
    static char *MsgTxt = (char*) &MsgType;
    static unsigned char ComaPoint = 0xff;
    static unsigned char CharPoint = 0;
    if (data == '$') {
        ByteCount = 0;
        ComaPoint = 0xff;
        MsgTxt = (char*) &MsgType;
        return (GPS_NULL);
    } /*//ждем начала стрки*/
    if (ByteCount == 0xff)
        return (GPS_NULL);
    ByteCount++;
    if (ByteCount <= 1)
        return (GPS_NULL);
    if (ByteCount < 6 && ByteCount > 1) /*//берем 4 символа заголовка*/
    {
        *MsgTxt = data; /*//и делаем из него число*/
        MsgTxt++;
        return (GPS_NULL);
    }

    switch (MsgType) {

    case 0x434D5250: /*//GPRMC*/
    case 0x434D524E: /*//GNRMC*/
        if (data == ',') {
            ComaPoint++;
            CharPoint = 0;
            RMC[ComaPoint][0] = 0;
            return (GPS_NULL);
        }
        if ((data) == ('*')) {
            MsgType = 0;

            return (GPS_NRMC);
        }
        RMC[ComaPoint][CharPoint++] = data;
        RMC[ComaPoint][CharPoint] = 0;
        return (GPS_NULL);

    case 0x47545650: /*//PVTG*/
        if (data == ',') {
            ComaPoint++;
            CharPoint = 0;
            VTG[ComaPoint][0] = 0;
            return (GPS_NULL);
        }
        if (data == '*') {
            return (GPS_PVTG);
        }
        VTG[ComaPoint][CharPoint++] = data;
        VTG[ComaPoint][CharPoint] = 0;
        return (GPS_NULL);

    case 0x4754564e: /*//NVTG*/
        if (data == ',') {
            ComaPoint++;
            CharPoint = 0;
            VTG[ComaPoint][0] = 0;
            return (GPS_NULL);
        }
        if (data == '*') {
            return (GPS_NVTG);
        }
        VTG[ComaPoint][CharPoint++] = data;
        VTG[ComaPoint][CharPoint] = 0;
        return (GPS_NULL);

    case 0x41474750: /*//PGGA*/
    case 0x4147474e: /*//NGGA*/
        if (data == ',') {
            ComaPoint++;
            CharPoint = 0;
            GGA[ComaPoint][0] = 0;
            return (GPS_NULL);
        }
        if (data == '*') {
            MsgType = 0;
            return (GPS_NGGA);
        }
        GGA[ComaPoint][CharPoint++] = data;
        GGA[ComaPoint][CharPoint] = 0;
        return (GPS_NULL);
    case 0x56534750: /*//PGSV*/
        if (data == ',') {
            ComaPoint++;
            CharPoint = 0;
            GSV[ComaPoint][0] = 0;
            return (GPS_NULL);
        }
        if (data == '*') {
            GPS_COUNT = asciiToInt(ViewSatellite);
            MsgType = 0;
            return (GPS_PGSV);
        }
        GSV[ComaPoint][CharPoint++] = data;
        GSV[ComaPoint][CharPoint] = 0;
        return (GPS_NULL);
    case 0x5653474c: /*//LGSV*/
        if (data == ',') {
            ComaPoint++;
            CharPoint = 0;
            GSV[ComaPoint][0] = 0;
            return (GPS_NULL);
        }
        if (data == '*') {
            GLONAS_COUNT = asciiToInt(ViewSatellite);
            MsgType = 0;
            return (GPS_LGSV);
        }
        GSV[ComaPoint][CharPoint++] = data;
        GSV[ComaPoint][CharPoint] = 0;
        return (GPS_NULL);

    case 0x4153474e: /*//NGSA 4153474e*/
    case 0x41534750: /*//NGSA 4153474e*/
        if (data == ',') {
            ComaPoint++;
            CharPoint = 0;
            GSA[ComaPoint][0] = 0;
            return (GPS_NULL);
        }
        if (data == '*') {
            return (GPS_GNGSA);
        }
        GSA[ComaPoint][CharPoint++] = data;
        GSA[ComaPoint][CharPoint] = 0;
        return (GPS_NULL);

    default:
        ByteCount = 0xff;
        break;
    }
    ByteCount = 0xff;
    return (GPS_NULL);
}

/***********************************************************************************/
/*             УТИЛИТЫ					*/
/************************************************************************************/
//Конвертация String в float, возвращает число , "151.6654684" -> 151.6654684*/
float stringToFloat(char *string) {
    float result = 0.0;
    int len = strlen(string);
    int dotPosition = 0;
    for (int i = 0; i < len; i++) {
        if (string[i] == '.') {
            dotPosition = len - i - 1;
        } else {
            result = result * 10.0 + (string[i] - '0');
        }
    }
    while (dotPosition--) {
        result /= 10.0;
    }
    return (result);
}

/*//Возвращает разницу времени в секундах*/
uint32_t getDifTime(uint32_t startTime, uint32_t stopTime) {
    uint64_t result = 0;
    /*//150012300*/
    int64_t difHour = (stopTime / 10000000) - (startTime / 10000000);
    int64_t difMinute = (stopTime / 100000) % 100 + difHour * 60
            - (startTime / 100000) % 100;
    int64_t difSecons = (stopTime % 100000) + difMinute * 60000
            - (startTime % 100000);
    result = difSecons;
    return (result);
}

/*//TODO Дописать функцию преобразования координат в десятичный вид*/
float convertStrDegToDecimal(char *coor) {
    /*// координаты приходят в виде "09224.91216" где 92 градуса 24.91216 минуты, секунд в посылке нет!!!*/
    float resultDecCoor, ss;
    uint16_t dd, mm;
    float fCoor = stringToFloat(coor) / 100;
    dd = fCoor;
    mm = ((fCoor - dd) * 100);
    ss = ((fCoor - dd) * 100) - mm;
    resultDecCoor = dd + ((float) mm / 60) + ((float) ss / 60);
    return (resultDecCoor);
}

/*//Конвертация String в Int, строка без точки, "154" -> 154*/
int asciiToInt(char* s) {
    int n = 0;
    while (*s >= '0' && *s <= '9') {
        n *= 10;
        n += *s++;
        n -= '0';
    }
    return (n);
}

/*//Конвертация String в Int, возвращает целую часть до точки "151.6654684" -> 154*/
uint32_t asciiBeforeDotToInt(char* s) {
    uint32_t n = 0;
    while (*s >= '0' && *s <= '9') {
        if (*s == '.')
            return (n);
        n *= 10;
        n += *s++;
        n -= '0';
    }
    return (n);
}

/*//Конвертация String в Int, возвращает дробную часть после точки, "151.6654684" -> 6654684*/
uint32_t asciiAfterDotToInt(char* s) {
    uint32_t n = 0;
    uint8_t flagDot = 0;
    while (*s) {
        if (*s == '.') {
            flagDot = 1;
            s++;
        }
        if (flagDot && (*s)) {
            n *= 10;
            n += *s++;
            n -= '0';
        } else {
            s++;
        }
    }
    return (n);
}

/*//Конвертация String в Int, возвращает число без точки, "151.6654684" -> 1516654684*/
uint32_t asciiRemoveDotToInt(char* s)

{
    uint32_t n = 0;
    while (*s) {
        if (*s == '.') {
            s++;
        }
        if (*s) {
            n *= 10;
            n += *s++;
            n -= '0';
        } else {
            s++;
        }
    }
    return (n);
}
/*//Конвертация Int в String, возвращает число без точки */


}
