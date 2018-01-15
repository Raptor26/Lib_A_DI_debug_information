/**
 * File:   %<%NAME%>%.%<%EXTENSION%>%
 * Author: %<%USER%>%
 *
 * Created on %<%DATE%>%, %<%TIME%>%
 */

#ifndef LIB_A_DI_DEBUG_INFORMATION_H
#define	LIB_A_DI_DEBUG_INFORMATION_H

/******************************************************************************/
//  Секция include (подключаем заголовочные файлы используемых модулей)
/*============================================================================*/
//  Стандартные библиотеки языка С
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
/*============================================================================*/


/*============================================================================*/
//  Библиотеки для работы с периферией микроконтроллера
/*============================================================================*/


/*============================================================================*/
//  Внешние модули
//          "Lib_A_CRC_cyclic_redundancy_check/Lib_A_CRC_cyclic_redundancy_check.h"
#include    "Lib_A_CRC_cyclic_redundancy_check/Lib_A_CRC_cyclic_redundancy_check.h"
/*============================================================================*/
/******************************************************************************/


/******************************************************************************/
//  Секция определения констант
/******************************************************************************/


/******************************************************************************/
//  Секция определения типов
//------------------------------------------------------------------------------
typedef struct {
    uint8_t beginMessageId;
    uint16_t messageType;
    float gyrArr[3];
    float accArr[3];
    float magArr[3];
    uint16_t crcMessage;
    char terminator;
} __attribute__((__packed__)) DI_gyr_acc_mag_package_s;
#define DI_GYR_ACC_MAG_PACKAGE_S_LENGHT                     sizeof(DI_gyr_acc_mag_package_s)
#define DI_GYR_ACC_MAG_PACKAGE_S_CRC_BYTES_NUMB             2
#define DI_GYR_ACC_MAG_PACKAGE_S_BYTES_NUMB_AFTER_CRC       1
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
typedef struct {
    uint8_t beginMessageId;
    uint8_t numbMessageBytes;
    float gyrArr[3];
    float accArr[3];
    float magArr[3];
    float quatArr[4];
    float eulerAnglArr[3];
    uint8_t crc;
} __attribute__((__packed__)) DI_package_for_serial_plot_s;

#define DI_PACKAGE_FOR_SERIAL_PLOT_S_LENGHT                 sizeof(DI_package_for_serial_plot_s)
#define DI_PACKAGE_FOR_SERIAL_PLOT_S_CRC_BYTES_NUMB         1
#define DI_PACKAGE_FOR_SERIAL_PLOT_S_BYTES_NUMB_AFTER_CRC   0
//------------------------------------------------------------------------------
/******************************************************************************/


/******************************************************************************/
//  Секция определения глобальных переменных
/******************************************************************************/


/******************************************************************************/
//  Секция прототипов глобальных функций
extern uint8_t DI_CopyGyrAccMagDataInStruct(DI_gyr_acc_mag_package_s *pPackageStruct,
        float *pGyrArr, float *pAccArr, float *pMagArr);

extern uint8_t DI_CopyDataInStructForSerialPlot(DI_package_for_serial_plot_s *pPackageStruct,
        float *pGyrArr, float *pAccArr, float *pMagArr,
        float *pQuatArr,
        float *pEulerAnglArr);
/******************************************************************************/


/******************************************************************************/
//  Секция определения макросов
/******************************************************************************/

#endif	/* LIB_A_DI_DEBUG_INFORMATION_H */

////////////////////////////////////////////////////////////////////////////////
//  END OF FILE
////////////////////////////////////////////////////////////////////////////////