/**
 * @file   	Lib_A_DI_debug_information.h
 * @author 	Isaev Mickle
 * @version	beta
 * @date 	10.04.2018
 * @brief	Библиотека содержит функции для формирования пакета данных, который
 * 			будет переведен программной SerialPlot в график
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
#include <string.h>
#include <stdarg.h>
/*============================================================================*/


/*============================================================================*/
//  Библиотеки для работы с периферией микроконтроллера
/*============================================================================*/


/*============================================================================*/
//  Внешние модули
#include "../Lib_A_CRC_cyclic_redundancy_check/Lib_A_CRC_cyclic_redundancy_check.h"
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
    /**
     * Условие старта пакета данных "Start frame"
     */
    uint8_t beginMessageId;

    /**
     * Количество полезных байтов данных;
     */
    uint8_t numbMessageBytes;

    /**
     * Массив угловых скоростей по 3-м осям;
     */
    float gyrArr[3];

    /**
     * Массив линейных ускрений по 3-м осям;
     */
    float accArr[3];

    /**
     * Массив магнитного поля по 3-м осям;
     */
    float magArr[3];

    /**
     * Массив компонент кватерниона;
     */
    float quatArr[4];

    /**
     *  Массив углов Эйлера;
     */
    float eulerAnglArr[3];

    /**
     * Коэффициент пропорциональной коррекции;
     */
    float kProp;

    /**
     * Норма акселерометров
     */
    float accNorm;

    /**
     * Массив дрейфа гироскопов по 3-м осям;
     */
    float gyrBiasArr[3];

    /**
     * Контрольная сумма пакета данных без учета следующих полей структуры:
     * - "beginMessageId"
     * - "numbMessageBytes"
     */
    uint8_t crc;
} __attribute__((__packed__)) DI_inert_sens_package_for_serial_plot_s;

#define DI_INERT_SENS_PACKAGE_FOR_SERIAL_PLOT_S_LENGHT                 sizeof(DI_inert_sens_package_for_serial_plot_s)
#define DI_INERT_SENS_PACKAGE_FOR_SERIAL_PLOT_S_CRC_BYTES_NUMB         1
#define DI_INERT_SENS_PACKAGE_FOR_SERIAL_PLOT_S_BYTES_NUMB_AFTER_CRC   0
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------

typedef struct {
    uint8_t beginMessageId;
    uint8_t numbMessageBytes;

    float currentAbsoluteAngel;
    float currentAngelInElectAngel;
    float needAngelInElectAngel;
    float needAngelElectAndCurrentAngelDiff;
    float amplitudeCurrent;
    float needAbsoluteAngelAndCurrentAngelDiff;
    float angularSpeed;
    float angularSpeed2;
    uint8_t crc;
} __attribute__((__packed__)) DI_vect_motor_control_package_for_serial_plot_s;

#define DI_VECT_MOTOR_CONTROL_PACKAGE_FOR_SERIAL_PLOT_S_LENGHT                  sizeof (DI_vect_motor_control_package_for_serial_plot_s)
#define DI_VECT_MOTOR_CONTROL_PACKAGE_FOR_SERIAL_PLOT_S_CRC_BYTES_NUMB          1
#define DI_VECT_MOTOR_CONTROL_PACKAGE_FOR_SERIAL_PLOT_S_BYTES_NUMB_AFTER_CRC    0
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

typedef struct {
    uint8_t beginMessageId;
    uint8_t numbMessageBytes;

    float dataWithOutFilt;
    float iir_lowPass10Hz;
    float iir_lowPass50Hz;
    float iir_lowPass100Hz;
    float iir_lowPass150Hz;
    float iir_lowPass200Hz;
    float iir_lowPass250Hz;
    float iir_lowPass300Hz;
    float iir_lowPass350Hz;
    float iir_lowPass400Hz;

    uint8_t crc;
} __attribute__((__packed__)) DI_win_filter_comp_for_serial_plot_s;

#define DI_WIN_FILTER_COMP_FOR_SERIAL_PLOT_S_LENGHT				sizeof (DI_win_filter_comp_for_serial_plot_s)
#define DI_WIN_FILTER_COMP_FOR_SERIAL_PLOT_S_CRC_BYTES_NUMB		1
#define DI_WIN_FILTER_COMP_FOR_SERIAL_PLOT_S_BYTES_AFTER_CRC	0
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

typedef struct {
    uint8_t beginMessageId;
    uint8_t numbMessageBytes;

    float dataArr[40];

    uint8_t crc;
} __attribute__((__packed__)) DI_data_for_serial_plot_s;
//------------------------------------------------------------------------------
/******************************************************************************/


/******************************************************************************/
//  Секция определения глобальных переменных
/******************************************************************************/


/******************************************************************************/
//  Секция прототипов глобальных функций
//  Для отладки <системы ориентации>;
extern uint8_t DI_CopyGyrAccMagDataInStruct(
        DI_gyr_acc_mag_package_s *pPackageStruct,
        float *pGyrArr,
        float *pAccArr,
        float *pMagArr);

extern uint8_t DI_CopyInertSensDataInStructForSerialPlot(
        DI_inert_sens_package_for_serial_plot_s *pPackageStruct,
        float *pGyrArr,
        float *pAccArr,
        float *pMagArr,
        float *pQuatArr,
        float *pEulerAngleArr,
        float *kProp,
        float *accNorm,
        float *gyrBiasArr);

//  Для отладки <векторного управления> 3-х фазным электродвигателем;
extern uint8_t DI_CopyVectMotorControlDataInStructForSerialPlot(
        DI_vect_motor_control_package_for_serial_plot_s *pPackageStruct,
        float currentAbsoluteAngel,
        float currentAngelInElectAngel,
        float needAngelInElectAngel,
        float needAngelAndCurrentAngelDiff,
        float amplitudeCurrent,
        float needAbsoluteAngelAndCurrentAngelDiff,
        float angularSpeed,
        float angularSpeed2);

extern uint8_t DI_CopyWinFiltDataInStructForSerialPlot(
        DI_win_filter_comp_for_serial_plot_s *pPackageStruct,
        float dataWithOutFilt,
        float iir_lowPass10Hz,
        float iir_lowPass50Hz,
        float iir_lowPass100Hz,
        float iir_lowPass150Hz,
        float iir_lowPass200Hz,
        float iir_lowPass250Hz,
        float iir_lowPass300Hz,
        float iir_lowPass350Hz,
        float iir_lowPass400Hz);

extern size_t DI_CopyDataForSerialPlot_f32(DI_data_for_serial_plot_s *pStruct,
        float data,
        ...);
/******************************************************************************/


/******************************************************************************/
//  Секция определения макросов
/******************************************************************************/

#endif	/* LIB_A_DI_DEBUG_INFORMATION_H */

////////////////////////////////////////////////////////////////////////////////
//  END OF FILE
////////////////////////////////////////////////////////////////////////////////