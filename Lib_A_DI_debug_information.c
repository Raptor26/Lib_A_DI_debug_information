/**
 * @file   	Lib_A_DI_debug_information.c
 * @author 	Isaev Mickle
 * @version	beta
 * @date 	10.04.2018
 * @brief	Библиотека содержит функции для формирования пакета данных, который
 * 			будет переведен программной SerialPlot в график
 */

/*#### |Begin| --> Секция - "Include" ########################################*/
#include "Lib_A_DI_debug_information.h"
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Глобальные переменные" ##########################*/
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/


/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
/*#### |End  | <-- Секция - "Локальные переменные" ###########################*/


/*#### |Begin| --> Секция - "Прототипы локальных функций" ####################*/
/*#### |End  | <-- Секция - "Прототипы локальных функций" ####################*/

/*#### |Begin| --> Секция - "Описание глобальных функций" ####################*/
uint8_t DI_CopyGyrAccMagDataInStruct(DI_gyr_acc_mag_package_s *pPackageStruct,
                                     float *pGyrArr, float *pAccArr, float *pMagArr)
{
    //  Условием начала пакета данных является код "0xAA";
    pPackageStruct->beginMessageId = 0xAA;

    //  Идентификатор данного сообщения равен "1";
    pPackageStruct->messageType = 1;

    //  Копируем в структуру данные гироскопа;
    pPackageStruct->gyrArr[0] = *pGyrArr++;
    pPackageStruct->gyrArr[1] = *pGyrArr++;
    pPackageStruct->gyrArr[2] = *pGyrArr;

    //  Копируем в структуру данные акселерометра;
    pPackageStruct->accArr[0] = *pAccArr++;
    pPackageStruct->accArr[1] = *pAccArr++;
    pPackageStruct->accArr[2] = *pAccArr;

    //  Копируем в структуру данные магнитометра;
    pPackageStruct->magArr[0] = *pMagArr++;
    pPackageStruct->magArr[1] = *pMagArr++;
    pPackageStruct->magArr[2] = *pMagArr;

    //  Считаем контрольную сумму данных;
    pPackageStruct->crcMessage = CRC_XOR_CCITT_Poly0x1021_Crc16((uint8_t*) pPackageStruct,
                                                                DI_GYR_ACC_MAG_PACKAGE_S_LENGHT
                                                                - DI_GYR_ACC_MAG_PACKAGE_S_CRC_BYTES_NUMB
                                                                - DI_GYR_ACC_MAG_PACKAGE_S_BYTES_NUMB_AFTER_CRC);
    pPackageStruct->terminator = '\n';
    return 0;
}

uint8_t DI_CopyInertSensDataInStructForSerialPlot(
                                                  DI_inert_sens_package_for_serial_plot_s *pPackageStruct,
                                                  float *pGyrArr, float *pAccArr, float *pMagArr,
                                                  float *pQuatArr,
                                                  float *pEulerAngleArr,
                                                  float *kProp,
                                                  float *accNorm,
                                                  float *gyrBiasArr)
{
    //  Start frame;
    pPackageStruct->beginMessageId = 0xAA;

    //  Вычисление длинны пакета данных без учета байта "beginMessageId",
    //  "numbMessageBytes" и "crc";
    pPackageStruct->numbMessageBytes = sizeof (DI_inert_sens_package_for_serial_plot_s)
            - sizeof (pPackageStruct->beginMessageId)
            - sizeof (pPackageStruct->numbMessageBytes)
            - sizeof (pPackageStruct->crc)
            - DI_INERT_SENS_PACKAGE_FOR_SERIAL_PLOT_S_BYTES_NUMB_AFTER_CRC;

    //  Копирование в структуру данных гироскопа;
    memcpy(pPackageStruct->gyrArr, pGyrArr, sizeof (float) * 3);

    //  Копирование в структуру данных акселерометра;
    memcpy(pPackageStruct->accArr, pAccArr, sizeof (float) * 3);

    //  Копирование в структуру данных магнитометра;
    memcpy(pPackageStruct->magArr, pMagArr, sizeof (float) * 3);

    //  Копирование в структуру данных кватерниона;
    memcpy(pPackageStruct->quatArr, pQuatArr, sizeof (float) * 4);

    //  Копирование в стурктуру углов Эйлера;
    memcpy(pPackageStruct->eulerAnglArr, pEulerAngleArr, sizeof (float) * 3);

    pPackageStruct->kProp = *kProp;
    pPackageStruct->accNorm = *accNorm;

    memcpy(pPackageStruct->gyrBiasArr, gyrBiasArr, sizeof (float) * 3);

    //  Расчет контрольной суммы;
    pPackageStruct->crc = CRC_XOR_Crc8((uint8_t*) & pPackageStruct->gyrArr[0],
                                       sizeof (DI_inert_sens_package_for_serial_plot_s)
                                       - sizeof (pPackageStruct->beginMessageId)
                                       - sizeof (pPackageStruct->numbMessageBytes)
                                       - sizeof (pPackageStruct->crc)
                                       - DI_INERT_SENS_PACKAGE_FOR_SERIAL_PLOT_S_BYTES_NUMB_AFTER_CRC);
    return 0;
}

uint8_t DI_CopyVectMotorControlDataInStructForSerialPlot(
                                                         DI_vect_motor_control_package_for_serial_plot_s *pPackageStruct,
                                                         float currentAbsoluteAngel,
                                                         float currentAngelInElectAngel,
                                                         float needAngelInElectAngel,
                                                         float needAngelAndCurrentAngelDiff,
                                                         float amplitudeCurrent,
                                                         float needAbsoluteAngelAndCurrentAngelDiff,
                                                         float angularSpeed,
                                                         float angularSpeed2)
{
    //  Start frame;
    pPackageStruct->beginMessageId = 0xAA;

    //  Вычисление длинны покате данных без учета байта "beginMessageId",
    //  "numbMessageBytes" и "crc";
    pPackageStruct->numbMessageBytes = sizeof (DI_vect_motor_control_package_for_serial_plot_s)
            - sizeof (pPackageStruct->beginMessageId)
            - sizeof (pPackageStruct->numbMessageBytes)
            - sizeof (pPackageStruct->crc)
            - DI_VECT_MOTOR_CONTROL_PACKAGE_FOR_SERIAL_PLOT_S_BYTES_NUMB_AFTER_CRC;

    pPackageStruct->currentAbsoluteAngel = currentAbsoluteAngel;
    pPackageStruct->needAngelElectAndCurrentAngelDiff = needAngelAndCurrentAngelDiff;
    pPackageStruct->currentAngelInElectAngel = currentAngelInElectAngel;
    pPackageStruct->needAngelInElectAngel = needAngelInElectAngel;
    pPackageStruct->amplitudeCurrent = amplitudeCurrent;
    pPackageStruct->needAbsoluteAngelAndCurrentAngelDiff = needAbsoluteAngelAndCurrentAngelDiff;
    pPackageStruct->angularSpeed = angularSpeed;
    pPackageStruct->angularSpeed2 = angularSpeed2;

    //  Расчет контрольной суммы;
    //	pPackageStruct->crc = CRC_XOR_Crc8((uint8_t*) &pPackageStruct->currentAbsoluteAngel,
    //	                                   sizeof(DI_vect_motor_control_package_for_serial_plot_s)
    //	                                   - sizeof(pPackageStruct->beginMessageId)
    //	                                   - sizeof(pPackageStruct->numbMessageBytes)
    //	                                   - sizeof(pPackageStruct->crc)
    //	                                   - DI_VECT_MOTOR_CONTROL_PACKAGE_FOR_SERIAL_PLOT_S_BYTES_NUMB_AFTER_CRC);

    return 1;
}

uint8_t DI_CopyWinFiltDataInStructForSerialPlot(DI_win_filter_comp_for_serial_plot_s *pPackageStruct,
                                                float dataWithOutFilt,
                                                float iir_lowPass10Hz,
                                                float iir_lowPass50Hz,
                                                float iir_lowPass100Hz,
                                                float iir_lowPass150Hz,
                                                float iir_lowPass200Hz,
                                                float iir_lowPass250Hz,
                                                float iir_lowPass300Hz,
                                                float iir_lowPass350Hz,
                                                float iir_lowPass400Hz)
{
    //  Start frame;
    pPackageStruct->beginMessageId = 0xAA;

    //  Вычисление длинны пакета данных без учета байта "beginMessageId",
    //  "numbMessageBytes" и "crc";
    pPackageStruct->numbMessageBytes = sizeof (DI_win_filter_comp_for_serial_plot_s)
            - sizeof (pPackageStruct->beginMessageId)
            - sizeof (pPackageStruct->numbMessageBytes)
            - sizeof (pPackageStruct->crc)
            - DI_WIN_FILTER_COMP_FOR_SERIAL_PLOT_S_BYTES_AFTER_CRC;

    pPackageStruct->dataWithOutFilt = dataWithOutFilt;
    pPackageStruct->iir_lowPass10Hz = iir_lowPass10Hz;
    pPackageStruct->iir_lowPass50Hz = iir_lowPass50Hz;
    pPackageStruct->iir_lowPass100Hz = iir_lowPass100Hz;
    pPackageStruct->iir_lowPass150Hz = iir_lowPass150Hz;
    pPackageStruct->iir_lowPass200Hz = iir_lowPass200Hz;
    pPackageStruct->iir_lowPass250Hz = iir_lowPass250Hz;
    pPackageStruct->iir_lowPass300Hz = iir_lowPass300Hz;
    pPackageStruct->iir_lowPass350Hz = iir_lowPass350Hz;
    pPackageStruct->iir_lowPass400Hz = iir_lowPass400Hz;

    //  Расчет контрольной суммы;
    pPackageStruct->crc = CRC_XOR_Crc8((uint8_t*) & pPackageStruct->iir_lowPass100Hz,
                                       sizeof (DI_win_filter_comp_for_serial_plot_s)
                                       - sizeof (pPackageStruct->beginMessageId)
                                       - sizeof (pPackageStruct->numbMessageBytes)
                                       - sizeof (pPackageStruct->crc)
                                       - DI_VECT_MOTOR_CONTROL_PACKAGE_FOR_SERIAL_PLOT_S_BYTES_NUMB_AFTER_CRC);
    return 0;
}

/**
 * @brief	Функция выполняет копирование переменных в структуру типа
 * 			"DI_data_for_serial_plot_s" для дальнейшей отправки в программу "SerialPlot"
 * @param	*pStruct:	Указатель на структуру в которую необходимо скопировать
 * 						переменные;
 * @param	data: 	Переменная которую необходимо скопировать в стуктуру
 * 					типа "DI_data_for_serial_plot_s";
 * @param	...		Переменное число параметров, которое необходимо скопировать
 * 					в стуктуру типа "DI_data_for_serial_plot_s";
 * @param	0xFFFFFFFF:	Терминальный символ, по которому функция определяет
 * 						конец передаваемых в функцию параметров;
 * @return	Количество байт данных, которое необходимо отправить в программу "SerialPlot"
 */
size_t DI_CopyDataForSerialPlot_f32(DI_data_for_serial_plot_s *pStruct,
                                    float data,
                                    ...)
{
    pStruct->beginMessageId = 0xAA;

    //	Объявление указателя переменное число параметров функции;
    va_list pInParam;

    //	Инициализация указателя;
    va_start(pInParam, data);

    float dataTmp = data;

    // Счетчик каличества переменных типа "float";
    size_t i = 0;

    while (dataTmp != (float) 0xFFFFFFFF)
    {
        pStruct->dataArr[i] = dataTmp;
        i++;
        dataTmp = va_arg(pInParam, double);
    }

    pStruct->numbMessageBytes = i * 4;

    va_end(pInParam);

    // Возвращаем количество байт, которое необходимо отправить по шине данных;
    return (i * 4)
            + sizeof (pStruct->beginMessageId)
            + sizeof (pStruct->numbMessageBytes);
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/