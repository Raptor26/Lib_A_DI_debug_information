/**
 * File:   %<%NAME%>%.%<%EXTENSION%>%
 * Author: %<%USER%>%
 *
 * Created on %<%DATE%>%, %<%TIME%>%
 */

/******************************************************************************/
// Секция include: здесь подключается заголовочный файл к модулю
#include "Lib_A_DI_debug_information.h"
/******************************************************************************/


/******************************************************************************/
/*============================================================================*/
// Глобальные переменные
/*============================================================================*/


/*============================================================================*/
// Локальные переменные
/*============================================================================*/
/******************************************************************************/


/******************************************************************************/
// Секция прототипов локальных функций
/******************************************************************************/


/******************************************************************************/
// Секция описания функций (сначала глобальных, потом локальных)

/*============================================================================*/
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
            - sizeof (pPackageStruct->crc);

	pPackageStruct->currentAbsoluteAngel = currentAbsoluteAngel;
	pPackageStruct->needAngelElectAndCurrentAngelDiff = needAngelAndCurrentAngelDiff;
	pPackageStruct->currentAngelInElectAngel = currentAngelInElectAngel;
	pPackageStruct->needAngelInElectAngel = needAngelInElectAngel;
	pPackageStruct->amplitudeCurrent = amplitudeCurrent;
	pPackageStruct->needAbsoluteAngelAndCurrentAngelDiff = needAbsoluteAngelAndCurrentAngelDiff;
	pPackageStruct->angularSpeed = angularSpeed;
	pPackageStruct->angularSpeed2 = angularSpeed2;

    return 1;
}
/*============================================================================*/
/******************************************************************************/


////////////////////////////////////////////////////////////////////////////////
// END OF FILE
////////////////////////////////////////////////////////////////////////////////
