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

uint8_t DI_CopyDataInStructForSerialPlot(DI_package_for_serial_plot_s *pPackageStruct,
                                         float *pGyrArr, float *pAccArr, float *pMagArr,
                                         float *pQuatArr,
                                         float *pEulerAnglArr)
{
    pPackageStruct->beginMessageId = 0xAA;

    pPackageStruct->numbMessageBytes = sizeof (DI_package_for_serial_plot_s)
            - sizeof (pPackageStruct->beginMessageId)
            - sizeof (pPackageStruct->numbMessageBytes)
            - sizeof (pPackageStruct->crc);

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

    //  Копируем в стурктуру данные кватерниона;
    pPackageStruct->quatArr[0] = *pQuatArr++;
    pPackageStruct->quatArr[1] = *pQuatArr++;
    pPackageStruct->quatArr[2] = *pQuatArr++;
    pPackageStruct->quatArr[3] = *pQuatArr;

    pPackageStruct->eulerAnglArr[0] = *pEulerAnglArr++;
    pPackageStruct->eulerAnglArr[1] = *pEulerAnglArr++;
    pPackageStruct->eulerAnglArr[2] = *pEulerAnglArr;

    //  Расчет контрольной суммы;
    pPackageStruct->crc = CRC_XOR_Crc8((uint8_t*) & pPackageStruct->gyrArr[0],
                                       sizeof (DI_package_for_serial_plot_s) - 1);
    return 0;
}
/*============================================================================*/
/******************************************************************************/


////////////////////////////////////////////////////////////////////////////////
// END OF FILE
////////////////////////////////////////////////////////////////////////////////
