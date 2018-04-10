/**
 * @file   	Lib_A_DI_debug_information.c
 * @author 	Isaev Mickle
 * @version	beta
 * @date 	10.04.2018
 * @brief	Библиотека содержит функции для формирования пакета данных, который
 * 			будет переведен программной SerialPlot в график/графики (в
 * 			зависимости от количества переданных параметров)
 * 		@see	Скачать программу SerialPlot можно отсюд:
 * 				https://hackaday.io/project/5334-serialplot-realtime-plotting-software
 *
 * @code
 * 		// Пример использования функции DI_CopyDataForSerialPlot_f32()
 * 		...
 * 		DI_data_for_serial_plot_s dataForSerialPlotStruct;
 *		...
 * 		int main (void)
 * 		{
 * 			...
 * 			while (1)
 * 			{
 * 				...
 * 				// Копирование переменных, по которым необходимо построить
 * 				// графики в программе SerialPlot
 * 				uint16_t bytesCnt = DI_CopyDataForSerialPlot_f32(&dataForSerialPlotStruct,
 * 											 					 (float) data1,
 * 											 					 (float) data2,
 * 											 					 (float) data3,
 * 											 					 (float) data4,
 * 											 					 (float) data5,
 * 											 					 (float) DI_TERMINAL_SYMBOL)
 *
 * 				// Передача пакета данных по интерфейсу UART в ПК
 * 				UART_Tramsmitter((uint8_t*) &DI_CopyDataForSerialPlot_f32,
 * 								 bytesCnt);
 * 				...
 * 			}
 * 			return 1;
 * 		}
 * @endcode
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
/**
 * @brief	Функция выполняет копирование переменных в структуру типа
 * 			"DI_data_for_serial_plot_s" для дальнейшей отправки в программу "SerialPlot"
 * @param[out]	*pStruct:	Указатель на структуру в которую необходимо скопировать
 * 							переменные
 * @param[in]	data: 	Переменная которую необходимо скопировать в стуктуру
 * 						типа "DI_data_for_serial_plot_s"
 * @param[in]	...		Переменное число параметров, которое необходимо скопировать
 * 						в стуктуру типа "DI_data_for_serial_plot_s"
 * @param[in]	0xAAAAAAAA:	Терминальный символ, по которому функция определяет
 * 							конец передаваемых в функцию параметров
 * @return	Размер получившегося пакета данных в байтах
 */
uint16_t DI_CopyDataForSerialPlot_f32(
    DI_data_for_serial_plot_s *pStruct,
    float data,
    ...)
{
	pStruct->frameStart = 0xAAAA;

	/* Объявление указателя на переменное число параметров функции */
	va_list pInParam;

	/* Инициализация указателя */
	va_start(pInParam,
	         data);

	float dataTmp = data;

	/* Счетчик количества переменных типа "float" */
	size_t i = 0;

	while (dataTmp != (float) DI_TERMINAL_SYMBOL)
	{
		pStruct->dataArr[i] = (float) dataTmp;
		i++;
		dataTmp = va_arg(pInParam,
		                 double);
	}

	pStruct->frameSize = i * sizeof(float);

	va_end(pInParam);

	/* Возвращает количество байт, которое необходимо отправить в программу
	 * SerialPlot */
	return (i * 4)
	       + sizeof(pStruct->frameStart)
	       + sizeof(pStruct->frameSize);
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/

/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/

/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
