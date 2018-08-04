/**
 * @file   	Lib_A_DI_debug_information.h
 * @author 	Isaev Mickle
 * @version	v1.0
 * @date 	10.04.2018
 * @brief	Файл содержит описание структур и перечисляемых типов
 */

#ifndef LIB_A_DI_DEBUG_INFORMATION_H
#define	LIB_A_DI_DEBUG_INFORMATION_H

/*#### |Begin| --> Секция - "Include" ########################################*/
/*==== |Begin| --> Секция - "C libraries" ====================================*/
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
/*==== |End  | <-- Секция - "C libraries" ====================================*/

/*==== |Begin| --> Секция - "MK peripheral libraries" ========================*/
/*==== |End  | <-- Секция - "MK peripheral libraries" ========================*/

/*==== |Begin| --> Секция - "Extern libraries" ===============================*/
#include "../Lib_A_CRC_cyclic_redundancy_check/Lib_A_CRC_cyclic_redundancy_check.h"
/*==== |End  | <-- Секция - "Extern libraries" ===============================*/
/*#### |End  | <-- Секция - "Include" ########################################*/

/*#### |Begin| --> Секция - "Определение констант" ###########################*/
#if defined (DI_MAX_PLOTS_IN_PACKAGE)
#else
#error "Set max numb of plots in package"
#endif

#define DI_START_FRAME_SYMBOL   0xAAAAAAAA ///<     Start frame symbol
#define DI_TERMINAL_SYMBOL      0xAAAAAAAA ///<		Terminal symbol
/*#### |End  | <-- Секция - "Определение констант" ###########################*/

/*#### |Begin| --> Секция - "Определение типов" ##############################*/
typedef struct {
	uint16_t frameStart; ///< 	Символ старта пакета данных
	uint8_t frameSize; ///< 	Количество полезных байтов

	float dataArr[DI_MAX_PLOTS_IN_PACKAGE]; ///<	Данные, по которым SerialPlot построит графики

	uint8_t crc; ///<			Конрольная сумма пакета данных;
} __attribute__((__packed__)) DI_data_for_serial_plot_s;
/*#### |End  | <-- Секция - "Определение типов" ##############################*/

/*#### |Begin| --> Секция - "Определение глобальных переменных" ##############*/
/*#### |End  | <-- Секция - "Определение глобальных переменных" ##############*/

/*#### |Begin| --> Секция - "Прототипы глобальных функций" ###################*/
extern uint16_t DI_CopyDataForSerialPlot_f32(
	DI_data_for_serial_plot_s *pStruct,
	float data,
	...);
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/

/*#### |Begin| --> Секция - "Определение макросов" ###########################*/
/*#### |End  | <-- Секция - "Определение макросов" ###########################*/

#endif	/* LIB_A_DI_DEBUG_INFORMATION_H */

/*############################################################################*/
/*################################ END OF FILE ###############################*/
/*############################################################################*/
