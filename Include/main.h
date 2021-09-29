//-----Поключение библиотек-----//

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f051x8.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "timers.h"  
#include "cmsis_os.h"   // osDelayUntil
 
//-----Основы-----//

#define APBCLK   48000000UL
#define BAUDRATE 115000UL

TaskHandle_t xDMA1_Channel1_IRQ_Handle;             //задачи с учетом ДМА и прерываний

xQueueHandle Signal1PeriodData;                     // Обращение к очереди для передачи из прерывания  значения периода сигнала
xQueueHandle USARTPeriodPrint;                      // Обращение к очереди для восприятия значения периода сигнала
xQueueHandle USARTRMSPrint;                         // Обращение к очереди  значения RMS - предельная синусоидальная мощность сигнала
xQueueHandle FrequencyERRORPrint;                   // Обращение к очереди для вызова ошибки
xQueueHandle Integr_Simpl_Queue;                    // Обращение к очереди сумм квадратов для рассчета квадратных корней

xSemaphoreHandle xBinarySemaphoreTIM15;             // Семафор прерывания счета периода таймером
xSemaphoreHandle xBinarySemaphoreDMA1_Channel1;     // Семафор прерывания окончания передачи значений массива значений АЦП

SemaphoreHandle_t xCountingSemaphoreADCSpeedNormal; // Счетный семафор прерывания TIM14. Нужен для контроля успеваем ли
                                                    // измерить, передать и посчитать значения симпла
                                                    // до запуска начала новых измерений

//-----FreeRTOS-----//

void generator(uint32_t frequency);
void LedsInit (void);
void TIM1_Init(void);
void ADC_Init(void);
void TIM14_Init(uint16_t psc, uint16_t arr);
void TIM15_Init(void);
uint16_t sqrt_GLS (uint32_t x);

//-----USART-----//

void USART1_Init (void);
void USART1_Error (void);
void USART1_Line_Rdy (void);
void USART1_Send (char chr);
void USART1_Send_String (char* str);

//-----Сигналы-----//

void vTaskLed1 (void *argument);
void vTaskLed2 (void *argument);
void vTaskSignal1PeriodMeasuring (void *argument);
void vTaskUSART1Print (void *argument);

//-----Прерывания-----//

void vTaskTIM15_IRQ (void *argument);
void vTaskDMA1_Channel1_IRQ (void *argument);
void vTaskPeriod_End (void *argument);




#ifdef __cplusplus
}
#endif

#endif