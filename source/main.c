//
// Основной файл
//


#include <main.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

const uint16_t MeasuringCountersPrescaler = 480-1;

const uint8_t Period_sample = 24;                   //количество измеряемых точек в период
const uint16_t T_N_Simpl = 32;                      //размер массива выборки значений

volatile uint32_t ADC1_Simpl [32] = { 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0 };

uint32_t Filtr_A = 0;
uint32_t Filtr_B = 0;
uint32_t Filtr_C = 0;
uint32_t Filtr_D = 0;


int main (void) {
    Leds_Init();
    USART_Init();

    GPIOC-> ^= GPIO_ODR_9; //led инициализация
    generator(167);

    USART1_Send_String ("\r\nHonest RMS Voltage Measurement\r\n");
            USART1->CR1 |= USART_CR1_CMIE;                  // Enable CM interrupt прерывание Character match (совпадение символов)
            USART1->CR1 |= USART_CR1_RXNEIE;                // RXNE Int ON
            USART1->CR1 |= USART_CR1_TXEIE;                 // TXNE Int ON


    TIM1_Init();
    TIM15_Init();
    ADC_Init();

    NVIC->ISER[0] = (1 << ((ADC1_COMP_IRQn) & 0x1F));               // Разрешить прерывание ADC1_COMP_IRQn
    NVIC->ISER[0] = (1 << ((DMA1_Channel1_IRQn) & 0x1F));           // Разрешить прерывание DMA1_Channel1_IRQn



    //  Error[Li005]: no definition for "xQueueCreateCountingSemaphore"
    // Не работает без #define configUSE_COUNTING_SEMAPHORES            1       во FreeRTOSConfig.h


    xTaskCreate(vTaskUSART1Print, "ConsoleOut", 128, NULL, 0, NULL);    // Задача привратника
    xTaskCreate(vTaskSignal1PeriodMeasuring, "TIM15Start", 32, NULL, 0, NULL); // Задача запускающая TIM15 в резиме сравнения для подсчета периода сигнала
    xTaskCreate(vTaskPeriod_End, "SQRTGLS", 64, NULL, 0, NULL); // Задача расчета квадратных корней и действий с плавающей точкой

    xTaskCreate(vTaskLed1, "LED1", 32, NULL, 0, NULL);
    xTaskCreate(vTaskLed2, "LED2", 32, NULL, 0, NULL);



    Signal1PeriodData   = xQueueCreate(1, sizeof(uint16_t));    // Очередь значений периода сигнала
    USARTPeriodPrint    = xQueueCreate(2, sizeof(uint16_t));    // Очередь значений периода сигнала на вывод
    Integr_Simpl_Queue  = xQueueCreate(4, sizeof(uint32_t));    // Очередь значений сумм квадратов симплов по 4 каналам для расчета квадратных корней
    USARTRMSPrint       = xQueueCreate(4, sizeof(float));       // Очередь значений RMS по 4 каналам на вывод
    FrequencyERRORPrint = xQueueCreate(2, sizeof(uint8_t));     // Очередь печати сообщения об ошибке, когда TIM14 обновляется быстрей, чем считает vTaskTIM15_IRQ

    vSemaphoreCreateBinary(xBinarySemaphoreTIM15);              // Разблокирует отложенный обработчик прерывания TIM15
    if( xBinarySemaphoreTIM15 != NULL )
    {
//           xSemaphoreTake(xBinarySemaphoreTIM15, 0);
        xTaskCreate(vTaskTIM15_IRQ, "TIM15CC1IF", 64, NULL, 1, NULL); // Задача разблокируемая в прерывании TIM15 передает в TIM14 время семплирования
    }

    vSemaphoreCreateBinary(xBinarySemaphoreDMA1_Channel1);      // Разблокирует отложенный обработчик прерывания DMA1_Channel1
    if( xBinarySemaphoreDMA1_Channel1 != NULL )
    {
        xSemaphoreTake(xBinarySemaphoreDMA1_Channel1, 0);
        xTaskCreate(vTaskDMA1_Channel1_IRQ, "DMA_ISR_TCIF1", 64, NULL, 3, &xDMA1_Channel1_IRQ_Handle); // Задача формирует интеральную сумму квадратов симплов по 4 каналам
    }

//    xCountingSemaphoreADCSpeedNormal = xSemaphoreCreateCounting(100000,0);     // Семафор для блокировки опережающего расчеты запуска АЦП (Можно использовать очередь)
//    Удаляется и создается в vTaskTIM15_IRQ

    vTaskStartScheduler();


    while(1)
    {

    }

}

void LedsInit (void){

    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Pin 8 Синенький светодиод Дискавери
    GPIOC->MODER         |= GPIO_MODER_MODER8_0;       // 01 (GPIO_MODER_MODER0_0) - выход
    GPIOC->PUPDR         |= GPIO_PUPDR_PUPDR8_1;       // подтяжка к "-"
    GPIOC->OSPEEDR       |= GPIO_OSPEEDER_OSPEEDR8;    // 50МГц

    // Pin 9 Зелененький светодиод Дискавери
    GPIOC->MODER         |= GPIO_MODER_MODER9_0;       // 01 (GPIO_MODER_MODER0_0) - выход
    GPIOC->PUPDR         |= GPIO_PUPDR_PUPDR9_1;       // подтяжка к "-"
    GPIOC->OSPEEDR       |= GPIO_OSPEEDER_OSPEEDR9;    // 50МГц
}

void vTaskLed1 (void *argument){

    while(1)
    {
        // Pin 8 Синенький светодиод Дискавери
        //		GPIOC->ODR ^= GPIO_ODR_8;
        GPIOC->BSRR |= GPIO_BSRR_BS_8;
        vTaskDelay(1000);
        GPIOC->BSRR |= GPIO_BSRR_BR_8;
        vTaskDelay(1000);


    }

}
void vTaskLed2 (void *argument){

    while(1)
    {
        // Pin 9 Зелененький светодиод Дискавери
        GPIOC->ODR ^= GPIO_ODR_9;
        vTaskDelay(1000);
    }

}

void vTaskSignal1PeriodMeasuring (void *argument){
    uint32_t tickcount;

    while(1)
    {


        tickcount = osKernelSysTick();


//                  TIM15->CR1    &= ~TIM_CR1_CEN;
//                  NVIC_DisableIRQ(TIM15_IRQn);          // выключаем прерывание

        TIM15->EGR    |= TIM_EGR_UG;          // генерация обновления
        NVIC_EnableIRQ(TIM15_IRQn);           // включаем прерывание
        TIM15->CR1    |= TIM_CR1_CEN;         // Запускаем счет

        osDelayUntil(&tickcount, 1000);      // Включается, бля, в FreeRTOSConfig.h

    }
}
// Функция обработчика отложенного прерывания TIM15
// После получения значения периода сигнала, мы разбиваем его на количество симплов Period_sampling
// и инициализируем этим значением TIM14. По каждому прерыванию по переполнению TIM14 запускаем
// ADC с DMA по группе из 4 каналов на 8 измерений для последующего усреднения

void vTaskTIM15_IRQ (void *argument){

    uint16_t PeriodTime;

    while(1)
    {
        xSemaphoreTake(xBinarySemaphoreTIM15, portMAX_DELAY);

        if (uxQueueMessagesWaiting(Signal1PeriodData) != 0){

            xQueueReceive(Signal1PeriodData, &PeriodTime, 0);

            vSemaphoreDelete (xCountingSemaphoreADCSpeedNormal);
            // Не удаляйте семафор, для которого заблокированы задачи (задачи, которые находятся в состоянии «Заблокировано», ожидая, пока семафор станет доступным).
            xCountingSemaphoreADCSpeedNormal = xSemaphoreCreateCounting(100000,0);     // Семафор для блокировки опережающего расчеты запуска АЦП (Можно использовать очередь)

            if ((PeriodTime / Period_sampling) > 0) {
                TIM14_Init(MeasuringCountersPrescaler, PeriodTime / Period_sampling);
                // В противном случае счетчик считает до 0xffff
            }

            xQueueSend(USARTPeriodPrint, &PeriodTime, 0); // Период на печать

        }

    }
}

void vTaskDMA1_Channel1_IRQ (void *argument){

    // Если переменная была продекларирована как статическая (static) – в этом случае будет существовать только одна копия
    // переменной, и она будет использоваться совместно всеми созданными экземплярами задачи
    static uint8_t Period_calculation_counter = 1;        // Счетчик когда все квадраты суммированы и можно отправлять считать корни

    static uint8_t Average_counter = 0;// Счетчик выборки значений (8 измерений в каждой точке по 4 каналам)
    int32_t Average_Frame_A = 0;       // Переменные суммы выборки 8 раз по 4 датчикам
    int32_t Average_Frame_B = 0;
    int32_t Average_Frame_C = 0;
    int32_t Average_Frame_D = 0;

    static uint32_t Integr_Sin_A = 0;
    static uint32_t Integr_Sin_B = 0;
    static uint32_t Integr_Sin_C = 0;
    static uint32_t Integr_Sin_D = 0;

    static uint8_t SemaphoreADCSpeedNormal_counter = 0;
    static uint8_t FrequencyERROR = 0;

//  static uint32_t tick_print_count_old = 0;
//         uint32_t tick_print_count_new;


    while(1)
    {
        xSemaphoreTake(xBinarySemaphoreDMA1_Channel1, portMAX_DELAY);

        while (Average_counter < T_N_Simpl) {
            Average_Frame_A += ADC1_Simpl [Average_counter++];
            Average_Frame_B += ADC1_Simpl [Average_counter++];
            Average_Frame_C += ADC1_Simpl [Average_counter++];
            Average_Frame_D += ADC1_Simpl [Average_counter++];
        }
        Average_counter = 0;

        xSemaphoreTake(xCountingSemaphoreADCSpeedNormal, 0);
        // Уменьшим значение семафора счетчика ресурсов
        // Мы обработали и освободили массив ADC1_Simpl[]

        SemaphoreADCSpeedNormal_counter = uxSemaphoreGetCount (xCountingSemaphoreADCSpeedNormal);
        if (SemaphoreADCSpeedNormal_counter == 0) {
            // Если на этот момент не случилось еще несколько прерываний TIM14, обрабатываем данные, иначе ошибка
            asm("nop");   // Удобно для тестирования, как точка останова

/*              xQueueReceive(SignalSimplNumber, &i, 0);        // Очистить очередь из прерывания TIM14

              if (uxQueueMessagesWaiting(SignalSimplNumber) > 1){                      // Если очередь не пуста, то мы не успели посчитать до очередного запуска ADC

                  Calculation_Error();
              }
*/

            // У нас 8 выброк по каждому каналу. Усредним их.
            Average_Frame_A = (Average_Frame_A >> 3) - Zero_OUT_Filtr_A;
            Average_Frame_B = (Average_Frame_B >> 3) - Zero_OUT_Filtr_B;
            Average_Frame_C = (Average_Frame_C >> 3) - Zero_OUT_Filtr_C;
            Average_Frame_D = (Average_Frame_D >> 3) - Zero_OUT_Filtr_D;
            // У датчиков в предыдущих моих примерах было "странное" понимание нуля, поэтому их приходилось калибровать статистически.

            /* Дальше все равно возводим в квадрат
            if (Average_Frame_A < 0) Average_Frame_A = - Average_Frame_A;     // Среднее значения тока положительно
            if (Average_Frame_B < 0) Average_Frame_B = - Average_Frame_B;
            if (Average_Frame_C < 0) Average_Frame_C = - Average_Frame_C;
            if (Average_Frame_U < 0) Average_Frame_U = - Average_Frame_U;
            */

            // Честно интегрируем по 24-м точкам
            Integr_Sin_A += Average_Frame_A * Average_Frame_A;     // Нам нужно среднеквадратичное значение
            Integr_Sin_B += Average_Frame_B * Average_Frame_B;     // Сумма квадратов B
            Integr_Sin_C += Average_Frame_C * Average_Frame_C;     // Сумма квадратов C
            Integr_Sin_D += Average_Frame_D * Average_Frame_D;     // Сумма квадратов D
            // Здесь можно заметить, что разрядность АЦП 12 бит и мы влезаем в uint32_t суммой из 24 точек

            Average_Frame_A = 0;
            Average_Frame_B = 0;
            Average_Frame_C = 0;
            Average_Frame_D = 0;


            if (++Period_calculation_counter > Period_sampling) {

                if (uxQueueMessagesWaiting(Integr_Simpl_Queue) == 0)      {

                    xQueueSend(Integr_Simpl_Queue, &Integr_Sin_A, 0); // Считать корни
                    xQueueSend(Integr_Simpl_Queue, &Integr_Sin_B, 0); // Считать корни
                    xQueueSend(Integr_Simpl_Queue, &Integr_Sin_C, 0); // Считать корни
                    xQueueSend(Integr_Simpl_Queue, &Integr_Sin_D, 0); // Считать корни
                }
                Period_calculation_counter = 1;

                Integr_Sin_A = 0;
                Integr_Sin_B = 0;
                Integr_Sin_C = 0;
                Integr_Sin_D = 0;
            }
        }
        else {

            Average_Frame_A = 0;
            Average_Frame_B = 0;
            Average_Frame_C = 0;
            Average_Frame_D = 0;

            xQueueSend(FrequencyERRORPrint, &FrequencyERROR, 0); // Ошибку на печать
            SemaphoreADCSpeedNormal_counter++;                        // Мы уменьшали семафор перед проверкой условия на ошибку
            xQueueSend(FrequencyERRORPrint, &SemaphoreADCSpeedNormal_counter, 0); // Ошибку на печать

        }
    }
}


//______________________________________________________________________________
void vTaskPeriod_End (void *argument){

    uint32_t Integr_Sin_A = 0;
    uint32_t Integr_Sin_B = 0;
    uint32_t Integr_Sin_C = 0;
    uint32_t Integr_Sin_D = 0;

    float   A_RMS = 0.0;
    float   B_RMS = 0.0;
    float   C_RMS = 0.0;
    float   D_RMS = 0.0;

    uint32_t AveragingCounter = 0;
    float   SUM_A_RMS = 0.0;
    float   SUM_B_RMS = 0.0;
    float   SUM_C_RMS = 0.0;
    float   SUM_D_RMS = 0.0;


    while(1)
    {
//              if( uxQueueMessagesWaiting(Integr_Simpl_Queue) == 4)      {
        // Кратность очереди четырем обеспечивается ее полным заполнением в задаче vTaskDMA1_Channel1_IRQ
        // Поскольку это псевдообработчик прерывания с высоким приоритетом, то этот процесс заполнения ни кем не прерывается

        xQueueReceive(Integr_Simpl_Queue, &Integr_Sin_A, portMAX_DELAY);
        xQueueReceive(Integr_Simpl_Queue, &Integr_Sin_B, portMAX_DELAY);
        xQueueReceive(Integr_Simpl_Queue, &Integr_Sin_C, portMAX_DELAY);
        xQueueReceive(Integr_Simpl_Queue, &Integr_Sin_D, portMAX_DELAY);
        // portMAX_DELAY - уводит задачу в блокировку, точно так же как vTaskDelay(100);
        // Что дает раздышаться другим задачам. Однако при появлении данных в очереди (всех данных см. выше)
        // Управление сразу передается сюда по программному прерыванию приоритета нашей этой задачи.
//              }

        A_RMS = sqrt_GLS(Integr_Sin_A / Period_sampling) / 4095.0 * 2.98;      // Честный расчет действующего значения
        B_RMS = sqrt_GLS(Integr_Sin_B / Period_sampling) / 4095.0 * 2.98;      // stm32f051x8 не поддерживает аппаратное деление с плавающей точкой
        C_RMS = sqrt_GLS(Integr_Sin_C / Period_sampling) / 4095.0 * 2.98;      // Корень считется в целых числах итерационным методом
        D_RMS = sqrt_GLS(Integr_Sin_D / Period_sampling) / 4095.0 * 2.98;      // Так что все тут очень не быстро, но как получится. Все равно глазами смотреть.
        // Для любви к искусству
        // ACS722LLCTR-05AB-T2 от -5 до 5 А 264 mV/A  при VCC 3,3 V. от 1,65 V.
        // 4095.0*3.3/0.264; - ACS722LLCTR-05AB-T2 Датчик тока на эффекте Холла.
        // У платы STM32F0DISCOVERY стоит на входе диод Шотки BAT60JFILM. Поэтому напряжение питания цилоскоп показывает 2,98 V

        if (uxQueueMessagesWaiting(USARTRMSPrint) == 0)      {
            // Счетчики показывают, что процессор успевает за АЦП считать корни делить и умножать с плавающей точкой
            // Блокируя задачу вывода vTaskDelay(100) можно усреднять значение величины
            // Если приблизить это время к 1 секунде, то усреднение получается на коэффициент значения частоты в Гц
            if (AveragingCounter != 0)        {
                A_RMS = SUM_A_RMS / AveragingCounter;
                B_RMS = SUM_B_RMS / AveragingCounter;
                C_RMS = SUM_C_RMS / AveragingCounter;
                D_RMS = SUM_D_RMS / AveragingCounter;

                AveragingCounter = 0;
                SUM_A_RMS = 0.0;
                SUM_B_RMS = 0.0;
                SUM_C_RMS = 0.0;
                SUM_D_RMS = 0.0;
            }

            xQueueSend(USARTRMSPrint, &A_RMS, 0); // Выводить RMS
            xQueueSend(USARTRMSPrint, &B_RMS, 0); // Выводить RMS
            xQueueSend(USARTRMSPrint, &C_RMS, 0); // Выводить RMS
            xQueueSend(USARTRMSPrint, &D_RMS, 0); // Выводить RMS
        }
        else  {
            AveragingCounter++;
            SUM_A_RMS += A_RMS;
            SUM_B_RMS += B_RMS;
            SUM_C_RMS += C_RMS;
            SUM_D_RMS += D_RMS;
        }

    }

}

void vTaskUSART1Print (void *argument){

    uint16_t PeriodTime;
    uint8_t ErrorFlag;
    uint8_t FrequencyDivider; // Во сколько раз TIM14 запускает ADC1 чаще, чем последний успевает слить данные в DMA1_Channel1 и их сумировать в xDMA1_Channel1_IRQ_Handle
    char outstr[60];

    float   Period_Print = 0.0;
    float   Frequency_Print = 0.0;

    float   A_RMS = 0.0;
    float   B_RMS = 0.0;
    float   C_RMS = 0.0;
    float   D_RMS = 0.0;



    while(1)
    {

        if (uxQueueMessagesWaiting(USARTPeriodPrint) != 0){
            // Если поставить xQueueReceive(USARTPeriodPrint, &data, portMAX_DELAY); то в блокировку выкидывает всю задачу
            // включая xQueueReceive(USARTRMSPrint, &A_RMS, portMAX_DELAY); что аналогично vTaskDelay(1000);

            xQueueReceive(USARTPeriodPrint, &PeriodTime, 0);
            Period_Print = PeriodTime / 100.0;
            Frequency_Print = 1.0 / Period_Print *1000.0;
            memset(outstr, 0, 60);
            if (PeriodTime != 0) sprintf(outstr,"\r\nsignal period = %3.2f mS,  frequency = %4.2f Hz\r\n\r\n", Period_Print, Frequency_Print);
            else                 sprintf(outstr,"\r\n Error! Reduce signal frequency! \r\n\r\n");
            USART1_Send_String(&outstr[0]);

        }

        if (uxQueueMessagesWaiting(FrequencyERRORPrint) == 2){
            xQueueReceive(FrequencyERRORPrint, &ErrorFlag, 0);
            xQueueReceive(FrequencyERRORPrint, &FrequencyDivider, 0);
            memset(outstr, 0, 60);
            sprintf(outstr,"\r\n Error! Reduce signal frequency \r\n");
            USART1_Send_String(&outstr[0]);
            if (FrequencyDivider > 1) {
                memset(outstr, 0, 60);
                sprintf(outstr," by about %d times! \r\n\r\n", FrequencyDivider);
                USART1_Send_String(&outstr[0]);
            }

        }



        if (uxQueueMessagesWaiting(USARTRMSPrint) == 4){

            xQueueReceive(USARTRMSPrint, &A_RMS, 0);
            xQueueReceive(USARTRMSPrint, &B_RMS, 0);
            xQueueReceive(USARTRMSPrint, &C_RMS, 0);
            xQueueReceive(USARTRMSPrint, &D_RMS, 0);
            memset(outstr, 0, 60);
            sprintf(outstr,"RMS [v] = %3.2f, %3.2f, %3.2f, %3.2f\r\n", A_RMS, B_RMS, C_RMS, D_RMS);
            USART1_Send_String(&outstr[0]);

        }
        vTaskDelay(900);       // См. выше.

    }

}




