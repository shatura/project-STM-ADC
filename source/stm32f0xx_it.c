//
// файл по настройке усарт у стм, найден
//

#include "stm32f051x8.h"

#include "FreeRTOS.h"
#include "semphr.h"


extern const uint8_t T_N_Simpl;                       // Конец массива выборки значений
extern volatile uint32_t ADC1_Simpl [32];

extern xSemaphoreHandle xBinarySemaphoreTIM15;
extern xSemaphoreHandle xBinarySemaphoreDMA1_Channel1;

extern SemaphoreHandle_t xCountingSemaphoreADCSpeedNormal;

extern xQueueHandle Signal1PeriodData;


void USART1_Send_String (char* str);
void USART1_Error (void);
void USART1_Line_Rdy (void);
void DAC_ConversionSpeedError (void);


//______________________________________________________________________________
void  TIM1_BRK_UP_TRG_COM_IRQHandler (void) // Прерывание при сбое, обновлении, пуске и коммутации таймера TIM1
{

    if(TIM1->SR & TIM_SR_UIF)   {

        TIM1->SR &= ~TIM_SR_UIF;     // Сбрасываем флаг прерывания по переполнению


//    TIM1->CCR1 = *p_Sin_A;
//    TIM1->CCR2 = *p_Sin_B;
//    TIM1->CCR3 = *p_Sin_C;
//TIM1->CR1 |= TIM_CR1_UDIS;      // Запретить обновление
//TIM1->DIER &= ~TIM_DIER_UIE;    // Запретить прерывание по переполнению
// Разрешать мы его будем в TIM2 после загрузки очередных значений ШИМ
    }
/*
//-----
  if(TIM1->SR & TIM_SR_BIF)   {
    TIM1->SR &= ~TIM_SR_BIF;     // Сбрасываем флаг прерывания
// Сбрасываем флаг аварийного останова, причем он не сбросится, пока PB12 != 0.
                              }
*/
}
//______________________________________________________________________________
void  TIM6_DAC_IRQHandler (void)    // Прерывание от TIM6 & DAC
{

    if(DAC->SR & DAC_SR_DMAUDR1)   {    // Это флаг опустошения DMA. Когда DAC не успел завершить преобразование, а следующий запрос уже пришел
        DAC->CR &= ~DAC_CR_DMAEN1;  // Выключить DMA, понизить частоту и по новой все инициализировать
        DAC->SR |= DAC_SR_DMAUDR1;  // DAC channel1 DMA underrun flag в мануале сказано очистить написав «1»
        // Но не сказано когда. Не чистится до отключения DMA
        DAC_ConversionSpeedError ();
    }
}

//______________________________________________________________________________
void  TIM14_IRQHandler (void)    // Прерывание от TIM6 & DAC
{
    uint16_t FrequencyFlag = 0;

    if(TIM14->SR & TIM_SR_UIF)   {
        TIM14->SR &= ~TIM_SR_UIF;     // Сбрасываем флаг прерывания

//        xQueueSendToBackFromISR(SignalSimplNumber, &FrequencyERROR, NULL);
        xSemaphoreGiveFromISR(xCountingSemaphoreADCSpeedNormal, 0);
        FrequencyFlag = uxSemaphoreGetCount(xCountingSemaphoreADCSpeedNormal);
        // TIM14->SR & TIM_SR_UIF живет своей жизнью и начинает отрабатывать при TIM_EGR_UG при инициализации
        // Нам нужно, чтобы он запускал АЦП только один раз пока данные не будут обработаны
        // После обработки данных семафор уменьшится в программе обработки и снова увеличится до 1 здесь
        if (FrequencyFlag == 1) {

            //!!!!!
            ADC1->CR |= ADC_CR_ADSTP;               // ADC stop of conversion command
            while (ADC1->CR & ADC_CR_ADSTART);      // Дождаться пока ADSTART будет равен нулю
            // Пугают! По остановам в цикле Всегда была ОДНА итерация
            ADC1->ISR |= ADC_ISR_EOSEQ;             // Сбросить флаг окончания последовательности. Написано сбрасывается записью 1
            ADC1->IER |= ADC_IER_EOSIE;             // Разрешить прерывания по концу последовательности каналов АЦП
            ADC1->CR |= ADC_CR_ADSTART;
        }
    }
}

//______________________________________________________________________________
//RM0091 pg. 951 A.9.4 Input capture data management code example
void  TIM15_IRQHandler (void)    // Прерывание по захвату
{
    static portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    static uint8_t gap = 0;
    static uint32_t counter = 0;
    static uint32_t counter0 = 0;
    static uint32_t counter1 = 0;

// Примечание. Этот код управляет только переполнением одного счетчика.
// Чтобы управлять множеством переполнений счетчиков, необходимо включить прерывание обновления (UIE = 1) и правильно управлять им.
    if(TIM15->SR & TIM_SR_CC1IF)
    {

        if(TIM15->SR & TIM_SR_CC1OF)            // Check the overflow
        {
            // Overflow error management      Эта хрень обнуляет то, что творилось до включения прерывания, когда TIM15->CCR1 не считывался
            gap = 0; // Reinitialize the laps computing                                       Повторная инициализация этапа вычислений
            TIM15->SR &= ~(TIM_SR_CC1OF | TIM_SR_CC1IF); // Clear the flags
            return;
        }

        if (gap == 0) // Test if it is the first rising edge                                          Проверьте, является ли это первым нарастающим фронтом
        {
            counter0 = TIM15->CCR1; // Read the capture counter which clears the CC1ICF               Считайте счетчик захвата, который очищает CC1ICF
            gap = 1; // Indicate that the first rising edge has yet been detected                     Укажите, что первый нарастающий фронт еще не обнаружен
        }
        else
        {
            counter1 = TIM15->CCR1; // Read the capture counter which clears the CC1ICF               Считайте счетчик захвата, который очищает CC1ICF
            if (counter1 > counter0) // Проверить переполнение счетчика захвата
            {
                counter = counter1 - counter0;
            }
            else
            {
                counter = counter1 + 0xFFFF - counter0 + 1;
            }
            counter0 = counter1;

            TIM15->CR1    &= ~TIM_CR1_CEN;
            NVIC_DisableIRQ(TIM15_IRQn);          // выключаем прерывание

            xQueueSendToBackFromISR(Signal1PeriodData, &counter, &xHigherPriorityTaskWoken);

            xSemaphoreGiveFromISR(xBinarySemaphoreTIM15, &xHigherPriorityTaskWoken);

            if( xHigherPriorityTaskWoken == pdTRUE )
            {
                portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
            }

        }
    }

    else
    {
// Unexpected Interrupt Непредвиденное прерывание
// Manage an error for robust application Управление ошибкой для надежного приложения
    }

}

//______________________________________________________________________________
void  ADC1_COMP_IRQHandler (void)    // Прерывания АЦП
{
/*
  if(ADC1->ISR & ADC_ISR_AWD)   {       // Прерывания от сторожевого таймера

  TIM1->EGR |= TIM_EGR_BG;                // Установить флаг прерывания аварийного останова TIM1

  ADC1->ISR |= ADC_ISR_AWD;             // Написано сбрасывается записью 1
                                }
*/
//-----
    if(ADC1->ISR & ADC_ISR_EOSEQ)   {       // Прерывания по концу последовательности преобразования каналов
        ADC1->ISR |= ADC_ISR_EOSEQ;           // Написано сбрасывается записью 1
//-----
        ADC1->CR |= ADC_CR_ADSTP;               // ADC stop of conversion command
        while (ADC1->CR & ADC_CR_ADSTART);      // Дождаться пока ADSTART будет равен нулю
// Пугают! По остановам в цикле Всегда была ОДНА итерация
        ADC1->IER &= ~ADC_IER_EOSIE;            // запретить прерывания по концу последовательности каналов АЦП
        ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;      // Включить передачу DMA на АЦП в круговом режиме
        ADC1->CR |= ADC_CR_ADSTART;             // ADC start of conversion

        DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR));         //  Настроить адрес регистра периферийных данных
        DMA1_Channel1->CMAR = (uint32_t)(&ADC1_Simpl[0]);       //  Настроить адрес памяти
        DMA1_Channel1->CNDTR = T_N_Simpl;       //  Настроить количество байт DMA-передачи, которое должно выполняться по каналу DMA 1
        DMA1_Channel1->CCR |= DMA_CCR_EN;                       // Включить канал DMA 1

        DMA1->IFCR = DMA_IFCR_CGIF1;           // 1: очищает флаги GIF, TEIF, HTIF и TCIF в регистре DMA_ISR
        DMA1_Channel1->CCR |= DMA_CCR_TCIE;                     //  Разрешение прерывания с полной передачей
    }
}

//______________________________________________________________________________
void  DMA1_Channel1_IRQHandler (void)    // Прерывания DMA ADC1
{
//  static portBASE_TYPE xHigherPriorityTaskWoken;
    static portBASE_TYPE PoFig;
    // Переменная называется везде одинаково, но не несет смысловой нагрузки
    // Единственный фокус - имя семафора. По этому имени RTOS выбирает приоритетную задачу, которую нужно разблокировать.
    PoFig = pdFALSE;

    if (DMA1->ISR & DMA_ISR_TCIF1)   {    // Channel 1 Transfer Complete flag
        DMA1_Channel1->CCR &= ~DMA_CCR_TCIE;    //  Запретить прерывания с полной передачей
        DMA1->IFCR = DMA_IFCR_CGIF1;            // 1: очищает флаги GIF, TEIF, HTIF и TCIF в регистре DMA_ISR
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;      // Выключить канал DMA1
        ADC1->CR |= ADC_CR_ADSTP;               // ADC stop of conversion command
        ADC1->CFGR1 &= ~(ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG);      // Выключить передачу DMA на АЦП в круговом режиме
        //ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;        // Прекращаем читать данные.
        ADC1->CR |= ADC_CR_ADSTART;             // ADC start of conversion

        xSemaphoreGiveFromISR(xBinarySemaphoreDMA1_Channel1, &PoFig);

        if (PoFig == pdTRUE)
        {
            portEND_SWITCHING_ISR(PoFig);
        }

    }
}
//______________________________________________________________________________
void USART1_IRQHandler (void) {

    if (USART1->ISR & USART_ISR_RXNE)               // Флаг сбрасывается сам по чтению регистра данных
    {
        if (USART1->RDR == '0') {

            USART1_Send_String("OFF\r\n");
            GPIOC->BSRR |= GPIO_BSRR_BR_8;
        }

        if (USART1->RDR == '1') {

            USART1_Send_String("ON\r\n");
            GPIOC->BSRR |= GPIO_BSRR_BS_8;
        }
    }

    if (USART1->ISR & USART_ISR_CMF)                 // character match
    {
        USART1->ICR |= USART_ICR_CMCF;     // Character Match Clear Flag
        USART1_Line_Rdy();
    }

    if (USART1->ISR & USART_ISR_ORE)        // OverRun Error
    {
        USART1->ICR |= USART_ICR_ORECF;     // OverRun Error Clear Flag
        USART1_Error();
    }

    if (USART1->ISR & USART_ISR_FE)         // Framing Error
    {
        USART1->ICR |= USART_ICR_FECF;      // Framing Error Clear Flag
    }
}

//        if (USART1->ISR & USART_ISR_TXE)
//            cat_sendchar();


