//
// Файл настройки генерации данных
//


#include "stm32f051x8.h"
#include "Sin_DAC_3V.h"

#define APBCLK = 48000000UL

void DAC_Init(void);
void DMA1_Init(void);
void TIM6_Init(void);

static uint16_t TIM6_prescalerRegister;
static uint16_t TIM6_autoReloadRegister;

void generator(uint32_t frequency) {

        if (frequency <= 50)
        {
            TIM6_prescalerRegister = 60-1;
            TIM6_autoReloadRegister = APBCLK / 60 / frequency / 24;
        }

        if ((frequency >50) && (frequency<=100))
        {
            TIM6_prescalerRegister = 2-1;
            TIM6_autoReloadRegister = APBCLK / 2 / frequency / 24;
        }

        if (frequency > 100)
        {
            TIM6_prescalerRegister = 0;
            TIM6_autoReloadRegister = APBCLK / frequency / 24;
        }

   DAC_Init();
   DMA1_Init();
   TIM6_Init();
   NVIC_EnableIRQ (TIM6_DAC_IRQn);

}


void DAC_Init(void)
{
    GPIOA->MODER |= GPIO_MODER_MODER4;          //определение режима работы - аналоговый режим
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;          //включение тактирования ЦАП
    DAC->CR |= DAC_CR_EN1;                      //активация ЦАП
    DAC->CR &= ~DAC_CR_BOFF1;                   //выключение выходного буфера

    DAC->CR |= DAC_CR_TEN1;                     //активация запуска от события
    DAC->CR &= ~DAC_CR_TSEL1;                   //включениие пробразование при команде
    DAC->CR |= DAC_CR_DMAEN1;                   //включение ДМА
    DAC->CR |= DAC_CR_DMAUDRIE1;                //разрешение генерации прерывания в случае ошибок с ДМА

}

void DMA1_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN            //включение ДМА

    DMA1_Channel3->CRR |= DMA_CRR_MINC;         //инкремент адреса памяти
    DMA1_Channel3->CRR |= DMA_CRR_MSIZE_0;      //передача данных по 16 бит
    DMA1_Channel3->CRR |= DMA_CRR_PSIZE_0;      //память перефирии по 16 бит
    DMA1_Channel3->CRR |= DMA_CRR_CIRC;         //запуск кольцевого режима
    DMA1_Channel3->CRR |= DMA_CRR_DIR;          //направление данных по ДМА

    DMA1_Channel3->CNDTR = 24;                  //деление на 24 точки
    DMA1_Channel3->CPAR = (uint32_t) &DAC->DHR12R1; //выравнивание
    DMA1_Channel3->CMAR = (uint32_t) Sin_DAC;   //вид отображения

    DMA1_Channel3->CRR = DMA |= DMA_CRR_EN;     //вкл дма

}

void TIM6_init)(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;         //разрешить тактирование таймера
    TIM6->PSC = TIM6_prescalerRegister;         //предделитель
    TIM6->ARR = TIM6_autoReloadRegister;        //переполнение две секунды
    TIM6->CR1 |= TIM_CR1_ARPE;                  //Включен режим предварительной записи регистра автоперезагрузки
    TIM6->DIER |= TIM_DIER_UDE;                 //разрешение ДМА
    TIM6->CR2 |= TIM_CR2_MMS_1;                 //событие обновления используется как выход триггера
    TIM6->CR1 |= TIM_CR1_CEN;                   //запуск счета
}

void DAC_ConversionSpeedError (void)
{
    DAC->CR &= ~DAC_CR_EN1;                     //активация ЦАП
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;          //активация канала
    TIM6->CR1 &= ~TIM_CR1_CEN;                  //активация счетчика
    NVIC_DisableIRQ(TIM6_DAC_IRQn);             //прерывания
    GPIOC->BSRR |= GPIO_BSRR_BS_8;              //работа с регистром чтения записи
}
}


