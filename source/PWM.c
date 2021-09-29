//
// Работа с ШИМ
//

#include "stm32f051x8.h"


void TIM1_init(void);

//______________________________________________________________________________
static uint16_t TIM1_prescalerRegister = 480 - 1;
static uint16_t TIM1_autoReloadRegister = 300 - 1;      // Считает в обе стороны, поэтому не 1000

static uint16_t TIM1_FirstComparatorRegister = 25;
static uint16_t TIM1_SecondComparatorRegister = 50;
static uint16_t TIM1_ThirdComparatorRegister = 75;

const uint16_t Deadtime = 0;                            // Мертвое время комплементарных сигналов TIM ( = 12)

void TIM1_Init(void)
{


//Включить тактирование порта. Порты I/O по шине AHBENR
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;                 // Включить TIM1 по шине APB2ENR


    GPIOA->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR8;         // 50МГц
    GPIOA->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR9;         // 50МГц
    GPIOA->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR10;        // 50МГц
    GPIOB->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR13;        // 50МГц
    GPIOB->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR14;        // 50МГц
    GPIOB->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR15;        // 50МГц

// Настроить TIM_CH1 -> PA8, TIM_CH2 -> PA9, TIM_CH3 -> PA10
    GPIOA->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
// Настроить TIM_CH1N -> PB13, TIM_CH2N -> PB14, TIM_CH3N -> PB15
    GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
// MODER8_1 "10" - альтернативная функция
    GPIOA->AFR[1] |= (0x02 << 0) | (0x02 << 4) | (0x02 << 8);
//GPIOB->AFR[1] |= (0x02 << 20) | (0x02 << 24) | (0x02 << 28);
    GPIOB->AFR[1] |= 0x22200000;
//GPIOA->AFR[0] |= (0x02 << 28);
//GPIOB->AFR[1] |= (0x02 << 0) | (0x02 << 4);

// PB12 - pin аварийной остановки
    GPIOB->PUPDR    |= (GPIO_PUPDR_PUPDR12_1);          // Подтяжка к "-"
    GPIOB->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR12;        // 50МГц
    GPIOB->MODER    |= GPIO_MODER_MODER12_1;            //Alternate function TIM1_BKIN
    GPIOB->AFR[1]   |= (0x02 << 16);


// конфигурация таймера TIM1
// Генерация несущей. 3-фазный ШИМ с выравниванием по центру

// Установка предварительного делителя
    TIM1->PSC = TIM1_prescalerRegister;                 //F=fCK_PSC/(PSC[0:15]+1)
// В SM32 внутренний RC генератор с частотой 48 МГц.
// Установим предварительный делитель равным 32.
// Счетчик считает 2 раза (симметричный режим).
// Значения счетчика будут инкрементироваться с частотой 41 Кгц:
// Коэффициент деления равен 1 при нулевом значении предварительного делителя
// Для получения коэффициента деления N, необходимо установить в N-1.

// Установка границы счета
    TIM1->ARR = TIM1_autoReloadRegister;
// TIMx_ARR – регистр автоматической перезагрузки,
// счётчик считает от 0 до TIMx_ARR, или наоборот в зависимости
// от направления счёта, изменяя это значение, мы изменяем частоту ШИМ.

// Предварительная установка скважности
    TIM1->CCR1 = TIM1_FirstComparatorRegister;
    TIM1->CCR2 = TIM1_SecondComparatorRegister;
    TIM1->CCR3 = TIM1_ThirdComparatorRegister;
// TIMx_CCRy[x – номер таймера, y – номер канала] – определяет коэффициент
// заполнения ШИМ. То есть, если в ARR мы запишем 1000, а в CCRy 300,
// то коэффициент заполнения при положительном активном уровне
// и прямом ШИМ будет равен 0.3 или 30%.

// Включаем режим канал в режим ШИМ
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;     // ШИМ режим 1 (OC1M = 110) | TIM_CCMR1_OC1M_0
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;     // ШИМ режим 1 (OC1M = 110) | TIM_CCMR1_OC2M_0
    TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;     // ШИМ режим 1 (OC1M = 110) | TIM_CCMR2_OC3M_0
// Это прямой ШИМ. Если записать "111" будет инверсный

    TIM1->CR1 |= TIM_CR1_ARPE;                              //Включен режим предварительной записи регистра автоперезагрузки

    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;                         // Включаем регистр предварительной загрузки компаратора канала 1
    TIM1->CCMR1 |= TIM_CCMR1_OC2PE;                         // Включаем регистр предварительной загрузки компаратора канала 2
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;                         // Включаем регистр предварительной загрузки компаратора канала 3

//TIM1->CR1 |= TIM_CR1_DIR; // Если установить, будет считать вниз
    TIM1->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CMS_1; // Режим 3 выравнивания по центру
// (Center-aligned mode 3). Счетчик считает вверх и вниз поочередно.
// Флаги прерывания сравнения выхода каналов, настроенных на выход
// (CCxS=00 в регистре TIMx_CCMRx) устанавливаются только тогда,
// когда счетчик считает вверх или вниз

// Select active high polarity on OC1 (CC1P = 0, reset value), enable the output on OC1 (CC1E = 1)
// Выберите активную высокую полярность в OC1 (CC1P = 0, значение сброса), включите выход на OC1 (CC1E = 1)
// CC1P & CC1NP - выбор полярности 0: активный уровень высокий - "1". 1: активный уровень низкий - "0".
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;
    TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE;
    TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NE;

    TIM1->BDTR |= Deadtime;                                 // Мертвое время. Константа рассчитана из задержек конкретного железа.
    TIM1->BDTR |= TIM_BDTR_BKE | TIM_BDTR_BKP;              // break input, active polarity = high


    TIM1->BDTR |= TIM_BDTR_MOE;                             // Разрешаем вывод сигнала на выводы

    TIM1->EGR |= TIM_EGR_UG;                                // генерация обновления

    TIM1->DIER |= TIM_DIER_UIE;                             // Разрешить прерывание по переполнению

    TIM1->CR1 |= TIM_CR1_CEN;                               // Запускаем счет

    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    NVIC->ISER[0] = (1 << ((TIM1_BRK_UP_TRG_COM_IRQn) & 0x1F)); // Разрешить TIM1_BRK_UP_TRG_COM_IRQn, если что № 13  & 0x1F
// NVIC->ISER[1] |= 0x00000002; // Разрешить IRQ33, если бы у нас было больше 30 векторов. ))))

//NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1);
//NVIC->IP[3] |= 0x00004000; // Приоритет TIM1_BRK_UP_TRG_COM_IRQn - ОДИН
//NVIC->IP[M] M = 13 DIV 4 = 3
//N = 13 MOD 4 = 1. Байты о-5 обнуляются контроллером.
//Остаются байты 6-7 под 4 уровня приоритета 0x0000С000 - приоритет ТРИ.
//i = NVIC_GetPriority(TIM1_BRK_UP_TRG_COM_IRQn);
//NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1);
