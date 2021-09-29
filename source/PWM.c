//
// ������ � ���
//

#include "stm32f051x8.h"


void TIM1_init(void);

//______________________________________________________________________________
static uint16_t TIM1_prescalerRegister = 480 - 1;
static uint16_t TIM1_autoReloadRegister = 300 - 1;      // ������� � ��� �������, ������� �� 1000

static uint16_t TIM1_FirstComparatorRegister = 25;
static uint16_t TIM1_SecondComparatorRegister = 50;
static uint16_t TIM1_ThirdComparatorRegister = 75;

const uint16_t Deadtime = 0;                            // ������� ����� ��������������� �������� TIM ( = 12)

void TIM1_Init(void)
{


//�������� ������������ �����. ����� I/O �� ���� AHBENR
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;                 // �������� TIM1 �� ���� APB2ENR


    GPIOA->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR8;         // 50���
    GPIOA->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR9;         // 50���
    GPIOA->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR10;        // 50���
    GPIOB->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR13;        // 50���
    GPIOB->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR14;        // 50���
    GPIOB->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR15;        // 50���

// ��������� TIM_CH1 -> PA8, TIM_CH2 -> PA9, TIM_CH3 -> PA10
    GPIOA->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
// ��������� TIM_CH1N -> PB13, TIM_CH2N -> PB14, TIM_CH3N -> PB15
    GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
// MODER8_1 "10" - �������������� �������
    GPIOA->AFR[1] |= (0x02 << 0) | (0x02 << 4) | (0x02 << 8);
//GPIOB->AFR[1] |= (0x02 << 20) | (0x02 << 24) | (0x02 << 28);
    GPIOB->AFR[1] |= 0x22200000;
//GPIOA->AFR[0] |= (0x02 << 28);
//GPIOB->AFR[1] |= (0x02 << 0) | (0x02 << 4);

// PB12 - pin ��������� ���������
    GPIOB->PUPDR    |= (GPIO_PUPDR_PUPDR12_1);          // �������� � "-"
    GPIOB->OSPEEDR  |=  GPIO_OSPEEDER_OSPEEDR12;        // 50���
    GPIOB->MODER    |= GPIO_MODER_MODER12_1;            //Alternate function TIM1_BKIN
    GPIOB->AFR[1]   |= (0x02 << 16);


// ������������ ������� TIM1
// ��������� �������. 3-������ ��� � ������������� �� ������

// ��������� ���������������� ��������
    TIM1->PSC = TIM1_prescalerRegister;                 //F=fCK_PSC/(PSC[0:15]+1)
// � SM32 ���������� RC ��������� � �������� 48 ���.
// ��������� ��������������� �������� ������ 32.
// ������� ������� 2 ���� (������������ �����).
// �������� �������� ����� ������������������ � �������� 41 ���:
// ����������� ������� ����� 1 ��� ������� �������� ���������������� ��������
// ��� ��������� ������������ ������� N, ���������� ���������� � N-1.

// ��������� ������� �����
    TIM1->ARR = TIM1_autoReloadRegister;
// TIMx_ARR � ������� �������������� ������������,
// ������� ������� �� 0 �� TIMx_ARR, ��� �������� � �����������
// �� ����������� �����, ������� ��� ��������, �� �������� ������� ���.

// ��������������� ��������� ����������
    TIM1->CCR1 = TIM1_FirstComparatorRegister;
    TIM1->CCR2 = TIM1_SecondComparatorRegister;
    TIM1->CCR3 = TIM1_ThirdComparatorRegister;
// TIMx_CCRy[x � ����� �������, y � ����� ������] � ���������� �����������
// ���������� ���. �� ����, ���� � ARR �� ������� 1000, � � CCRy 300,
// �� ����������� ���������� ��� ������������� �������� ������
// � ������ ��� ����� ����� 0.3 ��� 30%.

// �������� ����� ����� � ����� ���
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;     // ��� ����� 1 (OC1M = 110) | TIM_CCMR1_OC1M_0
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;     // ��� ����� 1 (OC1M = 110) | TIM_CCMR1_OC2M_0
    TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;     // ��� ����� 1 (OC1M = 110) | TIM_CCMR2_OC3M_0
// ��� ������ ���. ���� �������� "111" ����� ���������

    TIM1->CR1 |= TIM_CR1_ARPE;                              //������� ����� ��������������� ������ �������� ����������������

    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;                         // �������� ������� ��������������� �������� ����������� ������ 1
    TIM1->CCMR1 |= TIM_CCMR1_OC2PE;                         // �������� ������� ��������������� �������� ����������� ������ 2
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;                         // �������� ������� ��������������� �������� ����������� ������ 3

//TIM1->CR1 |= TIM_CR1_DIR; // ���� ����������, ����� ������� ����
    TIM1->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CMS_1; // ����� 3 ������������ �� ������
// (Center-aligned mode 3). ������� ������� ����� � ���� ����������.
// ����� ���������� ��������� ������ �������, ����������� �� �����
// (CCxS=00 � �������� TIMx_CCMRx) ��������������� ������ �����,
// ����� ������� ������� ����� ��� ����

// Select active high polarity on OC1 (CC1P = 0, reset value), enable the output on OC1 (CC1E = 1)
// �������� �������� ������� ���������� � OC1 (CC1P = 0, �������� ������), �������� ����� �� OC1 (CC1E = 1)
// CC1P & CC1NP - ����� ���������� 0: �������� ������� ������� - "1". 1: �������� ������� ������ - "0".
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;
    TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE;
    TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NE;

    TIM1->BDTR |= Deadtime;                                 // ������� �����. ��������� ���������� �� �������� ����������� ������.
    TIM1->BDTR |= TIM_BDTR_BKE | TIM_BDTR_BKP;              // break input, active polarity = high


    TIM1->BDTR |= TIM_BDTR_MOE;                             // ��������� ����� ������� �� ������

    TIM1->EGR |= TIM_EGR_UG;                                // ��������� ����������

    TIM1->DIER |= TIM_DIER_UIE;                             // ��������� ���������� �� ������������

    TIM1->CR1 |= TIM_CR1_CEN;                               // ��������� ����

    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    NVIC->ISER[0] = (1 << ((TIM1_BRK_UP_TRG_COM_IRQn) & 0x1F)); // ��������� TIM1_BRK_UP_TRG_COM_IRQn, ���� ��� � 13  & 0x1F
// NVIC->ISER[1] |= 0x00000002; // ��������� IRQ33, ���� �� � ��� ���� ������ 30 ��������. ))))

//NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1);
//NVIC->IP[3] |= 0x00004000; // ��������� TIM1_BRK_UP_TRG_COM_IRQn - ����
//NVIC->IP[M] M = 13 DIV 4 = 3
//N = 13 MOD 4 = 1. ����� �-5 ���������� ������������.
//�������� ����� 6-7 ��� 4 ������ ���������� 0x0000�000 - ��������� ���.
//i = NVIC_GetPriority(TIM1_BRK_UP_TRG_COM_IRQn);
//NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1);
