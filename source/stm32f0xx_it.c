//
// ���� �� ��������� ����� � ���, ������
//

#include "stm32f051x8.h"

#include "FreeRTOS.h"
#include "semphr.h"


extern const uint8_t T_N_Simpl;                       // ����� ������� ������� ��������
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
void  TIM1_BRK_UP_TRG_COM_IRQHandler (void) // ���������� ��� ����, ����������, ����� � ���������� ������� TIM1
{

    if(TIM1->SR & TIM_SR_UIF)   {

        TIM1->SR &= ~TIM_SR_UIF;     // ���������� ���� ���������� �� ������������


//    TIM1->CCR1 = *p_Sin_A;
//    TIM1->CCR2 = *p_Sin_B;
//    TIM1->CCR3 = *p_Sin_C;
//TIM1->CR1 |= TIM_CR1_UDIS;      // ��������� ����������
//TIM1->DIER &= ~TIM_DIER_UIE;    // ��������� ���������� �� ������������
// ��������� �� ��� ����� � TIM2 ����� �������� ��������� �������� ���
    }
/*
//-----
  if(TIM1->SR & TIM_SR_BIF)   {
    TIM1->SR &= ~TIM_SR_BIF;     // ���������� ���� ����������
// ���������� ���� ���������� ��������, ������ �� �� ���������, ���� PB12 != 0.
                              }
*/
}
//______________________________________________________________________________
void  TIM6_DAC_IRQHandler (void)    // ���������� �� TIM6 & DAC
{

    if(DAC->SR & DAC_SR_DMAUDR1)   {    // ��� ���� ����������� DMA. ����� DAC �� ����� ��������� ��������������, � ��������� ������ ��� ������
        DAC->CR &= ~DAC_CR_DMAEN1;  // ��������� DMA, �������� ������� � �� ����� ��� ����������������
        DAC->SR |= DAC_SR_DMAUDR1;  // DAC channel1 DMA underrun flag � ������� ������� �������� ������� �1�
        // �� �� ������� �����. �� �������� �� ���������� DMA
        DAC_ConversionSpeedError ();
    }
}

//______________________________________________________________________________
void  TIM14_IRQHandler (void)    // ���������� �� TIM6 & DAC
{
    uint16_t FrequencyFlag = 0;

    if(TIM14->SR & TIM_SR_UIF)   {
        TIM14->SR &= ~TIM_SR_UIF;     // ���������� ���� ����������

//        xQueueSendToBackFromISR(SignalSimplNumber, &FrequencyERROR, NULL);
        xSemaphoreGiveFromISR(xCountingSemaphoreADCSpeedNormal, 0);
        FrequencyFlag = uxSemaphoreGetCount(xCountingSemaphoreADCSpeedNormal);
        // TIM14->SR & TIM_SR_UIF ����� ����� ������ � �������� ������������ ��� TIM_EGR_UG ��� �������������
        // ��� �����, ����� �� �������� ��� ������ ���� ��� ���� ������ �� ����� ����������
        // ����� ��������� ������ ������� ���������� � ��������� ��������� � ����� ���������� �� 1 �����
        if (FrequencyFlag == 1) {

            //!!!!!
            ADC1->CR |= ADC_CR_ADSTP;               // ADC stop of conversion command
            while (ADC1->CR & ADC_CR_ADSTART);      // ��������� ���� ADSTART ����� ����� ����
            // ������! �� ��������� � ����� ������ ���� ���� ��������
            ADC1->ISR |= ADC_ISR_EOSEQ;             // �������� ���� ��������� ������������������. �������� ������������ ������� 1
            ADC1->IER |= ADC_IER_EOSIE;             // ��������� ���������� �� ����� ������������������ ������� ���
            ADC1->CR |= ADC_CR_ADSTART;
        }
    }
}

//______________________________________________________________________________
//RM0091 pg. 951 A.9.4 Input capture data management code example
void  TIM15_IRQHandler (void)    // ���������� �� �������
{
    static portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    static uint8_t gap = 0;
    static uint32_t counter = 0;
    static uint32_t counter0 = 0;
    static uint32_t counter1 = 0;

// ����������. ���� ��� ��������� ������ ������������� ������ ��������.
// ����� ��������� ���������� ������������ ���������, ���������� �������� ���������� ���������� (UIE = 1) � ��������� ��������� ��.
    if(TIM15->SR & TIM_SR_CC1IF)
    {

        if(TIM15->SR & TIM_SR_CC1OF)            // Check the overflow
        {
            // Overflow error management      ��� ����� �������� ��, ��� ��������� �� ��������� ����������, ����� TIM15->CCR1 �� ����������
            gap = 0; // Reinitialize the laps computing                                       ��������� ������������� ����� ����������
            TIM15->SR &= ~(TIM_SR_CC1OF | TIM_SR_CC1IF); // Clear the flags
            return;
        }

        if (gap == 0) // Test if it is the first rising edge                                          ���������, �������� �� ��� ������ ����������� �������
        {
            counter0 = TIM15->CCR1; // Read the capture counter which clears the CC1ICF               �������� ������� �������, ������� ������� CC1ICF
            gap = 1; // Indicate that the first rising edge has yet been detected                     �������, ��� ������ ����������� ����� ��� �� ���������
        }
        else
        {
            counter1 = TIM15->CCR1; // Read the capture counter which clears the CC1ICF               �������� ������� �������, ������� ������� CC1ICF
            if (counter1 > counter0) // ��������� ������������ �������� �������
            {
                counter = counter1 - counter0;
            }
            else
            {
                counter = counter1 + 0xFFFF - counter0 + 1;
            }
            counter0 = counter1;

            TIM15->CR1    &= ~TIM_CR1_CEN;
            NVIC_DisableIRQ(TIM15_IRQn);          // ��������� ����������

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
// Unexpected Interrupt �������������� ����������
// Manage an error for robust application ���������� ������� ��� ��������� ����������
    }

}

//______________________________________________________________________________
void  ADC1_COMP_IRQHandler (void)    // ���������� ���
{
/*
  if(ADC1->ISR & ADC_ISR_AWD)   {       // ���������� �� ����������� �������

  TIM1->EGR |= TIM_EGR_BG;                // ���������� ���� ���������� ���������� �������� TIM1

  ADC1->ISR |= ADC_ISR_AWD;             // �������� ������������ ������� 1
                                }
*/
//-----
    if(ADC1->ISR & ADC_ISR_EOSEQ)   {       // ���������� �� ����� ������������������ �������������� �������
        ADC1->ISR |= ADC_ISR_EOSEQ;           // �������� ������������ ������� 1
//-----
        ADC1->CR |= ADC_CR_ADSTP;               // ADC stop of conversion command
        while (ADC1->CR & ADC_CR_ADSTART);      // ��������� ���� ADSTART ����� ����� ����
// ������! �� ��������� � ����� ������ ���� ���� ��������
        ADC1->IER &= ~ADC_IER_EOSIE;            // ��������� ���������� �� ����� ������������������ ������� ���
        ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;      // �������� �������� DMA �� ��� � �������� ������
        ADC1->CR |= ADC_CR_ADSTART;             // ADC start of conversion

        DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR));         //  ��������� ����� �������� ������������ ������
        DMA1_Channel1->CMAR = (uint32_t)(&ADC1_Simpl[0]);       //  ��������� ����� ������
        DMA1_Channel1->CNDTR = T_N_Simpl;       //  ��������� ���������� ���� DMA-��������, ������� ������ ����������� �� ������ DMA 1
        DMA1_Channel1->CCR |= DMA_CCR_EN;                       // �������� ����� DMA 1

        DMA1->IFCR = DMA_IFCR_CGIF1;           // 1: ������� ����� GIF, TEIF, HTIF � TCIF � �������� DMA_ISR
        DMA1_Channel1->CCR |= DMA_CCR_TCIE;                     //  ���������� ���������� � ������ ���������
    }
}

//______________________________________________________________________________
void  DMA1_Channel1_IRQHandler (void)    // ���������� DMA ADC1
{
//  static portBASE_TYPE xHigherPriorityTaskWoken;
    static portBASE_TYPE PoFig;
    // ���������� ���������� ����� ���������, �� �� ����� ��������� ��������
    // ������������ ����� - ��� ��������. �� ����� ����� RTOS �������� ������������ ������, ������� ����� ��������������.
    PoFig = pdFALSE;

    if (DMA1->ISR & DMA_ISR_TCIF1)   {    // Channel 1 Transfer Complete flag
        DMA1_Channel1->CCR &= ~DMA_CCR_TCIE;    //  ��������� ���������� � ������ ���������
        DMA1->IFCR = DMA_IFCR_CGIF1;            // 1: ������� ����� GIF, TEIF, HTIF � TCIF � �������� DMA_ISR
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;      // ��������� ����� DMA1
        ADC1->CR |= ADC_CR_ADSTP;               // ADC stop of conversion command
        ADC1->CFGR1 &= ~(ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG);      // ��������� �������� DMA �� ��� � �������� ������
        //ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;        // ���������� ������ ������.
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

    if (USART1->ISR & USART_ISR_RXNE)               // ���� ������������ ��� �� ������ �������� ������
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


