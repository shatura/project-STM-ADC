//
// �������� ����
//


#include <main.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

const uint16_t MeasuringCountersPrescaler = 480-1;

const uint8_t Period_sample = 24;                   //���������� ���������� ����� � ������
const uint16_t T_N_Simpl = 32;                      //������ ������� ������� ��������

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

    GPIOC-> ^= GPIO_ODR_9; //led �������������
    generator(167);

    USART1_Send_String ("\r\nHonest RMS Voltage Measurement\r\n");
            USART1->CR1 |= USART_CR1_CMIE;                  // Enable CM interrupt ���������� Character match (���������� ��������)
            USART1->CR1 |= USART_CR1_RXNEIE;                // RXNE Int ON
            USART1->CR1 |= USART_CR1_TXEIE;                 // TXNE Int ON


    TIM1_Init();
    TIM15_Init();
    ADC_Init();

    NVIC->ISER[0] = (1 << ((ADC1_COMP_IRQn) & 0x1F));               // ��������� ���������� ADC1_COMP_IRQn
    NVIC->ISER[0] = (1 << ((DMA1_Channel1_IRQn) & 0x1F));           // ��������� ���������� DMA1_Channel1_IRQn



    //  Error[Li005]: no definition for "xQueueCreateCountingSemaphore"
    // �� �������� ��� #define configUSE_COUNTING_SEMAPHORES            1       �� FreeRTOSConfig.h


    xTaskCreate(vTaskUSART1Print, "ConsoleOut", 128, NULL, 0, NULL);    // ������ �����������
    xTaskCreate(vTaskSignal1PeriodMeasuring, "TIM15Start", 32, NULL, 0, NULL); // ������ ����������� TIM15 � ������ ��������� ��� �������� ������� �������
    xTaskCreate(vTaskPeriod_End, "SQRTGLS", 64, NULL, 0, NULL); // ������ ������� ���������� ������ � �������� � ��������� ������

    xTaskCreate(vTaskLed1, "LED1", 32, NULL, 0, NULL);
    xTaskCreate(vTaskLed2, "LED2", 32, NULL, 0, NULL);



    Signal1PeriodData   = xQueueCreate(1, sizeof(uint16_t));    // ������� �������� ������� �������
    USARTPeriodPrint    = xQueueCreate(2, sizeof(uint16_t));    // ������� �������� ������� ������� �� �����
    Integr_Simpl_Queue  = xQueueCreate(4, sizeof(uint32_t));    // ������� �������� ���� ��������� ������� �� 4 ������� ��� ������� ���������� ������
    USARTRMSPrint       = xQueueCreate(4, sizeof(float));       // ������� �������� RMS �� 4 ������� �� �����
    FrequencyERRORPrint = xQueueCreate(2, sizeof(uint8_t));     // ������� ������ ��������� �� ������, ����� TIM14 ����������� �������, ��� ������� vTaskTIM15_IRQ

    vSemaphoreCreateBinary(xBinarySemaphoreTIM15);              // ������������ ���������� ���������� ���������� TIM15
    if( xBinarySemaphoreTIM15 != NULL )
    {
//           xSemaphoreTake(xBinarySemaphoreTIM15, 0);
        xTaskCreate(vTaskTIM15_IRQ, "TIM15CC1IF", 64, NULL, 1, NULL); // ������ �������������� � ���������� TIM15 �������� � TIM14 ����� �������������
    }

    vSemaphoreCreateBinary(xBinarySemaphoreDMA1_Channel1);      // ������������ ���������� ���������� ���������� DMA1_Channel1
    if( xBinarySemaphoreDMA1_Channel1 != NULL )
    {
        xSemaphoreTake(xBinarySemaphoreDMA1_Channel1, 0);
        xTaskCreate(vTaskDMA1_Channel1_IRQ, "DMA_ISR_TCIF1", 64, NULL, 3, &xDMA1_Channel1_IRQ_Handle); // ������ ��������� ����������� ����� ��������� ������� �� 4 �������
    }

//    xCountingSemaphoreADCSpeedNormal = xSemaphoreCreateCounting(100000,0);     // ������� ��� ���������� ������������ ������� ������� ��� (����� ������������ �������)
//    ��������� � ��������� � vTaskTIM15_IRQ

    vTaskStartScheduler();


    while(1)
    {

    }

}

void LedsInit (void){

    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Pin 8 ��������� ��������� ���������
    GPIOC->MODER         |= GPIO_MODER_MODER8_0;       // 01 (GPIO_MODER_MODER0_0) - �����
    GPIOC->PUPDR         |= GPIO_PUPDR_PUPDR8_1;       // �������� � "-"
    GPIOC->OSPEEDR       |= GPIO_OSPEEDER_OSPEEDR8;    // 50���

    // Pin 9 ����������� ��������� ���������
    GPIOC->MODER         |= GPIO_MODER_MODER9_0;       // 01 (GPIO_MODER_MODER0_0) - �����
    GPIOC->PUPDR         |= GPIO_PUPDR_PUPDR9_1;       // �������� � "-"
    GPIOC->OSPEEDR       |= GPIO_OSPEEDER_OSPEEDR9;    // 50���
}

void vTaskLed1 (void *argument){

    while(1)
    {
        // Pin 8 ��������� ��������� ���������
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
        // Pin 9 ����������� ��������� ���������
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
//                  NVIC_DisableIRQ(TIM15_IRQn);          // ��������� ����������

        TIM15->EGR    |= TIM_EGR_UG;          // ��������� ����������
        NVIC_EnableIRQ(TIM15_IRQn);           // �������� ����������
        TIM15->CR1    |= TIM_CR1_CEN;         // ��������� ����

        osDelayUntil(&tickcount, 1000);      // ����������, ���, � FreeRTOSConfig.h

    }
}
// ������� ����������� ����������� ���������� TIM15
// ����� ��������� �������� ������� �������, �� ��������� ��� �� ���������� ������� Period_sampling
// � �������������� ���� ��������� TIM14. �� ������� ���������� �� ������������ TIM14 ���������
// ADC � DMA �� ������ �� 4 ������� �� 8 ��������� ��� ������������ ����������

void vTaskTIM15_IRQ (void *argument){

    uint16_t PeriodTime;

    while(1)
    {
        xSemaphoreTake(xBinarySemaphoreTIM15, portMAX_DELAY);

        if (uxQueueMessagesWaiting(Signal1PeriodData) != 0){

            xQueueReceive(Signal1PeriodData, &PeriodTime, 0);

            vSemaphoreDelete (xCountingSemaphoreADCSpeedNormal);
            // �� �������� �������, ��� �������� ������������� ������ (������, ������� ��������� � ��������� ��������������, ������, ���� ������� ������ ���������).
            xCountingSemaphoreADCSpeedNormal = xSemaphoreCreateCounting(100000,0);     // ������� ��� ���������� ������������ ������� ������� ��� (����� ������������ �������)

            if ((PeriodTime / Period_sampling) > 0) {
                TIM14_Init(MeasuringCountersPrescaler, PeriodTime / Period_sampling);
                // � ��������� ������ ������� ������� �� 0xffff
            }

            xQueueSend(USARTPeriodPrint, &PeriodTime, 0); // ������ �� ������

        }

    }
}

void vTaskDMA1_Channel1_IRQ (void *argument){

    // ���� ���������� ���� ���������������� ��� ����������� (static) � � ���� ������ ����� ������������ ������ ���� �����
    // ����������, � ��� ����� �������������� ��������� ����� ���������� ������������ ������
    static uint8_t Period_calculation_counter = 1;        // ������� ����� ��� �������� ����������� � ����� ���������� ������� �����

    static uint8_t Average_counter = 0;// ������� ������� �������� (8 ��������� � ������ ����� �� 4 �������)
    int32_t Average_Frame_A = 0;       // ���������� ����� ������� 8 ��� �� 4 ��������
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
        // �������� �������� �������� �������� ��������
        // �� ���������� � ���������� ������ ADC1_Simpl[]

        SemaphoreADCSpeedNormal_counter = uxSemaphoreGetCount (xCountingSemaphoreADCSpeedNormal);
        if (SemaphoreADCSpeedNormal_counter == 0) {
            // ���� �� ���� ������ �� ��������� ��� ��������� ���������� TIM14, ������������ ������, ����� ������
            asm("nop");   // ������ ��� ������������, ��� ����� ��������

/*              xQueueReceive(SignalSimplNumber, &i, 0);        // �������� ������� �� ���������� TIM14

              if (uxQueueMessagesWaiting(SignalSimplNumber) > 1){                      // ���� ������� �� �����, �� �� �� ������ ��������� �� ���������� ������� ADC

                  Calculation_Error();
              }
*/

            // � ��� 8 ������ �� ������� ������. �������� ��.
            Average_Frame_A = (Average_Frame_A >> 3) - Zero_OUT_Filtr_A;
            Average_Frame_B = (Average_Frame_B >> 3) - Zero_OUT_Filtr_B;
            Average_Frame_C = (Average_Frame_C >> 3) - Zero_OUT_Filtr_C;
            Average_Frame_D = (Average_Frame_D >> 3) - Zero_OUT_Filtr_D;
            // � �������� � ���������� ���� �������� ���� "��������" ��������� ����, ������� �� ����������� ����������� �������������.

            /* ������ ��� ����� �������� � �������
            if (Average_Frame_A < 0) Average_Frame_A = - Average_Frame_A;     // ������� �������� ���� ������������
            if (Average_Frame_B < 0) Average_Frame_B = - Average_Frame_B;
            if (Average_Frame_C < 0) Average_Frame_C = - Average_Frame_C;
            if (Average_Frame_U < 0) Average_Frame_U = - Average_Frame_U;
            */

            // ������ ����������� �� 24-� ������
            Integr_Sin_A += Average_Frame_A * Average_Frame_A;     // ��� ����� ������������������ ��������
            Integr_Sin_B += Average_Frame_B * Average_Frame_B;     // ����� ��������� B
            Integr_Sin_C += Average_Frame_C * Average_Frame_C;     // ����� ��������� C
            Integr_Sin_D += Average_Frame_D * Average_Frame_D;     // ����� ��������� D
            // ����� ����� ��������, ��� ����������� ��� 12 ��� � �� ������� � uint32_t ������ �� 24 �����

            Average_Frame_A = 0;
            Average_Frame_B = 0;
            Average_Frame_C = 0;
            Average_Frame_D = 0;


            if (++Period_calculation_counter > Period_sampling) {

                if (uxQueueMessagesWaiting(Integr_Simpl_Queue) == 0)      {

                    xQueueSend(Integr_Simpl_Queue, &Integr_Sin_A, 0); // ������� �����
                    xQueueSend(Integr_Simpl_Queue, &Integr_Sin_B, 0); // ������� �����
                    xQueueSend(Integr_Simpl_Queue, &Integr_Sin_C, 0); // ������� �����
                    xQueueSend(Integr_Simpl_Queue, &Integr_Sin_D, 0); // ������� �����
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

            xQueueSend(FrequencyERRORPrint, &FrequencyERROR, 0); // ������ �� ������
            SemaphoreADCSpeedNormal_counter++;                        // �� ��������� ������� ����� ��������� ������� �� ������
            xQueueSend(FrequencyERRORPrint, &SemaphoreADCSpeedNormal_counter, 0); // ������ �� ������

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
        // ��������� ������� ������� �������������� �� ������ ����������� � ������ vTaskDMA1_Channel1_IRQ
        // ��������� ��� ���������������� ���������� � ������� �����������, �� ���� ������� ���������� �� ��� �� �����������

        xQueueReceive(Integr_Simpl_Queue, &Integr_Sin_A, portMAX_DELAY);
        xQueueReceive(Integr_Simpl_Queue, &Integr_Sin_B, portMAX_DELAY);
        xQueueReceive(Integr_Simpl_Queue, &Integr_Sin_C, portMAX_DELAY);
        xQueueReceive(Integr_Simpl_Queue, &Integr_Sin_D, portMAX_DELAY);
        // portMAX_DELAY - ������ ������ � ����������, ����� ��� �� ��� vTaskDelay(100);
        // ��� ���� ����������� ������ �������. ������ ��� ��������� ������ � ������� (���� ������ ��. ����)
        // ���������� ����� ���������� ���� �� ������������ ���������� ���������� ����� ���� ������.
//              }

        A_RMS = sqrt_GLS(Integr_Sin_A / Period_sampling) / 4095.0 * 2.98;      // ������� ������ ������������ ��������
        B_RMS = sqrt_GLS(Integr_Sin_B / Period_sampling) / 4095.0 * 2.98;      // stm32f051x8 �� ������������ ���������� ������� � ��������� ������
        C_RMS = sqrt_GLS(Integr_Sin_C / Period_sampling) / 4095.0 * 2.98;      // ������ �������� � ����� ������ ������������ �������
        D_RMS = sqrt_GLS(Integr_Sin_D / Period_sampling) / 4095.0 * 2.98;      // ��� ��� ��� ��� ����� �� ������, �� ��� ���������. ��� ����� ������� ��������.
        // ��� ����� � ���������
        // ACS722LLCTR-05AB-T2 �� -5 �� 5 � 264 mV/A  ��� VCC 3,3 V. �� 1,65 V.
        // 4095.0*3.3/0.264; - ACS722LLCTR-05AB-T2 ������ ���� �� ������� �����.
        // � ����� STM32F0DISCOVERY ����� �� ����� ���� ����� BAT60JFILM. ������� ���������� ������� �������� ���������� 2,98 V

        if (uxQueueMessagesWaiting(USARTRMSPrint) == 0)      {
            // �������� ����������, ��� ��������� �������� �� ��� ������� ����� ������ � �������� � ��������� ������
            // �������� ������ ������ vTaskDelay(100) ����� ��������� �������� ��������
            // ���� ���������� ��� ����� � 1 �������, �� ���������� ���������� �� ����������� �������� ������� � ��
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

            xQueueSend(USARTRMSPrint, &A_RMS, 0); // �������� RMS
            xQueueSend(USARTRMSPrint, &B_RMS, 0); // �������� RMS
            xQueueSend(USARTRMSPrint, &C_RMS, 0); // �������� RMS
            xQueueSend(USARTRMSPrint, &D_RMS, 0); // �������� RMS
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
    uint8_t FrequencyDivider; // �� ������� ��� TIM14 ��������� ADC1 ����, ��� ��������� �������� ����� ������ � DMA1_Channel1 � �� ���������� � xDMA1_Channel1_IRQ_Handle
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
            // ���� ��������� xQueueReceive(USARTPeriodPrint, &data, portMAX_DELAY); �� � ���������� ���������� ��� ������
            // ������� xQueueReceive(USARTRMSPrint, &A_RMS, portMAX_DELAY); ��� ���������� vTaskDelay(1000);

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
        vTaskDelay(900);       // ��. ����.

    }

}




