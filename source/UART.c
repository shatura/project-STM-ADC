//
// Работа с передачей данных
//

#include "stm32f051x8.h"
#include "UART.h"
#include <string.h>

void USART1_Init (void){

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;                  // вкл

    GPIOB->OTYPER &= ~GPIO_OTYPER_OT_6;                 // определяется тип вывыда
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR6;                 // подтяжка вывода
    GPIOB->MODER |= GPIO_MODER_MODER6_1;                // режим работы
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1;

    GPIOB->MODER |= GPIO_MODER_MODER7_1;                // альтернативный режим работы
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1;

    GPIOB->AFR[0]   &= ~(0xF << 24);                    // USART1_TX
    GPIOB->AFR[0]   &= ~(0xF << 28);                    // USART1_RX

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    USART1->CR2 |= 0x24<<USART_CR2_ADD_Pos;             //активное получение данных

    RCC->CFGR3 &= ~RCC_CFGR3_USART1SW;                  // USART1SW[1:0] bits Почистить оба бита
    RCC->CFGR3 |=  RCC_CFGR3_USART1SW_SYSCLK;           //System clock (SYSCLK) + USART1
    USART1->CR1 &= ~USART_CR1_M;                        // Данные - 8 бит
    USART1->CR2 &= ~USART_CR2_STOP;                     // 1 стоп-бит
    USART1->BRR =(APBCLK+BAUDRATE/2)/BAUDRATE;          // Скорость usart
    USART1->CR1 |= USART_CR1_TE;                        // USART1 ON, TX ON, RX ON
    USART1->CR1 |= USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;


    void USART1_Error (void){
    }
    void USART1_Line_Rdy (void){
    }
    void USART1_Send (char chr){

        while (!(USART1->ISR & USART_ISR_TC));
        USART1->TDR = chr;

    }
    void USART1_Send_String (char* str){

        uint8_t i = 0;

        while(str[i])
            USART1_Send (str[i++]);

    }