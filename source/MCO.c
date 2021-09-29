//
// файл для тактирования с перефирией для вывода значения
//

#include "stm32f051x8.h"


void MCO_out (void) {

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;           // вкл

    GPIOA->MODER |= GPIO_MODER_MODER8_1;         // Альтернативная функция
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;    // скорость по GPIO 50 MHz
    GPIOA->AFR[1] |= 0x0 << 0;
    RCC->CFGR |= RCC_CFGR_MCO_PLL;
}