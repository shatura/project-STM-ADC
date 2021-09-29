//
// Внешняя частота взаимодействия устройств
//


void SystemInit (void)
{

    //вкл
    RCC->CR |= RCC_CR_HSION;

    //ожидание стабильности
    while (!(RCC->CR & RCC_CR_HSIRDY));

    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    RCC->CFGR  |= RCC_CFGR_PLLSRC_HSI_DIV2;
    RCC->CFGR  |= RCC_CFGR_PLLMUL12;

    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    //установка автоподстройки
    RCC->CFGR |= RCC_CFGR_SW_PLL | RCC_CFGR_SWS_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {};

}