/**
 * @file timestamp.c
 * @brief Free-running 32-bit microsecond counter on TIM2.
 */

#include "timestamp.h"

void timestamp_init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* With the APB1 prescaler at 1 the timer clock equals PCLK1 (160 MHz). */
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_HCLK_DIV1) {
        timer_clock *= 2U;
    }

    TIM2->CR1 = 0U;
    TIM2->PSC = (timer_clock / 1000000U) - 1U;
    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->CNT = 0U;
    TIM2->EGR = TIM_EGR_UG; /* load the prescaler now */
    TIM2->CR1 = TIM_CR1_CEN;
}

uint32_t timestamp_us(void) {
    return TIM2->CNT;
}