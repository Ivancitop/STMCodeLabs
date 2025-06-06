#include "stm32f051x8.h"
#include <stdint.h>
#include <stdio.h>

void PWM_Init(void);
void pwmHandler(uint16_t dutyCycle, uint8_t canal, uint16_t frec);
void Bluetooth_ControlLoop(void);
void USART2_Init(void);

int main(void) {
    USART2_Init();
    PWM_Init();
    Bluetooth_ControlLoop();

    while (1) {


    }
}

// USART2 en PA2 (TX) y PA3 (RX)
void USART2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~((0x3 << (2 * 2)) | (0x3 << (3 * 2)));
    GPIOA->MODER |=  (0x2 << (2 * 2)) | (0x2 << (3 * 2));
    GPIOA->AFR[0] |= (1 << (2 * 4)) | (1 << (3 * 4)); // AF1 para USART2

    USART2->BRR = 833; // 8 MHz / 9600 baud
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void PWM_Init(void) {//Configuración inicial del pwm
    // Habilita reloj para TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configura PA1 como TIM2_CH2
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~(0x3 << (1 * 2));
    GPIOA->MODER |= (0x2 << (1 * 2)); // Alternate function
    GPIOA->AFR[0] &= ~(0xF << (1 * 4));
    GPIOA->AFR[0] |= (0x2 << (1 * 4)); // AF2 para TIM2_CH2 en STM32F051

    // Configura TIM2
    TIM2->PSC = 7;
    TIM2->ARR = 999;
    TIM2->CCR2 = 0;
    TIM2->CCMR1 = (6 << 12) | (1 << 11); // PWM mode 1 + preload (CH2)
    TIM2->CCER |= (1 << 4); // Enable CH2
    TIM2->BDTR |= TIM_BDTR_MOE;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    // TIM3_CH1 en PC6
        RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

        GPIOC->MODER &= ~(0x3 << (6 * 2));
        GPIOC->MODER |= (0x2 << (6 * 2)); // AF
        GPIOC->AFR[0] &= ~(0xF << (6 * 4));
        GPIOC->AFR[0] |= (0x0 << (6 * 4)); // AF0


        TIM3->CCMR1 = (6 << 4) | (1 << 3); // PWM mode 1, preload enable
        TIM3->CCER |= (1 << 0); // Enable CH1
        TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

        // TIM14_CH1 en PA7
        RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
        RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

        GPIOA->MODER &= ~(0x3 << (7 * 2));
        GPIOA->MODER |= (0x2 << (7 * 2));
        GPIOA->AFR[0] &= ~(0xF << (7 * 4));
        GPIOA->AFR[0] |= (0x4 << (7 * 4)); // AF4


        TIM14->CCMR1 = (6 << 4) | (1 << 3);
        TIM14->CCER |= (1 << 0);
        TIM14->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void pwmHandler(uint16_t dutyCycle, uint8_t canal, uint16_t freq) {//Función para controlar la frecuencia y dutyCycle del pwm
	 // Validación de parámetros
	if (dutyCycle > 100) dutyCycle = 100;

	    uint32_t clk = 8000000;
	    uint32_t arr = 999;
	    uint32_t psc = (clk / ((arr + 1) * freq));
	    uint32_t pulse = (arr + 1) * dutyCycle / 100;
    switch (canal) {
        case 1: // TIM3_CH1 - PC6
            TIM3->PSC = psc;
            TIM3->ARR = arr;
            TIM3->CCR1 = pulse;
            break;
        case 2: // TIM2_CH2 - PA1
            TIM2->PSC = psc;
            TIM2->ARR = arr;
            TIM2->CCR2 = pulse;
            break;
        case 3: // TIM14_CH1 - PA7
            TIM14->PSC = psc;
            TIM14->ARR = arr;
            TIM14->CCR1 = pulse;
            break;
        default:
            break;
    }
}



char USART2_Read(void) {//Lectura de datos en el puerto USART
    while (!(USART2->ISR & USART_ISR_RXNE));
    char c = USART2->RDR;
    return c;
}


void Bluetooth_ControlLoop(void) {
    char buffer[50];
    int idx = 0;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~(0x3 << (8 * 2)); // Limpia los bits 17:16
    GPIOC->MODER |=  (0x1 << (8 * 2)); // Modo salida
    while (1) {

        char c = USART2_Read();
        GPIOC->ODR |= (1 << 8); // LED ON para cada char

        if (c == '\n' || c == '\r') {
            buffer[idx] = '\0';

            int duty = 0, frec = 0, canal = 0;
            if (sscanf(buffer, "%d %d %d", &duty, &frec, &canal) == 3) {
                pwmHandler(duty, canal, frec);
            }

            idx = 0;
            GPIOC->ODR &= ~(1 << 8); // LED OFF al procesar
        } else if (idx < sizeof(buffer) - 1) {
            buffer[idx++] = c;
        }
    }
}

