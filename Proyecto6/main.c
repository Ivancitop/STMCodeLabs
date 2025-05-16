/*
*@Author: Iván Delgado Ramos
*@Date: 14/05/25
*@Brief: Control of 12v motors via Bluetooth with USART communication 
*/

#include "stm32f051x8.h"
#include <stdint.h>
#include <stdio.h>

// Prototipos, para definir las funciones después del main
void USART2_Init(void);
char USART2_Read(void);
void Bluetooth_ControlLoop(void);
void PWM_Init(void);
void pwmHandler(uint16_t dutyCycle, uint16_t canal);

int main(void)
{
    USART2_Init();
    PWM_Init();
    Bluetooth_ControlLoop(); // Contiene su propio while(1)
}

// USART2 en PA2 (TX) y PA3 (RX)
void USART2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; //Habilitar reloj de USART2
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN; //Habilitar reloj del puerto A

    GPIOA->MODER &= ~((0x3 << (2 * 2)) | (0x3 << (3 * 2))); //Limpiar  los bits correspondientes a PA2 y PA3
    GPIOA->MODER |=  (0x2 << (2 * 2)) | (0x2 << (3 * 2)); //Establecer PA2 y PA3 como función alternativa (10)
    GPIOA->AFR[0] |= (1 << (4 * 2)) | (1 << (4 * 3)); // AF1 para USART2 (0001)

    USART2->BRR = 833; // 8 MHz / 9600 baud
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; //Habilita el transmisor, receptor y habilita el USART
}

// PWM en PC6–PC9 (CH1–CH4) - TIM3
void PWM_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Habilita el reloj del timer 3
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN; //Habilita el reloj del puerto C

    // Configura PC6–PC9 como AF (AF0)
    GPIOC->MODER &= ~((0x3 << (6 * 2)) | (0x3 << (7 * 2)) | (0x3 << (8 * 2)) | (0x3 << (9 * 2))); //Limpia bits
    GPIOC->MODER |=  (0x2 << (6 * 2)) | (0x2 << (7 * 2)) | (0x2 << (8 * 2)) | (0x2 << (9 * 2)); //Establece "10" en los pines

    // AF0 para PC6–PC9
    GPIOC->AFR[0] &= ~((0xF << (6 * 4)) | (0xF << (7 * 4))); // Limpia los 4 bits de PC6 y PC7, establece "0000"
    GPIOC->AFR[1] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4))); // PC8 y PC9 están en AFR[1]

    TIM3->PSC = 7;      // Prescaler: (8 MHz / (7+1)) = 1 MHz
    TIM3->ARR = 999;    // Periodo: 1 kHz

    // PWM mode 1 con preload habilitado
    TIM3->CCMR1 &= ~((0x7 << 4) | (0x7 << 12)); // CH1, CH2
    TIM3->CCMR2 &= ~((0x7 << 4) | (0x7 << 12)); // CH3, CH4

    TIM3->CCMR1 |=  (0x6 << 4) | (1 << 3);     // CH1: Output Compare 1 Mode (PWM Mode 1, "110") y Output compare 1 preload enable
    TIM3->CCMR1 |=  (0x6 << 12) | (1 << 11);   // CH2: Output Compare 2 Mode (PWM Mode 1, "110") y Output compare 2 preload enable
    TIM3->CCMR2 |=  (0x6 << 4) | (1 << 3);     // CH3: Output compare 3 mode (PWM) y Output compare 3 preload enable
    TIM3->CCMR2 |=  (0x6 << 12) | (1 << 11);   // CH4: Output compare 4 mode (PWM) y Output compare 4 preload enable

    // Activa todos los canales
    TIM3->CCER |= (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12); //Habilita Capture/Compare 1,2,3,4
    TIM3->CR1  |= TIM_CR1_CEN; //Habilita al counter
}

// Lee un carácter desde USART2
char USART2_Read(void)
{
    while (!(USART2->ISR & USART_ISR_RXNE)); //Read data register not empty
    return USART2->RDR; //Receive data value
}

// Controla el duty cycle de los 4 canales
void pwmHandler(uint16_t dutyCycle, uint16_t canal)
{
    if (dutyCycle > 100) dutyCycle = 100; //Si el dutyCycle es mayor a 100, reestablecer a 100
    uint16_t value = dutyCycle * 10; //Establece el valor del PWM

    switch (canal) { //Dependiendo del canal, manda el valor de ese PWM al canal correspondiente
        case 1: TIM3->CCR1 = value; break;
        case 2: TIM3->CCR2 = value; break;
        case 3: TIM3->CCR3 = value; break;
        case 4: TIM3->CCR4 = value; break;
    }
}

// Bucle de control por Bluetooth
void Bluetooth_ControlLoop(void)
{
    char buffer[10]; //Definir lista de 10 caractéres
    int idx = 0; //Indice

    while (1)
    {
        char c = USART2_Read(); //

        if (c == '\n' || c == '\r') { //Si encuentra un salto de línea o espacio...
            buffer[idx] = '\0'; //Cerrar lista

            int duty = 0, canal = 0; //Definir variables
            if (sscanf(buffer, "%d %d", &duty, &canal) == 2) { //Si lo que lee es válido (valor del PWM + canal = 2 valores)...
                if (canal >= 1 && canal <= 4 && duty >= 0 && duty <= 100) { //Si el canal es de 0 a 4, y el ciclo de 0 a 100...
                    pwmHandler(duty, canal); //Enviar valores a la función de asignación de PWM "pwmHandler"
                }
            }
            idx = 0; //Reestablecer índice = 0
        } else if (idx < sizeof(buffer) - 1) { //Si el indice es menor al tamaño del lista
            buffer[idx++] = c; //Guardar el mensaje en la variable "c"
        }
    }
}
