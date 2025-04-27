#include "stm32f051x8.h"
#include "stdint.h"

unsigned int duty_cycle = 0;

void delay_ms(uint16_t tiempo_ms);

void PWM_CH1_Init(int duty_cycle);

int main(void){
	RCC->APB1ENR |= (1 << 4); // Habilita clock TIM6
	for (;;){ //Bucle infinito
		for (int i=0; i<=100; i+=5){
			PWM_CH1_Init(i);
			delay_ms(300);
		}
	}
}


void delay_ms(uint16_t tiempo_ms){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; //Habilitar el reloj del timer 6
	TIM6->PSC = 7; // 8 MHz / (7 + 1) = 1 MHz → 1 us por cuenta
	TIM6->ARR = 1000; // Cuenta hasta 1 ms
	TIM6->SR = 0; //Limpiar la bandera de actualización
	TIM6->CR1 = TIM_CR1_CEN; //Habilitar el temporizador
	while (tiempo_ms != 0){
		// Esperar hasta que la bandera de actualización se active
		while ((TIM6->SR & TIM_SR_UIF) == 0){
		// Espera a que se active UIF
		}
	TIM6->SR = 0; // Limpiar la bandera
	tiempo_ms = tiempo_ms - 1; // Decrementar tiempo
	}
	// Detener el temporizador
	TIM6->CR1 = 0;
}


void PWM_CH1_Init(int duty_cycle){
	RCC->AHBENR |= (1 << 19); //Habilita el reloj para el puerto C (bit 19)
	GPIOC->MODER &= ~(0b11 << 12); // Limpia bits 12 y 13
	GPIOC->MODER |= (0b10 << 12); //Configurar como funcion alterna (10)
	RCC->APB1ENR |= (1 << 1); //Habilita el reloj del timer 3
	//Configura el temporizador del timer
	TIM3->PSC = 7; // Prescaler = 7 → Ftimer = 8MHz / (7 + 1) = 1MHz
	TIM3->ARR = 1000; // Periodo de 1ms
	TIM3->CCR1 = duty_cycle * 10; // (% duty cycle)
	TIM3->CCMR1 |= (0b110 << 4); //PWM mode 1 en canal 1 + preload enable
	TIM3->CCER |= (1 << 0); //Habilita la salida de canal 1
	TIM3->CR1 |= (1 << 0); //Inicia el temporizador en el bit 0
}
