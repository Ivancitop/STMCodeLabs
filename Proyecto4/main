#include "stm32f051x8.h"
#include "stdint.h"

void delay_ms(uint16_t tiempo_ms) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // Habilitar el reloj para TIM6
	// Configurar el prescaler y el auto-reload para 1 ms
	TIM6->PSC = 7; // 8 MHz / (7 + 1) = 1 MHz → 1 us por cuenta
	TIM6->ARR = 1000; // Cuenta hasta 1 ms
	TIM6->SR = 0; // Limpiar la bandera de actualización
	TIM6->CR1 = TIM_CR1_CEN; // Iniciar el temporizador
	while (tiempo_ms != 0) {
	// Esperar hasta que la bandera de actualización se active
		while ((TIM6->SR & TIM_SR_UIF) == 0) {
		// Espera a que se active UIF
		}
		TIM6->SR = 0; // Limpiar la bandera
		tiempo_ms = tiempo_ms - 1; // Decrementar tiempo
	}
	TIM6->CR1 = 0; // Detener el temporizador
}

int main(void){
	RCC->AHBENR |= (1<<17) | (1<<19); //Habilito el reloj del puerto A y C

	GPIOA->MODER &= ~(0x3<<4*2);//Limpiamos posibles configuraciones anteriores
	GPIOA->MODER |= (1<<4*2);//Configuramos el pin A4 como salida
	GPIOA->MODER &= ~(0x3<<5*2);//Limpiamos posibles configuraciones anteriores
	GPIOA->MODER |= (1<<5*2);//Configuramos el pin A5 como salida

	GPIOC->MODER &= ~(0x3<<6*2);//Configuramos el pin C6 como entrada
	GPIOC->PUPDR &= ~(0x3 << (6 * 2));//Limpiamos posibles bits en alto en el registro para el pin 6
	GPIOC->PUPDR |=  (0x2 << (6 * 2));//Habilitamos el pull-down para el bit 6

	GPIOC->MODER &= ~(0x3<<7*2);//Configuramos el pin C7 como entrada
	GPIOC->PUPDR &= ~(0x3 << (7 * 2));//Limpiamos posibles bits en alto en el registro para el pin 7
	GPIOC->PUPDR |=  (0x2 << (7 * 2));//Habilitamos el pull-down para el bit 7

	RCC->APB1ENR=(1<<4); //Habilita clock TIM6

	while(1){
		//Mover derecha
		if(GPIOC->IDR & (1<<6)){
			GPIOA->ODR |= (1<<4);
			GPIOA->ODR &= ~(1<<5);
			delay_ms(5000);
			GPIOA->ODR &= ~(1<<4);
		}
		//Mover izquierda
		else if(GPIOC->IDR & (1<<7)){
			GPIOA->ODR |= (1<<5);
			GPIOA->ODR &= ~(1<<4);
			delay_ms(3000);
			GPIOA->ODR &= ~(1<<5);
		}
	}
}
