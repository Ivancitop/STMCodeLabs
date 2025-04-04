/*
 ******************************************************************************
 * @Program        : Práctica 3, Pull Down y Pull Up
 * @Author         : Iván Delgado Ramos
 * @Date	   : 04/04/25
 * @brief          : Este programa está diseñado para el encendido de un led
 * 		     Al presionar un boton dispuesto en configuración Pull Down
 * 		     para evitar que el pin flote y depende la señal del botón
 * 		     el led se encienda o apague
 ******************************************************************************
 */

//Incluimos la bibliotecas para trabajar en C y con el micro STM32
#include <stdint.h>
#include <stm32f051x8.h>

int main(void)
{
	//Habilito el reloj del puerto B y C
	RCC->AHBENR |= (1<<18) | (1<<19);

	GPIOB->MODER |= (1<<15*2);//Configuramos el pin B15 como salida
	GPIOC->MODER &= ~(0x3<<6*2);//Configuramos el pin C6 como salida
	GPIOC->PUPDR &= ~(0x3 << (6 * 2));//Limpiamos posibles bits en alto en el registro para el pin 6
	//GPIOC->PUPDR |=  (0x2 << (6 * 2));//Habilitamos el pull-down para el bit 6
	GPIOC->PUPDR |=  (1 << (6 * 2));//Habilitamos el pull-down para el bit 6

    //Loop forever
	while(1){

		if(!(GPIOC->IDR &= (1<<6))){//Comparamos el estado del pin 6 en el registro de entrada con un estado en alto

			GPIOB->ODR |= (1<<15);//Si es verdad enviamos un alto lógico al pin B15 y enciende el led
		}//if

		else{

			GPIOB->ODR &= ~(1<<15);//De lo contrario enviamos un bajo lógico al pin B15 y apaga el led
		}//else

	}//while

}//main
