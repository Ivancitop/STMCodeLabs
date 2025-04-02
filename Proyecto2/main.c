/*
 ******************************************************************************
 * @file           : Práctica 2
 * @author         : Iván Delgado Ramos
 * @brief          : Configuración de pines GPIO para emular un semáforo
 ******************************************************************************
 */

#include <stdint.h> //Biblioteca para usar funciones de C
#include <stm32f051x8.h>//Biblioteca para trabajar con los registros, buses y relojes del mirco

void delay(volatile uint32_t time) {//Definimos una función para consumir ciclos de reloj, funcionando como un delay
	volatile uint32_t count =time*1000000;
    while (count--) {}//Dado el parametro vamos decrmentando el valor hasta que este se vulva 0 y salga del ciclo
}

int main(void) {
    // Habilitar el reloj del GPIOA, GPIOB, GPIOC
    RCC->AHBENR |= (1<<17) | (1<<18) | (1<<19);

    // limpiamos los modos de operación previos de los pines
    GPIOA->MODER &= ~(0x3 << 14); // Limpiar MODER A7
    GPIOB->MODER &= ~(0x3 << 0); // Limpiar MODER B0
    GPIOC->MODER &= ~(0x3 << 10); // Limpiar MODER C5

    // Definimos los pines como salida general
    GPIOA->MODER |= (1 << 14);  // Configurar como salida A7
    GPIOB->MODER |= (1 << 0);  // Configurar como salida B0
    GPIOC->MODER |= (1 << 10);  // Configurar como salida C5

    //Loop
    while (1) {
        // Encender led verde por 5 segundos
        GPIOA->ODR |= (1 << 7);
        GPIOB->ODR &= (0 << 0);
        GPIOC->ODR &= (0 << 5);
        delay(5);

        // Encender led amarillo por 2 segundos
        GPIOC->ODR |= (1 << 5);
        GPIOA->ODR &= (0 << 7);
        GPIOB->ODR &= (0 << 0);
        delay(2);

        // Encender led rojo por 5 segundos
        GPIOB->ODR |= (1 << 0);
        GPIOA->ODR &= (0 << 7);
        GPIOC->ODR &= (0 << 5);
        delay(5);


    }
}
