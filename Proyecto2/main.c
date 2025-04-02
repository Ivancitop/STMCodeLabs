/*
 * Proyecto: Semáforo con STM32F0 Discovery
 * Autor: Iván Delgado Ramos
 * Fecha: 01/03/2025
 * Descripción: Este código configura los pines GPIOA7, GPIOB0 y GPIOC5 como salidas 
 * para emular un semáforo. Se usa un loop infinito con retardos para cambiar los LEDs 
 * en la secuencia típica de un semáforo (verde -> amarillo -> rojo).
 * 
 * Configuración de pines:
 * - PA7 -> LED verde
 * - PB0 -> LED rojo
 * - PC5 -> LED amarillo
 * 
 * Se utiliza el reloj del sistema y retardos generados con un ciclo while.
 */

#include <stdint.h> //Biblioteca para usar funciones de C
#include <stm32f051x8.h>//Biblioteca para trabajar con los registros, buses y relojes del mirco

void delay(volatile uint32_t time) {//Definimos una función para consumir ciclos de reloj, funcionando como un delay
	volatile uint32_t count =time*1000000;
    while (count--) {}//Dado el parametro vamos decrementando el valor hasta que este se vulva 0 y salga del ciclo
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
	// Encender LED verde por 5 segundos y apagar los demás
        GPIOA->ODR |= (1 << 7);
        GPIOB->ODR &= ~(1 << 0);
        GPIOC->ODR &= ~(1 << 5);
        delay(5);

        // Encender LED amarillo por 2 segundos y apagar los demás
        GPIOC->ODR |= (1 << 5);
        GPIOA->ODR &= ~(1 << 7);
        GPIOB->ODR &= ~(1 << 0);
        delay(2);

        // Encender LED rojo por 5 segundos y apagar los demás
        GPIOB->ODR |= (1 << 0);
        GPIOA->ODR &= ~(1 << 7);
        GPIOC->ODR &= ~(1 << 5);
        delay(5);


    }
}
