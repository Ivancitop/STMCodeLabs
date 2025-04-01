/*
 ******************************************************************************
 * @file           : Práctica 1
 * @author         : Iván Delgado Ramos
 * @brief          : Encender y apagar los leds de los pines C8 y C9
 ******************************************************************************
 */

#include <stdint.h> //Biblioteca para usar funciones de C
#include <stm32f051x8.h>//Biblioteca para trabajar con los registros, buses y relojes del mirco

void delay(volatile uint32_t count) {//Definimos una función para consumir ciclos de reloj, funcionando como un delay
    while (count--) {}//Dado el parametro vamos decrmentando el valor hasta que este se vulva 0 y salga del ciclo
}

int main(void) {
    // Habilitar el reloj del GPIOC
    RCC->AHBENR |= (1<<19);

    // Configurar GPIOC 8 y 9 como salida
    GPIOC->MODER &= ~(0xF << 16); // Limpiar MODER8 y MODER9
    GPIOC->MODER |= (0x5 << 16);  // Configurar como salida

    // Ciclo principal
    while (1) {
        // Encender y Apagar LEDs (poner a 1 ODR8 y ODR9)
        GPIOC->ODR ^= (1 << 8) ^ (1 << 9);
        delay(4000000);


    }
}
