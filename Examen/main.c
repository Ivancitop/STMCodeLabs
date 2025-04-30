/*
 * Proyecto: Semáforo con STM32F0 Discovery
 * Autor: Iván Delgado Ramos
 * Fecha: 29/04/2025
 * Descripción: Diseñar e implementar un sistema que permita controlar el valor de la señal PWM
 * generada por el Timer 3 mediante entradas digitales para incrementar o disminuir su valor.
 * Las señales PWM controlarán un puente H BTS7960 utilizando la tarjeta de desarrollo STM32F0 Discovery.
 */

//Cabezeras
#include "stm32f051x8.h"
#include "stdint.h"

//Prototipos de función
void delayMs(uint16_t tiempoMs);//Delay
void pwmHandler(uint16_t dutyCycle, uint16_t sennal);//PWM
void movMotor(uint16_t dutyCycle, uint16_t sennal);//mover motor
void detenerMotor(void);//frenar motores


int main(void)//main
{
    //Habilitar reloj de puerto A (entradas digitales) y C (pwm)
    RCC->AHBENR |= (1 << 17) | (1 << 19);

    //Configurar pines A4-A6 como entrada con Pull-Up (Incrementadores)
	for (int pin = 4; pin <= 6; pin+=2) {
		GPIOA->MODER &= ~(0x3 << (pin * 2));
		GPIOA->PUPDR &= ~(0x3 << (pin * 2));
		GPIOA->PUPDR |=  (0x1 << (pin * 2));
	}

	//Configurar pines A5-A7 como entrada con Pull-Down (Decrementadores)
	for (int pin = 5; pin <= 7; pin+=2) {
		GPIOA->MODER &= ~(0x3 << (pin * 2));
		GPIOA->PUPDR &= ~(0x3 << (pin * 2));
		GPIOA->PUPDR |=  (0x2 << (pin * 2));
	}

    //Bandera
    int enMovimiento = 0;//Bandera que representa el estado de movimiento
    int inactividadMs = 0;//Bandera de estado de inactividad
    int dutycycleHandler = 0;//Acumulador del valor del dutyCycle pwm
    int movIzq = 0;//Bandera de estado de giro izquierdo
    int movD = 0;//Bandera de estado de giro derecho
    int botonPresionado = 0;//Bandera de estado de activación de boton

    while (1)//Ciclo principal
    {

        botonPresionado = 0;//Reinicio de botones presionados


        //Operación OR para determinar si se presionó un botón
        botonPresionado |= !(GPIOA->IDR & (1 << 4));
        botonPresionado |= (GPIOA->IDR & (1 << 5));
        botonPresionado |= !(GPIOA->IDR & (1 << 6));
        botonPresionado |= (GPIOA->IDR & (1 << 7));

        if (botonPresionado)//Si al menos se presiona un botón
        {

            inactividadMs = 0; // Se presionó algo, resetear contador de inactividad

            int botonesPresionados = 0;//Limpiamos la bandera, y lo usamos como sumador de operaciones bit a bit
            botonesPresionados += !(GPIOA->IDR & (1 << 4)) ? 1 : 0;// Si se presiona el botón, sumar 1, de lo contrario sumar 0
            botonesPresionados += (GPIOA->IDR & (1 << 5)) ? 1 : 0;
            botonesPresionados += !(GPIOA->IDR & (1 << 6)) ? 1 : 0;
            botonesPresionados += (GPIOA->IDR & (1 << 7)) ? 1 : 0;

            if (botonesPresionados > 1) continue; //Protección de múltiples botones

            //Si se presiona el boton, no hay otra operación en simultanea (presionar más de un btn)
            //si el duty cicly aún no es 100 y no nos estamos mov a la izquierda
            if (!(GPIOA->IDR & (1 << 4)) && (!enMovimiento)&& (dutycycleHandler!=100)&& (!movIzq)) // Incremento derecha
            {
            	movD=1;//Indico con bandera el mov derecho
                enMovimiento = 1;//Indico con bandera que hay una operación de movimiento
                dutycycleHandler+=10;//Aumento el valor de duty cycle en un paso de 10
                movMotor(dutycycleHandler, 1);//Muevo el motor derecho a cierto dutycycle (canal 1 "Der")
                movMotor(0, 2);//Evito enviar señal a la entrada de giro izquierdo (canal 2 "Izq")
                delayMs(1500);
                enMovimiento = 0;//Devuelvo el estado de movimiento para hacer otra operación
            }//if 2
            else if ((GPIOA->IDR & (1 << 5)) && (!enMovimiento)&& (dutycycleHandler!=0)&& (!movIzq)) // Decremento derecha
            {
            	movD=1;
                enMovimiento = 1;
                dutycycleHandler-=10;//Disminuyo el valor de duty cycle en un paso de 10
                movMotor(0, 2);
                movMotor(dutycycleHandler, 1);
                delayMs(1500);
                enMovimiento = 0;
            }//elif 1
            else if (!(GPIOA->IDR & (1 << 6)) && (!enMovimiento)&& (dutycycleHandler!=100)&& (!movD)) // Incremento izquierda
            {
            	movIzq=1;
                enMovimiento = 1;
                dutycycleHandler+=10;
                movMotor(dutycycleHandler, 2);
                movMotor(0, 1);
                delayMs(1500);
                enMovimiento = 0;
            }//elif 2
            else if (((GPIOA->IDR & (1 << 7))) && (!enMovimiento) && (dutycycleHandler!=0)&& (!movD)) // Decremento izquierda
            {
            	movIzq=1;
                enMovimiento = 1;
                dutycycleHandler-=10;
                movMotor(dutycycleHandler, 2);
                movMotor(0, 1);
                delayMs(1500);
                enMovimiento = 0;
            }//elif 3
        }//if 1
        else
        {
            delayMs(100); //Esperar 100ms entre chequeos
            inactividadMs += 100; //Aumentar el tiempo de inactividad

            if (inactividadMs >= 7000) //7 segundos de inactividad
            {
                detenerMotor();
                inactividadMs = 0; //Resetear contador
                //Reseteamos banderas y contadores
                movD=0;
                movIzq=0;
                dutycycleHandler = 0;
            }//if 3
        }//else 1
    } // while(1)
}//	main

void delayMs(uint16_t tiempoMs)//delay, entero como argumento
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; //Habilitar reloj de timer 6
    TIM6->PSC = 7;     // 8 MHz / (7+1) = 1 MHz
    TIM6->ARR = 999;   // 1 ms

    TIM6->SR = 0;
    TIM6->CR1 = TIM_CR1_CEN;

    while (tiempoMs != 0) {
        while (!(TIM6->SR & TIM_SR_UIF));
        TIM6->SR = 0;
        tiempoMs--;
    }
    TIM6->CR1 = 0;
}

void pwmHandler(uint16_t dutyCycle, uint16_t sennal)//Pwm, 2 enteros como argumento (magnitud duty cycle, canal)
													// 1 - Derecha, 2 - Izquierda
{
    int pin = (sennal == 1) ? 12 : 14;//Define el pin de acuerdo al canal definida en el argumento (operador ternario)

    GPIOC->MODER &= ~(0x3 << pin);
    GPIOC->MODER |= (0x2 << pin);

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 7;
    TIM3->ARR = 999;

    if (sennal == 1) {//Define de acuerdo a la señal definida en el argumento (operador ternario), el canal, el modo pwm y su habilitación
        TIM3->CCR1 = dutyCycle * 10;
        TIM3->CCMR1 &= ~(0x7 << 4);
        TIM3->CCMR1 |= (0x6 << 4);
        TIM3->CCER |= (1 << 0);
    } else {
        TIM3->CCR2 = dutyCycle * 10;
        TIM3->CCMR1 &= ~(0x7 << 12);
        TIM3->CCMR1 |= (0x6 << 12);
        TIM3->CCER |= (1 << 4);
    }

    TIM3->CR1 = TIM_CR1_CEN;
}

void movMotor(uint16_t dutyCycle, uint16_t sentido)//Función de motor
{
    pwmHandler(dutyCycle, sentido);//Llamado de la función de PWM
}

void detenerMotor(void)//Detener motores
{
    movMotor(0, 1);
    movMotor(0, 2);
}
