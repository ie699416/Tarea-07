/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    clase4.c
 * @brief   Application entry point.
 */

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define MAXCOUNT 10

SemaphoreHandle_t blueLED_semaphore;
SemaphoreHandle_t greenLED_semaphore;

void PORTA_IRQHandler() {
	BaseType_t xHigherPriorityTaskWoken;
	PORT_ClearPinsInterruptFlags(PORTA, 1 << 4);
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(blueLED_semaphore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void PORTC_IRQHandler() {
	BaseType_t xHigherPriorityTaskWoken;
	PORT_ClearPinsInterruptFlags(PORTC, 1 << 6);
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(greenLED_semaphore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void blue_led_task(void *arg) {
	for (;;) {
		xSemaphoreTake(blueLED_semaphore, portMAX_DELAY);
		GPIO_TogglePinsOutput(GPIOB, 1 << 21);
	}
}

void green_led_task(void *arg) {

	for (;;) {
		if (MAXCOUNT == uxSemaphoreGetCount(greenLED_semaphore)) {
			GPIO_TogglePinsOutput(GPIOE, 1 << 26);
			while(uxSemaphoreGetCount(greenLED_semaphore)){
				xSemaphoreTake(greenLED_semaphore, portMAX_DELAY);

			}

		}
	}

}

int main(void) {

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	CLOCK_EnableClock(kCLOCK_PortE);
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortA);

	port_pin_config_t config_led = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTB, 21, &config_led);
	PORT_SetPinConfig(PORTE, 26, &config_led);

	port_pin_config_t config_switch = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister };
	PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTC, 6, kPORT_InterruptFallingEdge);

	PORT_SetPinConfig(PORTA, 4, &config_switch);
	PORT_SetPinConfig(PORTC, 6, &config_switch);

	gpio_pin_config_t led_config_gpio = { kGPIO_DigitalOutput, 1 };

	GPIO_PinInit(GPIOB, 21, &led_config_gpio);
	GPIO_PinInit(GPIOE, 26, &led_config_gpio);

	gpio_pin_config_t switch_config_gpio = { kGPIO_DigitalInput, 0 };

	GPIO_PinInit(GPIOA, 4, &switch_config_gpio);
	GPIO_PinInit(GPIOC, 6, &switch_config_gpio);

	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);
	NVIC_SetPriority(PORTA_IRQn, 5);
	NVIC_SetPriority(PORTC_IRQn, 6);

	GPIO_WritePinOutput(GPIOB, 21, 0);
	GPIO_WritePinOutput(GPIOE, 26, 0);

	blueLED_semaphore = xSemaphoreCreateBinary();
	/* The semaphore cannot be used before it is created using a call to xSemaphoreCreateCounting().
	 The maximum value to which the semaphore can count in this example case is set to 10,
	 and the initial value assigned to the count is set to 0. */
	greenLED_semaphore = xSemaphoreCreateCounting(MAXCOUNT, 0);
	xTaskCreate(blue_led_task, "blue LED task", configMINIMAL_STACK_SIZE, NULL,
			configMAX_PRIORITIES - 1, NULL);

	if (greenLED_semaphore != NULL) {
		/* The semaphore was created successfully. The semaphore can now be used. */

		xTaskCreate(green_led_task, "green LED task",
		configMINIMAL_STACK_SIZE,
		NULL, configMAX_PRIORITIES - 2, NULL);
	}
	vTaskStartScheduler();
	while (1) {

	}
	return 0;
}
