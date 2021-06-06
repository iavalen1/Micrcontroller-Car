/*
 *
 * Copyright 2016-2020 NXP
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
 * @file    Project 3.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
void control_function();
void init_timer();
void interrupt();
int current_distance = 0;
int last_distance = 0;
float pwm = 0;
float kp = 0.1;
float ki = 0.01;
float desired_speed = (90/1249);
float measured_speed;
float sumError;
int time = 0;

/*
 * @brief   Application entry point.
 */

static int count = 0;

int main(void) {

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	//Clock gating
	SIM->SCGC6 |= 1 << 26; // Enable clock gate for TPM2
	SIM->SCGC5 |= 1 << 10 | 1 << 12; //Enable port b and port d clock gating
	SIM->SCGC5 |= 1 << 11; //Enable port c clock gating
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK: 8MHz

	//Motor, Switch and Servo PWM port/pin
	PORTB->PCR[2] &= ~0x700; //clears the mux for port B pin 2
	PORTB->PCR[2] |= 0x300; //sets the mux for port B pin 2 to TPM2 Channel 0
	PORTB->PCR[3] &= ~0x700; //Clears the mux for port B pin 3
	PORTB->PCR[3] |= 0x300; //Sets the mux for port B pin 3 to TPM2 Channel 1
	PORTC->PCR[3] &= ~0x703; //Clears the mux for port C pin 3
	PORTC->PCR[3] |= 0x103; //Sets the mux for port C pin 3 to GPIO
	PORTD->PCR[3] &= ~0x700; //Clears the mux for port D pin 3
	PORTD->PCR[3] |= 0x100; //Sets the mux tfor port D pin 3 to GPIO

	//Setup GPIO for motor direction
	GPIOD->PDDR = 1 << 3; // Sets Port D Pin 3 to output

	//Setup GPIO for switch 1
	GPIOC->PDDR &= ~(1 << 3); //Sets Port C pin 3 to input

	//Setup TPM clock
	TPM2->MOD = 1249; //Sets modulo or period for 20 ms
	TPM2->SC |= 0x07; //Sets prescalar to 128

	//Setup channel 0 for servo
	TPM2->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4); // Clear output on match and set output on reload
	TPM2->CONTROLS[0].CnV = 93; // 7.5% duty cycle / servo starts in neutral

	//Setup channel 1 for motor
	TPM2->CONTROLS[1].CnSC |= (0x2 << 2) | (0x2 << 4); // Clear output on match and set output on reload
	TPM2->CONTROLS[1].CnV = 0; // 0% duty cycle / motor starts turned off

	//Start TPM clock
	TPM2->SC |= 0x01 << 3; // Start the clock

	interrupt();

	while (1) {
		if (GPIOC->PDIR == (0 << 3)) { //Checks if button was pressed

			TPM2->CONTROLS[1].CnV = 90; // 5% duty cycle / motor starts turned off
			init_timer(); //Initialize timer

			//runs until the distance of 10m is reached
			while (count <= 550) {
				current_distance = count;
				if (time % 63 == 0) {
					measured_speed = (current_distance - last_distance) / 63;
					//printf("current distance: %d last distance: %d measured speed: %d\n", current_distance, last_distance, measured_speed);
					control_function(); //calls control function

					//checks to see if motor surpasses 20% duty cycle
					if ((TPM2->CONTROLS[1].CnV + (pwm*1249)) > 250) {
						TPM2->CONTROLS[1].CnV = 250;
					} else {
						//adjusts the motor speed based off of pwm value from control function
						TPM2->CONTROLS[1].CnV += (pwm*1249);
					}
					last_distance = current_distance;
				}

			}

			TPM2->CONTROLS[1].CnV = 0; // 0% duty cycle / motor starts turned off
			time = 0;
			count=0;
			current_distance = 0;
			last_distance = 0;
			pwm = 0;
		}
	}
	return 0;

}

void PORTC_PORTD_IRQHandler(void) {
	PORTD->PCR[2] |= (1 << 24); //clears flag by detecting the flag first
	count++; //increment counter
}

void interrupt() {
	PORTD->PCR[2] &= ~0xF0703; //clear port D
	PORTD->PCR[2] |= ((0x9 << 16) | (1 << 8) | 0x3); //Set mux, interrupt on rising edge, pf, and pe
	GPIOD->PDDR &= ~(1 << 2); // Setup pin 3 port d as input

	NVIC_EnableIRQ(31); //Call core api to enable irq
}

void control_function() {
	float error = (desired_speed - measured_speed);
	sumError += error;
	pwm = kp * error + ki * sumError;

}

void TPM0_IRQHandler(void) {
	TPM0->SC |= (1 << 7); // Reset Timer Interrupt
	time++;  // What happens on overflow?
}

void init_timer() {
	SIM->SCGC6 |= (1 << 24); // Clock enable TPM0
	SIM->SOPT2 |= (0x2 << 24); // TPM Clock Source OSCERCLK
	TPM0->MOD = 63; // Reload every millisecond.
	TPM0->SC = (1 << 7) | (1 << 6) | (1 << 3) | (0x7); // Reset TOF, Enable Interrupt, Prescaler = 128, Start Timer
	NVIC_EnableIRQ(17);
}

