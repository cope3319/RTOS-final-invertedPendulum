/*
 * app.h
 *
 *  Created on: Feb 11, 2021
 *      Author: peski
 */

#ifndef SRC_APP_H_
#define SRC_APP_H_

#include "em_emu.h"
#include  <kernel/include/os.h>
#include "display.h"
#include "textdisplay.h"
#include "bspconfig.h"
#include "retargettextdisplay.h"


/* ---------- Button Event FIFO ----------- */
struct button_event {
	// pushbutton being pressed and button state
	uint8_t pushbutton;
	uint8_t button_state;

	// pointer to next button event message
	struct button_event* next;
};
// pushbuttons enum
enum PUSHBUTTONS{
	PUSHBUTTON_PB0,
	PUSHBUTTON_PB1
};
// pushbutton states enum
enum BUTTON_STATES{
	PB_NOTPRESSED,
	PB_PRESSED
};
// Head of button event fifo structure
struct button_event* button_event_fifo_head;

/* ---------- Speed Setpoint Data Struct ----------- */
struct speed_setpoint_data_struct {
	uint32_t current_speed;
	uint32_t number_increments;
	uint32_t number_decrements;
	bool speed_violation;
};
// Speed Setpoint Data
struct speed_setpoint_data_struct speed_setpoint_data;

/* ---------- Vehicle Direction Data Struct ----------- */
struct vehicle_direction_data_struct {
	uint8_t current_direction;
	uint32_t time_held;
	uint32_t num_left_turns;
	uint32_t num_right_turns;
	uint32_t direction_violation;
};
// Vehicle Direction Data
struct vehicle_direction_data_struct vehicle_direction_data;

// enum for directions
enum DIRECTIONS{
	DIRECTION_STRAIGHT,
	DIRECTION_LEFT,
	DIRECTION_HARD_LEFT,
	DIRECTION_RIGHT,
	DIRECTION_HARD_RIGHT
};

/* ---------- ITC Constructs ----------- */
OS_SEM Speed_Setpoint_Semaphore;
OS_MUTEX Speed_Setpoint_Mutex;
OS_MUTEX Vehicle_Direction_Mutex;
OS_FLAG_GRP Vehicle_Monitor_Flags;
#define VEHICLE_MONITOR_FLAG_SPEED 		(1u << 0)
#define VEHICLE_MONITOR_FLAG_DIRECTION	(1u << 1)
#define VEHICLE_MONITOR_FLAG_ALL 		(VEHICLE_MONITOR_FLAG_SPEED | VEHICLE_MONITOR_FLAG_DIRECTION)
OS_FLAG_GRP LED_Output_Flags;
#define LED_OUTPUT_FLAG_LED0    	    (1u << 0)
#define LED_OUTPUT_FLAG_LED1    	    (1u << 1)
#define LED_OUTPUT_FLAG_ALL				(LED_OUTPUT_FLAG_LED0 | LED_OUTPUT_FLAG_LED1)

/* ---------- PUSHBUTTONS ----------- */
#define PB0_PORT        gpioPortF
#define PB0_PIN         6u
#define PB1_PORT        gpioPortF
#define PB1_PIN         7u


/* ---------- CAPACITIVE TOUCH ----------- */
#define TOUCH0_PORT     gpioPortC
#define TOUCH0_PIN      0u
#define TOUCH1_PORT     gpioPortC
#define TOUCH1_PIN      1u
#define TOUCH2_PORT     gpioPortC
#define TOUCH2_PIN      2u
#define TOUCH3_PORT     gpioPortC
#define TOUCH3_PIN      3u
// The capsenseconfig.h file in the SDK example only has defines
// for BUTTON0_CHANNEL (PC0) and BUTTON1_CHANNEL (PC3), so I
// define one for all four of the touch pad to be able to poll all four.
#define TOUCH0_CHANNEL      0
#define TOUCH1_CHANNEL      1
#define TOUCH2_CHANNEL      2
#define TOUCH3_CHANNEL      3

/* ---------- LED's ----------- */
#define LED0_PORT       gpioPortF
#define LED0_PIN        4u
#define LED1_PORT       gpioPortF
#define LED1_PIN        5u


void init_ITC();
void PB0_button_event(void);
void PB1_button_event(void);
uint8_t sample_touch(void);
void push_button_event(uint8_t pushbutton, uint8_t state);
struct button_event* peek_button_event(void);
void pop_button_event(void);
void button_open(void);
void buttonInterrupt_init(void);
void led_open(void);
void hardware_init(void);
void all_GPIO_IRQ(void);
void GPIO_EVEN_IRQHandler(void);
void GPIO_ODD_IRQHandler(void);


#endif /* SRC_APP_H_ */
