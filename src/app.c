/*
 * app.c
 *
 *  Created on: Jan 28, 2021
 *      Author: peski
 */

#include <stdint.h>
#include <stdlib.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "app.h"
#include "capsense.h"
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>

/* ----- Global Variables -----  */

/* ---------------------------------------------------------------- */
/*   ___               ______                _   _                  */
/*  / _ \              |  ___|              | | (_)                 */
/* / /_\ \_ __  _ __   | |_ _   _ _ __   ___| |_ _  ___  _ __  ___  */
/* |  _  | '_ \| '_ \  |  _| | | | '_ \ / __| __| |/ _ \| '_ \/ __| */
/* | | | | |_) | |_) | | | | |_| | | | | (__| |_| | (_) | | | \__ \ */
/* \_| |_/ .__/| .__/  \_|  \__,_|_| |_|\___|\__|_|\___/|_| |_|___/ */
/*       | |   | |                                                  */
/*       |_|   |_|                                                  */
/* ---------------------------------------------------------------- */

/**************************************************************************//**
 * @brief
 * Initialize all Inter-Task-Communication Stuffs
 *
 * @details
 * Creates Semaphores, Mutexs, and flags for the application.
 *
 *****************************************************************************/
void init_ITC(void){
	RTOS_ERR err;
	/* -- Create Speed Setpoint Semaphore -- */
	OSSemCreate(&Speed_Setpoint_Semaphore,
				"Speed Setpoint Semaphore",
				1,
				&err);
	//Check for Error
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	/* -- Create Speed Setpoint Mutex -- */
	OSMutexCreate(&Speed_Setpoint_Mutex,
				  "Speed Setpoint Mutex",
				  &err);
	//Check for Error
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	/* -- Create Vehicle Direction Mutex -- */
	OSMutexCreate(&Vehicle_Direction_Mutex,
				  "Vehicle Direction Mutex",
				  &err);
	//Check for Error
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	/* -- Create Vehicle Monitor Flags -- */
	OSFlagCreate(&Vehicle_Monitor_Flags,
				 "Vehicle Monitor Flags",
			 	 0,
				 &err);
	/* -- Create Vehicle Monitor Flags -- */
	OSFlagCreate(&LED_Output_Flags,
				 "LED Output Flags",
			 	 0,
				 &err);
	//Check for Error
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

}

/**************************************************************************//**
 * @brief
 * Create PB0 Event
 *
 * @details
 * Pushes an event to the FIFO depending on the state of PB0.
 *
 *****************************************************************************/
void PB0_button_event(void){
	// get current state
	uint8_t state;
	if(!GPIO_PinInGet(PB0_PORT, PB0_PIN)) state = PB_PRESSED;
	else state = PB_NOTPRESSED;

	// push to FIFO
	push_button_event(PUSHBUTTON_PB0, state);

	// post speed setpoint semaphore
	RTOS_ERR err;
	OSSemPost(&Speed_Setpoint_Semaphore,
			  OS_OPT_POST_1,
			  &err);
	//Check for Error
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
}


/**************************************************************************//**
 * @brief
 * Create PB1 Event
 *
 * @details
 * Pushes an event to the FIFO depending on the state of PB1.
 *
 *****************************************************************************/
void PB1_button_event(void){
	// get current state
	uint8_t state;
	if(!GPIO_PinInGet(PB1_PORT, PB1_PIN)) state = PB_PRESSED;
	else state = PB_NOTPRESSED;

	// push to FIFO
	push_button_event(PUSHBUTTON_PB1, state);

	// post speed setpoint semaphore
	RTOS_ERR err;
	OSSemPost(&Speed_Setpoint_Semaphore,
			  OS_OPT_POST_1,
			  &err);
}

/**************************************************************************//**
 * @brief
 *  Sample the PG12 Starter Kit Capacitive Touch sensor
 *
 * @details
 * Returns the current direction of the touch slider, per the DIRECTIONS
 * enum.
 *
 *****************************************************************************/
uint8_t sample_touch(void){
    // will be using functions defined in the capsense.c file provided
    // in the SDK to poll the capacitive touch slider.
    CAPSENSE_Sense();
    uint8_t state = DIRECTION_STRAIGHT;
    if(CAPSENSE_getPressed(TOUCH0_CHANNEL) && !CAPSENSE_getPressed(TOUCH1_CHANNEL)
        && !CAPSENSE_getPressed(TOUCH2_CHANNEL) && !CAPSENSE_getPressed(TOUCH3_CHANNEL))
            state = DIRECTION_HARD_LEFT;
    if(CAPSENSE_getPressed(TOUCH1_CHANNEL) && !CAPSENSE_getPressed(TOUCH0_CHANNEL)
        && !CAPSENSE_getPressed(TOUCH2_CHANNEL) && !CAPSENSE_getPressed(TOUCH3_CHANNEL))
            state = DIRECTION_LEFT;
    if(CAPSENSE_getPressed(TOUCH2_CHANNEL) && !CAPSENSE_getPressed(TOUCH0_CHANNEL)
        && !CAPSENSE_getPressed(TOUCH1_CHANNEL) && !CAPSENSE_getPressed(TOUCH3_CHANNEL))
            state = DIRECTION_RIGHT;
    if(CAPSENSE_getPressed(TOUCH3_CHANNEL) && !CAPSENSE_getPressed(TOUCH0_CHANNEL)
        && !CAPSENSE_getPressed(TOUCH2_CHANNEL) && !CAPSENSE_getPressed(TOUCH1_CHANNEL))
            state = DIRECTION_HARD_RIGHT;
    if((CAPSENSE_getPressed(TOUCH0_CHANNEL) || CAPSENSE_getPressed(TOUCH1_CHANNEL))
        && (CAPSENSE_getPressed(TOUCH2_CHANNEL) || CAPSENSE_getPressed(TOUCH3_CHANNEL)))
            state = DIRECTION_STRAIGHT;
    return state; //default
}

/* ----------------------------------------------------- */
/* ______ ___________ _____   _                 _        */
/* |  ___|_   _|  ___|  _  | | |               (_)       */
/* | |_    | | | |_  | | | | | |     ___   __ _ _  ___   */
/* |  _|   | | |  _| | | | | | |    / _ \ / _` | |/ __|  */
/* | |    _| |_| |   \ \_/ / | |___| (_) | (_| | | (__   */
/* \_|    \___/\_|    \___/  \_____/\___/ \__, |_|\___|  */
/*                                         __/ |         */
/*                                        |___/          */
/* ----------------------------------------------------- */
/**************************************************************************//**
 * @brief
 *  Push Button Event
 *
 * @details
 * Pushes a button evenet to the FIFO per the parameters passed.
 *
 * [uint8_t] pushbutton - the pushbutton changed
 * [uint8_t] state - the state of the pushbutton
 *
 *****************************************************************************/
void push_button_event(uint8_t pushbutton, uint8_t state){
	/* -- create new button event -- */
	struct button_event* new_button_event = (struct button_event*)malloc(sizeof(struct button_event));
	new_button_event->pushbutton = pushbutton;
	new_button_event->button_state = state;
	new_button_event->next = NULL;

	/* -- Append to current list -- */
	// edge case- head is null
	if(button_event_fifo_head == NULL){
		button_event_fifo_head = new_button_event;
		return;
	}
	struct button_event* cur = button_event_fifo_head;
	while(cur->next != NULL) cur = cur->next;
	cur->next = new_button_event;
	return;
}

/**************************************************************************//**
 * @brief
 *  Peek Button Event
 *
 * @details
 * Returns the head of the FIFO struct.
 *
 *****************************************************************************/
struct button_event* peek_button_event(void){
	return button_event_fifo_head;
}

/**************************************************************************//**
 * @brief
 *  Pop Button Event
 *
 * @details
 * Removes the event at the top of the FIFO structure.
 *
 *****************************************************************************/
void pop_button_event(void){
	if(button_event_fifo_head == NULL) return;
	struct button_event* tmp = button_event_fifo_head;
	button_event_fifo_head = button_event_fifo_head->next;
	free(tmp);
}



/* ------------------------------------------------------------------ */
/* | | | |             | |                        |_   _|    (_) |    */
/* | |_| | __ _ _ __ __| |_      ____ _ _ __ ___    | | _ __  _| |_   */
/* |  _  |/ _` | '__/ _` \ \ /\ / / _` | '__/ _ \   | || '_ \| | __|  */
/* | | | | (_| | | | (_| |\ V  V / (_| | | |  __/  _| || | | | | |_   */
/* \_| |_/\__,_|_|  \__,_| \_/\_/ \__,_|_|  \___|  \___/_| |_|_|\__|  */
/* ------------------------------------------------------------------ */

/**************************************************************************//**
 * @brief
 * Button Init Function
 *
 * @details
 * Enables the GPIO clock and initializes the Pushbuttons on the PG12
 *
 *****************************************************************************/
void button_open(void){
    CMU_ClockEnable(cmuClock_GPIO, true);
    // ------------- PUSHBUTTONS -------------
    // configure pushbuttons as input
    GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInputPull, 1); //PushButton 0
    GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInputPull, 1); //PushButton 1
}

/**************************************************************************//**
 * @brief
 *  Button Interrupt Initialization Function
 *
 * @details
 *  Enables interrupts for PB0 and PB1 of the PG12 Starter Kit on both rising
 *  and falling edges.
 *
 *****************************************************************************/
void buttonInterrupt_init(void){
    // ----------- PUSHBUTTONS -----------
    // enable interrupts on pushbuttons
    GPIO_IntConfig(PB0_PORT, PB0_PIN, true, false, true); //interrupt rising edge only
    GPIO_IntConfig(PB1_PORT, PB1_PIN, true, false, true); //interrupt rising edge only

    //using even and odd pins for pushbuttons
    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/**************************************************************************//**
 * @brief
 * LED Init Function
 *
 * @details
 * Enables the GPIO clock and initializes the LED's on the PG12
 *
 *****************************************************************************/
void led_open(void) {
    CMU_ClockEnable(cmuClock_GPIO, true);
    // ------------- LED's -------------
    GPIO_DriveStrengthSet(LED0_PORT, gpioDriveStrengthWeakAlternateWeak); // led0
    GPIO_PinModeSet(LED0_PORT, LED0_PIN, gpioModePushPull, false); //default off
    GPIO_DriveStrengthSet(LED1_PORT, gpioDriveStrengthWeakAlternateWeak); // led1
    GPIO_PinModeSet(LED1_PORT, LED1_PIN, gpioModePushPull, false); //default off
}

/**************************************************************************//**
 * @brief
 * Initialize Hardware
 *
 * @details
 * Initialize/open all hardware for this application.
 *
 *****************************************************************************/
void hardware_init(void){
	// initialize button clock and gpio
	button_open();
	buttonInterrupt_init();
	/* Start capacitive sense buttons using the capsense.c
	 * source code provided in the SDK example. */
	CAPSENSE_Init();
	// initialize LED clock and gpio
	led_open();

	/* --------------------
	 * LCD DISLAY INIT
	 * -------------------- */
	EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_STK_DEFAULT;
	CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_STK_DEFAULT;
    /* Init DCDC regulator and HFXO with kit specific parameters */
    EMU_DCDCInit(&dcdcInit);
    CMU_HFXOInit(&hfxoInit);

    /* Switch HFCLK to HFXO and disable HFRCO */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
    /* Initialize the display module. */
    DISPLAY_Init();
    /* Retarget stdio to a text display. */
    if (RETARGET_TextDisplayInit() != TEXTDISPLAY_EMSTATUS_OK) {
      while (1) ;
    }
};

/* -------------------------------------------------- */
/*  _____      _                             _        */
/* |_   _|    | |                           | |       */
/*   | | _ __ | |_ ___ _ __ _ __ _   _ _ __ | |_ ___  */
/*   | || '_ \| __/ _ \ '__| '__| | | | '_ \| __/ __| */
/*  _| || | | | ||  __/ |  | |  | |_| | |_) | |_\__ \ */
/*  \___/_| |_|\__\___|_|  |_|   \__,_| .__/ \__|___/ */
/*                                    | |             */
/*                                    |_|             */
/* -------------------------------------------------- */
/**************************************************************************//**
 * @brief
 *  All GPIO Interrupt IRQ Handler
 *
 * @details
 *  Clears and handles GPIO interrupts. Current implementation supports:
 *   - Pushbutton 0/1
 *
 *****************************************************************************/
void all_GPIO_IRQ(void){
    uint32_t interrupt_mask = GPIO_IntGet();
    GPIO_IntClear(interrupt_mask);
//   ----------- PUSHBUTTONS -----------
    if (interrupt_mask & (1 << PB0_PIN)) {
		PB0_button_event();
    }
    if (interrupt_mask & (1 << PB1_PIN)) {
		PB1_button_event();
    }
}

/**************************************************************************//**
 * @brief
 *  GPIO EVEN IRQ Handler
 *
 * @details
 *  All GPIO IRQ handlers route to all_GPIO_IRQ() function and are handled there.
 *
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void){
    all_GPIO_IRQ();
}

/**************************************************************************//**
 * @brief
 *  GPIO ODD IRQ Handler
 *
 * @details
 *  All GPIO IRQ handlers route to all_GPIO_IRQ() function and are handled there.
 *
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void){
    all_GPIO_IRQ();
}
