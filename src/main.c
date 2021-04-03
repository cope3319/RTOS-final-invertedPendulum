/***************************************************************************//**
 * @file
 * @brief Connor Peskin RTSO Lab 2 Spring 2021
 *******************************************************************************
 * GPIO, Timers, Interrupts
 *
 * !!!All SysTick and capsense code belongs to Silicon Laboratories Inc.!!!
 ******************************************************************************/

/*
 *  ------ INCLUDE FILES  ------
 *  									  */

#include <stdio.h>
#include <stdlib.h>
#include "stdint.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_core.h"
#include "em_device.h"
#include "em_chip.h"
#include "capsense.h" // for touch slider

#include "app.h"
#include  <bsp_os.h>
#include  "bsp.h"
#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>

#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>
#include "display.h"
#include "textdisplay.h"
#include "retargettextdisplay.h"
#include "glib.h"
#include "dmd.h"
#include <string.h>
#include "em_prs.h"

/*
 *  ------ LOCAL DEFINES ------
 *  									  */

// DEFINES IF WE WILL USE THE INTERRUPTs to Poll or Delay
#define LAB2_USE_INTERRUPT true

#define  MAIN_START_TASK_PRIO                20u
#define  MAIN_START_TASK_STK_SIZE            512u
#define  SPEED_SETPOINT_TASK_PRIO            22u //#2
#define  SPEED_SETPOINT_TASK_STK_SIZE        512u
#define  VECHICLE_DIRECTION_TASK_PRIO        21u //#1
#define  VEHICLE_DIRECTION_TASK_STK_SIZE     512u
#define  VEHICLE_MONITOR_TASK_PRIO           23u //#3
#define  VEHICLE_MONITOR_TASK_STK_SIZE       512u
#define  LED_OUTPUT_TASK_PRIO                24u //#4
#define  LED_OUTPUT_TASK_STK_SIZE            512u
#define  LCD_DISPLAY_TASK_PRIO               25u //#5
#define  LCD_DISPLAY_TASK_STK_SIZE           512u


/*
 *  ------ LOCAL GLOBAL VARIABLES ------
 *  									  */

/* Define the Stack and TCP variables for the tasks */

//  Main Start Task
static  CPU_STK  MainStartTaskStk[MAIN_START_TASK_STK_SIZE];
static  OS_TCB   MainStartTaskTCB;

// Speed Setpoint Task
static  CPU_STK  Speed_Setpoint_stk[SPEED_SETPOINT_TASK_STK_SIZE];
static  OS_TCB   Speed_Setpoint_TCB;
// Vehicle Direction Task
static  CPU_STK  Vehicle_Direction_stk[VEHICLE_DIRECTION_TASK_STK_SIZE];
static  OS_TCB   Vehicle_Direction_TCB;
// Vehicle Montor Task;
static  CPU_STK  Vehicle_Monitor_stk[VEHICLE_MONITOR_TASK_STK_SIZE];
static  OS_TCB   Vehicle_Monitor_TCB;
// LED Output Task;
static  CPU_STK  LED_Output_stk[LED_OUTPUT_TASK_STK_SIZE];
static  OS_TCB   LED_Output_TCB;
// LCD Display Task;
static  CPU_STK  LCD_Display_stk[LCD_DISPLAY_TASK_STK_SIZE];
static  OS_TCB   LCD_Display_TCB;


/*
 *  ------ LOCAL FUNCTION PROTOTYPES ------
 *  									  */

static void MainStartTask ();

static void Speed_Setpoint_Task();
static void Vehicle_Direction_Task();
static void Vehicle_Monitor_Task();
static void LED_Output_Task();
static void LCD_Display_Task();



/*
 *  ------ GLOBAL FUNCTIONS ------
 *  									  */


/**************************************************************************//**
 * @brief
 * Main function
 *
 * @details
 * Calls the main start task
 *
 *****************************************************************************/
int main(void)
{
	RTOS_ERR err;

	/* -- Init System -- */
	/* Chip errata */
	CHIP_Init();
	/* Initialize System */
	BSP_SystemInit();
	CPU_Init();
	OS_TRACE_INIT();
	/* Initialize the Kernel.                               */
    OSInit(&err);


	/* -- Create Main Start Task --	 */
    OSTaskCreate(&MainStartTaskTCB,
                 "Main Start Task",
                  MainStartTask,
                  DEF_NULL,
                  MAIN_START_TASK_PRIO,
                 &MainStartTaskStk[0],
                 (MAIN_START_TASK_STK_SIZE / 10u),
                  MAIN_START_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);

    /* Check for Error */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	/* Start the kernel. */
    OSStart(&err);
    /* Check for Error */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}


/*
 * --------------- LOCAL FUNCTIONS ---------------
 * 													*/
/**************************************************************************//**
 * @brie
 * Main Start Task
 *
 * @details
 * This task is to be called in the main function and will start all other tasks
 * using Micrium OS. Once complete, it will delete itself.
 *
 *****************************************************************************/
static void MainStartTask () {
	RTOS_ERR err;

    Common_Init(&err);   /* Call common module initialization example.           */
    BSP_OS_Init();       /* Initialize the BSP. It is expected that the BSP ...  */
    APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE, ;);
	// Initialize all hardware for application
	hardware_init();
	// initialize inter-task-communication structures for application
	init_ITC();


	/* -- Initialize all Application Tasks -- */
    OSTaskCreate(&Speed_Setpoint_TCB,
                 "Speed Setpoint Task",
                  Speed_Setpoint_Task,
                  DEF_NULL,
                  SPEED_SETPOINT_TASK_PRIO,
                 &Speed_Setpoint_stk[0],
                 (SPEED_SETPOINT_TASK_STK_SIZE / 10u),
                  SPEED_SETPOINT_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    OSTaskCreate(&Vehicle_Direction_TCB,
                 "Vehicle Direction Task",
                  Vehicle_Direction_Task,
                  DEF_NULL,
                  VEHICLE_MONITOR_TASK_PRIO,
                 &Vehicle_Direction_stk[0],
                 (VEHICLE_DIRECTION_TASK_STK_SIZE / 10u),
                  VEHICLE_DIRECTION_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    OSTaskCreate(&Vehicle_Monitor_TCB,
                 "Vehicle Monitor Task",
                  Vehicle_Monitor_Task,
                  DEF_NULL,
                  VEHICLE_MONITOR_TASK_PRIO,
                 &Vehicle_Monitor_stk[0],
                 (VEHICLE_MONITOR_TASK_STK_SIZE / 10u),
                  VEHICLE_MONITOR_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    OSTaskCreate(&LED_Output_TCB,
                 "LED Output Task",
                  LED_Output_Task,
                  DEF_NULL,
                  LED_OUTPUT_TASK_PRIO,
                 &LED_Output_stk[0],
                 (LED_OUTPUT_TASK_STK_SIZE / 10u),
                  LED_OUTPUT_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    OSTaskCreate(&LCD_Display_TCB,
                 "LCD Display Task",
                  LCD_Display_Task,
                  DEF_NULL,
                  LCD_DISPLAY_TASK_PRIO,
                 &LCD_Display_stk[0],
                 (LCD_DISPLAY_TASK_STK_SIZE / 10u),
                  LCD_DISPLAY_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);

	//Check for Error
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    // "a task should either be an infinite loop or delete itself if it's done.":w
    OSTaskDel(&MainStartTaskTCB, &err);
}

/**************************************************************************//**
 * @brief
 * Speed Setpoint Task
 *
 * @details
 * "Responsible for updating the Speed Setpoint data store upon user activation
 * of the Speed Increment and Speed Decrement buttons. It is awakened by
 * a semaphore that is posted by the Button ISR(s) when either button changes
 * state. It notifies the Vehicle Monitor task of any change in speed using
 * an event flag."
 *
 *****************************************************************************/
static void Speed_Setpoint_Task ()
{
	/* Init Error Variable */
	RTOS_ERR err;

	while(1){
		 OSSemPend(&Speed_Setpoint_Semaphore,
					0u,
					OS_OPT_PEND_BLOCKING,
					DEF_NULL,
					&err);
		struct button_event* top = peek_button_event();
		// pend on Speed Setpoint Mutex
		OSMutexPend(&Speed_Setpoint_Mutex,
					0,
					OS_OPT_PEND_BLOCKING,
					DEF_NULL,
					&err);
		// access speed_setpoint_data
		if(top == NULL){
			speed_setpoint_data.current_speed = 0;
			speed_setpoint_data.number_increments = 0;
			speed_setpoint_data.number_decrements = 0;
			speed_setpoint_data.speed_violation = false;
		}
		else if(top->pushbutton == PUSHBUTTON_PB0){
			speed_setpoint_data.current_speed += 5;
			speed_setpoint_data.number_increments += 1;
			pop_button_event();
		}
		else if(top->pushbutton == PUSHBUTTON_PB1){
			if(speed_setpoint_data.current_speed != 0) speed_setpoint_data.current_speed -= 5;
			speed_setpoint_data.number_decrements += 1;
			pop_button_event();
		}
		// post on Speed Setpoint Mutex
		OSMutexPost(&Speed_Setpoint_Mutex,
					OS_OPT_POST_1,
					&err);
		OSFlagPost(&Vehicle_Monitor_Flags,
				   VEHICLE_MONITOR_FLAG_SPEED,
				   OS_OPT_POST_FLAG_SET,
				   &err);
	}

	//Check for Error
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
}


/**************************************************************************//**
 * @brief
 * Vehicle Direction Task
 *
 * @details
 * "Responsible for updating the Vehicle Direction data store upon any change
 * in vehicle direction. It is awakened periodically to sample the position
 * of the Capacitive Touch Slider. It notifies the Vehicle Monitor task of any
 * change in direction using an event flag."
 *
 *****************************************************************************/
static void Vehicle_Direction_Task ()
{
	/* Init Error Variable */
	RTOS_ERR err;

	while(1) {
		uint8_t direction = sample_touch();
		// pend on the vehicle direction mutex
		OSMutexPend(&Vehicle_Direction_Mutex,
					0,
					OS_OPT_PEND_BLOCKING,
					DEF_NULL,
					&err);
		// access vehicle_directon_data
		if(vehicle_direction_data.current_direction != direction){
			vehicle_direction_data.current_direction = direction;
			vehicle_direction_data.time_held = 0;
			if((vehicle_direction_data.current_direction == DIRECTION_RIGHT ||
				vehicle_direction_data.current_direction == DIRECTION_HARD_RIGHT ||
				vehicle_direction_data.current_direction == DIRECTION_STRAIGHT) &&
			   (vehicle_direction_data.current_direction == DIRECTION_LEFT ||
				vehicle_direction_data.current_direction == DIRECTION_HARD_LEFT))
				vehicle_direction_data.num_left_turns += 1;
			if((vehicle_direction_data.current_direction == DIRECTION_LEFT ||
				vehicle_direction_data.current_direction == DIRECTION_HARD_LEFT ||
				vehicle_direction_data.current_direction == DIRECTION_STRAIGHT) &&
			   (vehicle_direction_data.current_direction == DIRECTION_RIGHT ||
				vehicle_direction_data.current_direction == DIRECTION_HARD_RIGHT))
				vehicle_direction_data.num_right_turns += 1;
		}
		else{
			vehicle_direction_data.time_held += 100;
		}
		// post back to the mutex
		OSMutexPost(&Vehicle_Direction_Mutex,
					OS_OPT_POST_1,
					&err);
		// post to Vehicle Monitor Flag
		OSFlagPost(&Vehicle_Monitor_Flags,
				   VEHICLE_MONITOR_FLAG_DIRECTION,
				   OS_OPT_POST_FLAG_SET,
				   &err);
		//Check for Error
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
		OSTimeDly(100,
				  OS_OPT_TIME_DLY,
				  &err);
	}

	//Check for Error
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
}

/**************************************************************************//**
 * @brief
 * Vehicle Monitor Task
 *
 * @details
 * "Responsible for checking for speed violations and direction violations and
 * notifying the LED Output task of any change in LED state. It is awakened by
 * an event flag set by the Speed Setpoint task and the Vehicle Direction task.
 * It notifies the Led Output task of a change in LED state using an event flag."
 *
 *****************************************************************************/
static void Vehicle_Monitor_Task ()
{
	/* Init Error Variable */
	RTOS_ERR err;
	OS_FLAGS flags;

	while(1){
		flags = OSFlagPend(&Vehicle_Monitor_Flags,
			        	   VEHICLE_MONITOR_FLAG_ALL,
				           0u,
						   OS_OPT_PEND_FLAG_SET_ANY|
						   OS_OPT_PEND_BLOCKING,
		        		   DEF_NULL,
			        	   &err);
		// pend on vehicle direction mutex to get current direction
		OSMutexPend(&Vehicle_Direction_Mutex,
					0,
					OS_OPT_PEND_BLOCKING,
					DEF_NULL,
					&err);
		// pend on speed setpoint mutex
		OSMutexPend(&Speed_Setpoint_Mutex,
					0,
					OS_OPT_PEND_BLOCKING,
					DEF_NULL,
					&err);

		// access data to get current values
		uint32_t current_speed = speed_setpoint_data.current_speed;
		uint8_t current_direction = vehicle_direction_data.current_direction;
		uint32_t time_held = vehicle_direction_data.time_held;
		// check if there's any violations
		bool direction_violation = (current_direction != DIRECTION_STRAIGHT) &&
								   (time_held >= 1250);
		bool speed_violation = (current_speed > 75u) | ((current_speed > 45u) && (current_direction != DIRECTION_STRAIGHT));
		// toggle LEDs if they're not already reflecting violation state
		if(speed_violation != speed_setpoint_data.speed_violation){
			OSFlagPost(&LED_Output_Flags,
					   LED_OUTPUT_FLAG_LED0,
					   OS_OPT_POST_FLAG_SET,
					   &err);
			speed_setpoint_data.speed_violation = speed_violation;
		}
		if(direction_violation != vehicle_direction_data.direction_violation){
			OSFlagPost(&LED_Output_Flags,
					   LED_OUTPUT_FLAG_LED1,
					   OS_OPT_POST_FLAG_SET,
					   &err);
			vehicle_direction_data.direction_violation = direction_violation;
		}

		//post on the speed setpoint mutex
		OSMutexPost(&Speed_Setpoint_Mutex,
					OS_OPT_POST_1,
					&err);
		//post on the vehicle direction mutex
		OSMutexPost(&Vehicle_Direction_Mutex,
					OS_OPT_POST_1,
					&err);

		OSFlagPost(&Vehicle_Monitor_Flags,
				   flags,
				   OS_OPT_POST_FLAG_CLR,
				   &err);
		//Check for Error
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
	}

	//Check for Error
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
}

/**************************************************************************//**
 * @brief
 * LED Output Task
 *
 * @details
 * "Responsible for updating the state of each LED upon notification of a change
 * in the desired state from the event flag. It is awakened by the event flag
 * posted by the Vehicle Monitor task."
 *
 *****************************************************************************/
static void LED_Output_Task ()
{
	/* Init Error Variable */
	RTOS_ERR err;
	OS_FLAGS flags;

	while(1){
		// wait for LED status change indication
		flags = OSFlagPend(&LED_Output_Flags,
					   	   LED_OUTPUT_FLAG_ALL,
						   0,
						   OS_OPT_PEND_FLAG_SET_ANY |
						   OS_OPT_PEND_BLOCKING,
						   DEF_NULL,
						   &err);
		if(flags & LED_OUTPUT_FLAG_LED0) {
			if(GPIO_PinInGet(LED0_PORT, LED0_PIN))
				GPIO_PinOutClear(LED0_PORT, LED0_PIN);
			else if(!GPIO_PinInGet(LED0_PORT, LED0_PIN))
				GPIO_PinOutSet(LED0_PORT, LED0_PIN);
		}
		if(flags & LED_OUTPUT_FLAG_LED1) {
			if(GPIO_PinInGet(LED1_PORT, LED1_PIN))
				GPIO_PinOutClear(LED1_PORT, LED1_PIN);
			else if(!GPIO_PinInGet(LED1_PORT, LED1_PIN))
				GPIO_PinOutSet(LED1_PORT, LED1_PIN);
		}
		OSFlagPost(&LED_Output_Flags,
				   flags,
				   OS_OPT_POST_FLAG_CLR,
				   &err);
		//Check for Error
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
	}

	//Check for Error
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
}

/**************************************************************************//**
 * @brief
 * LCD Display Task
 *
 * @details
 * "Responsible for periodically updating the current speed and direction on the
 * LCD display."
 *
 *****************************************************************************/
static void LCD_Display_Task () {
	/* Init Error Variable */
	RTOS_ERR err;
	uint32_t last_speed = 1;
	uint8_t last_direction = DIRECTION_STRAIGHT;

	while(1){
		OSTimeDly(100,
				  OS_OPT_TIME_DLY,
				  &err);
		// pend on Speed Setpoint Mutex
		OSMutexPend(&Speed_Setpoint_Mutex,
					0,
					OS_OPT_PEND_BLOCKING,
					DEF_NULL,
					&err);
		// access speed setpoint
		uint32_t current_speed = speed_setpoint_data.current_speed;
		//post on the speed setpoint mutex
		OSMutexPost(&Speed_Setpoint_Mutex,
					OS_OPT_POST_1,
					&err);
		// pend on Vehicle Direction Mutex
		OSMutexPend(&Vehicle_Direction_Mutex,
					0,
					OS_OPT_PEND_BLOCKING,
					DEF_NULL,
					&err);
		uint8_t direction = vehicle_direction_data.current_direction;
		// post vehicle direction mutex
		OSMutexPost(&Vehicle_Direction_Mutex,
					OS_OPT_POST_1,
					&err);
		if(current_speed != last_speed || direction != last_direction){
			switch(direction){
				case DIRECTION_STRAIGHT:
					printf("\fSpeed: %d\nDirection: Straight\b", (int)current_speed);
					break;
				case DIRECTION_LEFT:
					printf("\fSpeed: %d\nDirection: Left\b", (int)current_speed);
					break;
				case DIRECTION_HARD_LEFT:
					printf("\fSpeed: %d\nDirection: Hard Left\b", (int)current_speed);
					break;
				case DIRECTION_RIGHT:
					printf("\fSpeed: %d\nDirection: Right\b", (int)current_speed);
					break;
				case DIRECTION_HARD_RIGHT:
					printf("\fSpeed: %d\nDirection: Hard Right\b", (int)current_speed);
					break;
				default:
					printf("error \n");
					break;
			}
			last_speed = current_speed;
			last_direction = direction;
		}
		//Check for Error
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
	}

	//Check for Error
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
}
