/*
 * unittests.c
 * 
 * Brief:
 * This file will contain the unit tests and a unit test start
 * function.
 *
 *  Created on: Jan 28, 2021
 *      Author: peski
 */

/**************************************************************************//**
 * @brief
 * Force Increase Check
 *
 * @details
 * Call the PB0 (increase force) button and ensure:
 * 	- PWM value is changed on LED
 * 	- force value is changed in PendulumStateData
 *
 *****************************************************************************/
void forceIncreaseTest(){
}

/**************************************************************************//**
 * @brief
 * Force Decrease Check
 *
 * @details
 * Call the PB1 (decrease force) button and ensure:
 * 	- PWM value is changed on LED
 * 	- force value is changed in PendulumStateData
 *
 *****************************************************************************/
void forceDecreaseTest(){
}

/**************************************************************************//**
 * @brief
 * Direction Change Test
 *
 * @details
 * Test each direction (right, mid right, mid left, left) by adjusting 
 * Adjustment Data and make sure changes to PendulumStateData are made properly.
 *
 *****************************************************************************/
void directionChangeTest(){
}

/**************************************************************************//**
 * @brief
 * LED1 Test
 *
 * @details
 * Put the theta value to >{pi}/2 and ensure that LED1 is properly lit. 
 *
 *****************************************************************************/
void directionChangeTest(){
}

/**************************************************************************//**
 * @brief
 * Upper Limit Force Test
 *
 * @details
 * Increase force by calling pushbutton events until the value should be 
 * above the upper limit. Ensure it doesn't go above the specified limit. 
 *
 *****************************************************************************/
void upperLimitForceTest(){
}

/**************************************************************************//**
 * @brief
 * Lower Limit Force Test
 *
 * @details
 * Decrease the force using pushbutton event until it should be below 0. Ensure
 * that the force value does not go below zero.
 *
 *****************************************************************************/
void lowerLimitForceTest(){
}

/**************************************************************************//**
 * @brief
 * Display Update Frequency Test
 *
 * @details
 * Make a change to the inputs, causing a change in the display, and ensure that
 * the display has changed in the proper amount of time. This is specified by
 * the {tau}_display value
 *
 *****************************************************************************/
void displayUpdateFrequencyTest(){
}


/**************************************************************************//**
 * @brief
 * Physics Task Test
 *
 * @details
 * Set the values to the adjustments to ensure that we know how the pendulum 
 * state data should be after the physics task runs. Ensure that the correct
 * chagnes are run in the correct amount of time. 
 *
 *****************************************************************************/
void physicsTaskTest(){
}

/**************************************************************************//**
 * @brief
 * Garbage In, Garbage Out Test
 *
 * @details
 * Enter malformed (bad) data into the adjustments and ensure that the Physics
 * Task behaves properly and doesn't make the bad changes. 
 *
 *****************************************************************************/
void GIGOtest(){
}

/**************************************************************************//**
 * @brief
 * Pendulum State Data Mutex Test
 *
 * @details
 * Pend on the mutex, ensure that no other dependent tasks make changes by
 * delaying by a greater value than their specified frequency. 
 *
 *****************************************************************************/
void pendulumStateDataMutexTest(){
}

/**************************************************************************//**
 * @brief
 * Adjustment Data Mutex Test
 *
 * @details
 * Pend on the mutex, ensure that no other dependent tasks make changes by
 * delaying by a greater value than their specified frequency. 
 *
 *****************************************************************************/
void adjustmentDataMutexTest(){
}
