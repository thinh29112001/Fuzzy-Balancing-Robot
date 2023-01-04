/*
 * define.h
 *
 *  Created on: Nov 27, 2022
 *      Author: Lenovo
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#define TIM1_FREQ 500  //Hz
#define MAX_COUNT 5
#define SPEED_SCALE 0.5

#define UART_RX_BUFF_SIZE 20
#define PWM_CNT_PERIOD 1999

#define ROLL_OFFSET 2.0000f

#define K_IN0 7.3
#define K_IN1 110
#define K_OUT_FUZZY 2700



// #define K_IN0 7.3//10//15
// #define K_IN1 110//215//160//7000//1/5000//12
// #define K_OUT_FUZZY 2700//600//800
//#define KP1_MTR 7    //SP>1000
//#define KI1_MTR 8
//#define KD1_MTR 0

//#define KP2_MTR 4.5  //SP<1000
//#define KI2_MTR 0.5
//#define KD2_MTR 0.001

//#define KP_BLC 60//39.5//47
//#define KI_BLC 0//100//50
//#define KD_BLC 0.007//0.012//0.0162//0.001

#define MTR1_PWM_TIMER htim2
#define MTR1_PWM_CHANNEL TIM_CHANNEL_1
#define MTR1_DIR_GPIO_PORT GPIOC
#define MTR1_DIR_GPIO_PIN GPIO_PIN_14

#define MTR2_PWM_TIMER htim2
#define MTR2_PWM_CHANNEL TIM_CHANNEL_2
#define MTR2_DIR_GPIO_PORT GPIOD
#define MTR2_DIR_GPIO_PIN GPIO_PIN_0



#endif /* DEFINE_H_ */
