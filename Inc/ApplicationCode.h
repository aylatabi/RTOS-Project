/*
 * ApplicationCode.h
 *
 *  Created on: Nov 14, 2023
 *      Author: xcowa
 */

#ifndef INC_APPLICATIONCODE_H_
#define INC_APPLICATIONCODE_H_


#define BUTTON_PORT GPIOA
#define BUTTON_PIN GPIO_PIN_0

#define LED_PORT GPIOG
#define LED_GREEN GPIO_PIN_13
#define LED_RED GPIO_PIN_14


#define BUTTON_EVENT 0x00000001U
#define RG_EVENT 0x00000001U
#define GYRO_EVENT 0x00000002U
//#define



#define GENERATOR_POWER 20000 * 1000
#define SHOTMASS 50 * 1000
#define ELEVATION_ANGLE 0.8f
#define PLATFORM_MASS 100.0f
#define GRAVITY 9.8f
#define SENVAL 0.0008f
#define GYRO_SENSITIVITY 17.5f // mdps/LSB at 500dps scale
#define MAX_TILT_ANGLE (M_PI / 2.0f) // 90 degrees in radians

#define TILT_BOOST 3.0f
#define ANGLE_DEADZONE 0.03f //wfegr2 degrees
#define DAMPING 0.98f

#include "LCD_Driver.h"
#include "Gyro_Driver.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <time.h>


typedef enum
{
	GYRO_ROTATE_NONE,
	GYRO_ROTATE_CW_F,
	GYRO_ROTATE_CW_S,
	GYRO_ROTATE_CCW_S,
	GYRO_ROTATE_CCW_F
}GYRO_RotateState;


extern GYRO_RotateState state;

typedef enum
{
	BUTTON_ON,
	BUTTON_OFF
}button_state_t;

typedef struct
{
	uint16_t curr_speed;
	uint16_t speed_count;
}Speed_Data_t;

typedef enum
{
	LEFT,
	HARD_LEFT,
	RIGHT,
	HARD_RIGHT,
	STRAIGHT
}Direction;

typedef enum
{
	GYRO = 1,
	BUTTON,
	RAIL_GUN,
	HIT_CASTLE,
	SATCHEL,
	FORCE_SHEILD,
}message_type_t;

typedef struct
{
	message_type_t msg_type;
	float delta;
	int16_t rotation;
	float satchel_x_position;
	float satchel_y_position;
}PHYSICS_MSGQUEUE_OBJ_t;

typedef struct
{
	float position;
	float delta;
}RG_MSGQUEUE_OBJ_t;



typedef struct
{
	message_type_t msg_type;
	uint16_t gyro_position;
	uint16_t damage;
	float x_position;
	float y_position;
	float hit_castle_x_position;
	float hit_castle_y_position;
	float hit_castle_change;
	float charged_energy;
	float satchel_x_position;
	float satchel_y_position;
	int rail_gun_done;
	int platform_damage;
	int force_shield;
}LCD_MSGQUEUE_OBJ_t;

typedef struct
{
	Direction curr_direction;
	uint16_t num_turns;
}Direction_Data;

Direction gyro_rotation_state(void);
void ApplicationInit(void);

void RunDemoForLCD(void);
button_state_t user_button_state(void);

#endif /* INC_APPLICATIONCODE_H_ */
