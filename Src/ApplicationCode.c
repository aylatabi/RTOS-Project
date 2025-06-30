/*
 * ApplicationCode.c
 *
 *  Created on: April 29th, 2025
 *      Author: Ayla Tabi
 */
#include "ApplicationCode.h"

#include <math.h>

uint16_t total_charge = 50;

osMutexId_t time_stamp_mutex_id;
const osMutexAttr_t time_stamp_Mutex_attr = {"Time_Stamp_Mutex", 0,
											 NULL, 0U};

osMutexId_t bttn_time_stamp_mutex_id;
const osMutexAttr_t bttn_time_stamp_Mutex_attr = {"Button_Time_Stamp_Mutex", 0,
												  NULL, 0U};

osMutexId_t rg_mutex_id;
const osMutexAttr_t rg_Mutex_attr = {"rg_Mutex", 0,
									 NULL, 0U};

volatile uint64_t time_stamp = 0;
volatile uint64_t bttn_current_time_stamp = 0;
float button_saved_time_stamp = 0;

static osSemaphoreId_t gyro_semaphore_id;
static osEventFlagsId_t gpio_evt_id;

volatile float angle = 0;
volatile float velocity = 0;
volatile float position = 160;

osMessageQueueId_t physics_message_id;
osMessageQueueId_t lcd_message_id;
osMessageQueueId_t rg_message_id;

float x_velocity_rg = 0;
float y_velocity_rg = 0;

static osSemaphoreId_t satchel_semaphore;

float delta = 0;

osTimerId_t timer_id;
StaticTimer_t static_timer;
const osTimerAttr_t timer_attr =
	{.name = "my_timer",
	 .cb_mem = &static_timer,
	 .cb_size = sizeof(static_timer)};

osThreadId_t satchel_thread_id;
StaticTask_t satchel_thread_t;
uint32_t satchel_stack[512];
const osThreadAttr_t satchel_thread_attr = {.name = "satchel_thread",
											.stack_mem = &satchel_stack[0],
											.stack_size = sizeof(satchel_stack),
											.cb_mem = &satchel_thread_t,
											.cb_size = sizeof(satchel_thread_t),
											.priority = osPriorityNormal};

osThreadId_t button_thread_id;
StaticTask_t button_thread_t;
uint32_t button_stack[512];
const osThreadAttr_t button_thread_attr =
	{.name = "button_thread",
	 .stack_mem = &button_stack[0],
	 .stack_size = sizeof(button_stack),
	 .cb_mem = &button_thread_t,
	 .cb_size = sizeof(button_thread_t),
	 .priority = osPriorityNormal};

osThreadId_t physics_thread_id;
StaticTask_t physics_thread_t;
uint32_t physics_stack[512];
const osThreadAttr_t physics_thread_attr =
	{.name = "physics_thread",
	 .stack_mem = &physics_stack[0],
	 .stack_size = sizeof(physics_stack),
	 .cb_mem = &physics_thread_t,
	 .cb_size = sizeof(physics_thread_t),
	 .priority = osPriorityNormal};

osThreadId_t rail_gun_thread_id;
StaticTask_t rail_gun_thread_t;
uint32_t rail_gun_stack[512];
const osThreadAttr_t rail_gun_thread_attr =
	{.name = "rail_gun_thread",
	 .stack_mem = &rail_gun_stack[0],
	 .stack_size = sizeof(rail_gun_stack),
	 .cb_mem = &rail_gun_thread_t,
	 .cb_size = sizeof(rail_gun_thread_t),
	 .priority = osPriorityNormal};

osThreadId_t lcd_display_thread_id;
StaticTask_t lcd_display_thread_t;
uint32_t lcd_display_stack[512];
const osThreadAttr_t lcd_display_thread_attr =
	{.name = "lcd_display_thread",
	 .stack_mem = &lcd_display_stack[0],
	 .stack_size = sizeof(lcd_display_stack),
	 .cb_mem = &lcd_display_thread_t,
	 .cb_size = sizeof(lcd_display_thread_t),
	 .priority = osPriorityNormal1};

osThreadId_t gyro_thread_id;
StaticTask_t gyro_thread_t;
uint32_t gyro_stack[512];
const osThreadAttr_t gyro_thread_attr = {.name = "gyro_thread", .stack_mem = &gyro_stack[0], .stack_size = sizeof(gyro_stack), .cb_mem = &gyro_thread_t, .cb_size = sizeof(gyro_thread_t), .priority = osPriorityBelowNormal};

/***********************************************
 *  Releases the semaphore every 100 ms	to the gyro thread
 ***********************************************/
void Timer_Callback(void *arg) {
	int32_t argument = (int32_t)arg;
	UNUSED(argument);

	if (osSemaphoreGetCount(gyro_semaphore_id) == 0) {
		osSemaphoreRelease(gyro_semaphore_id);
	}
}

/***********************************************
 *  This thread is woken up from the IRQ Handler via a event
 * 	Determines whether it is a double tap or button press
 ***********************************************/
__NO_RETURN void button_thread(void *argument) {
	UNUSED(argument);

	PHYSICS_MSGQUEUE_OBJ_t msg;
	LCD_MSGQUEUE_OBJ_t lcd_msg;

	const uint32_t double_tap_time = 1500;
	int is_double_tap = 0;

	uint32_t on_off_time = 0;
	uint32_t on_on_time = 0;
	uint32_t button_on_time_stamp = 0;
	const uint32_t minimum_rg_time = 400;

	for (;;) {
		osEventFlagsWait(gpio_evt_id, BUTTON_EVENT, osFlagsWaitAny, osWaitForever);
		button_state_t button_state = user_button_state();

		if (button_state == BUTTON_ON) {
			on_on_time = HAL_GetTick() - button_on_time_stamp;
			button_on_time_stamp = HAL_GetTick();
		}

		if (button_state == BUTTON_OFF) {
			on_off_time = HAL_GetTick() - button_on_time_stamp;

			if (on_off_time > minimum_rg_time)	//If single tap then charge rail gun
			{
				delta = ((float)HAL_GetTick() - button_on_time_stamp) * 0.001f;
				msg.msg_type = BUTTON;
				msg.delta = delta;
				osMessageQueuePut(physics_message_id, &msg, 0U, osWaitForever);
			} else	//could be double tap
			{
				is_double_tap = 0;
				if (on_on_time < double_tap_time) {
					is_double_tap = 1;
				}

				if (is_double_tap) {
					lcd_msg.msg_type = FORCE_SHEILD;
					lcd_msg.force_shield = 1;
					osMessageQueuePut(lcd_message_id, &lcd_msg, 0U, osWaitForever);
				}
			}
		}
	}
}

/***********************************************
 *  Sends data to lcd thread
 ***********************************************/
__NO_RETURN void physics_thread(void *argument) {
	UNUSED(argument);

	PHYSICS_MSGQUEUE_OBJ_t phys_message_obj;
	LCD_MSGQUEUE_OBJ_t lcd_message_obj;
	RG_MSGQUEUE_OBJ_t rg_message_obj;

	osStatus_t ret;
	float platform_position = 160;
	for (;;) {
		ret = osMessageQueueGet(physics_message_id, &phys_message_obj, NULL, osWaitForever);
		if (ret != osOK) {
			for (;;)
				;
		}
		switch (phys_message_obj.msg_type) {
			case BUTTON:

				rg_message_obj.position = platform_position;
				rg_message_obj.delta = phys_message_obj.delta;
				ret = osMessageQueuePut(rg_message_id, &rg_message_obj, 0U, osWaitForever);
				if (ret != osOK) {
					for (;;)
						;
				}
				break;
			case GYRO:
				int16_t gyro_rotation = phys_message_obj.rotation;

				angle += ((float)gyro_rotation / 1000.0f);	// * (M_PI / 180.0f) * 0.1; //mdps to s
															//				float acceleration = sinf(angle) * GRAVITY;
															//				velocity += (acceleration * 0.1f);
				position += angle * 0.1f;

				if (position < 55) {
					position = 55;
					angle = -angle;
				}
				if (position > 285) {
					position = 285;
					angle = -angle;
				}

				if ((angle > 80 || angle < -80) && (position == 285 || position == 55)) {
					lcd_message_obj.platform_damage = 1;
				} else {
					lcd_message_obj.platform_damage = 0;
				}
				platform_position = position;
				lcd_message_obj.msg_type = GYRO;
				lcd_message_obj.gyro_position = (uint16_t)position;
				ret = osMessageQueuePut(lcd_message_id, &lcd_message_obj, 0U, osWaitForever);
				if (ret != osOK) {
					for (;;)
						;
				}
				break;

			default:
				for (;;)
					;
				break;
		}
	}
}

/***********************************************
 *  Sends rail gun info to lcd thread
 ***********************************************/
__NO_RETURN void rail_gun_thread(void *argument) {
	UNUSED(argument);

	RG_MSGQUEUE_OBJ_t message_obj;
	LCD_MSGQUEUE_OBJ_t lcd_mssg;
	float delta_l = 0;
	float curr_time_stamp = 0;
	float rg_velocity_x = 0;
	float rg_velocity_y = 0;
	float curr_position = 160.0f;

	float curr_total_charge = 50;
	float charged_energy = 0;
	uint16_t damage = 50;

	osStatus_t ret;

	for (;;) {
		ret = osMessageQueueGet(rg_message_id, &message_obj, NULL, osWaitForever);
		if (ret != osOK) {
			for (;;)
				;
		}

		delta_l = message_obj.delta;
		curr_position = message_obj.position;

		charged_energy = GENERATOR_POWER * delta_l;

		float button_velocity = sqrt((2 * charged_energy) / (50 * 1000)) * 2.0;

		rg_velocity_x = -button_velocity * cosf(.8);
		rg_velocity_y = button_velocity * sinf(.8);

		curr_time_stamp = 0;

		float converted_charged_energy = charged_energy * 0.000001f * 2.0f;
		if (converted_charged_energy > 50) converted_charged_energy = 50;

		osMutexAcquire(time_stamp_mutex_id, osWaitForever);
		curr_total_charge = total_charge;
		osMutexRelease(time_stamp_mutex_id);

		if (curr_total_charge > 30 && converted_charged_energy < curr_total_charge) {
			while (1) {
				float time = (float)(curr_time_stamp)*0.1f;
				float rg_x_position = curr_position + rg_velocity_x * time;
				float rg_y_position = 50.0f + rg_velocity_y * time - 0.5f * 9.8f * time * time;

				lcd_mssg.msg_type = RAIL_GUN;
				lcd_mssg.x_position = rg_x_position;
				lcd_mssg.damage = damage;
				lcd_mssg.y_position = rg_y_position;
				lcd_mssg.charged_energy = converted_charged_energy;
				lcd_mssg.rail_gun_done = 0;
				ret = osMessageQueuePut(lcd_message_id, &lcd_mssg, 0U, 0U);
				if (ret != osOK) {
					for (;;)
						;
				}

				curr_time_stamp += 2;

				if ((rg_x_position < 30) || (rg_y_position < 30))  //Rail Gun Bullet hit bounds
				{
					lcd_mssg.msg_type = RAIL_GUN;
					lcd_mssg.x_position = 0;
					lcd_mssg.y_position = 0;
					lcd_mssg.rail_gun_done = 1;
					if (rg_x_position < 30) {
						damage -= 25;
					}
					lcd_mssg.damage = damage;

					ret = osMessageQueuePut(lcd_message_id, &lcd_mssg, 0U, 0U);
					if (ret != osOK) {
						for (;;)
							;
					}

					if (rg_x_position < 30) {
						lcd_mssg.msg_type = HIT_CASTLE;
						lcd_mssg.hit_castle_x_position = rg_x_position;
						lcd_mssg.hit_castle_y_position = rg_y_position;
						lcd_mssg.hit_castle_change = 0;
						ret = osMessageQueuePut(lcd_message_id, &lcd_mssg, 0U, 0U);
						if (ret != osOK) {
							for (;;)
								;
						}

						lcd_mssg.hit_castle_change = 8;	 //Hit castle animation
						osMessageQueuePut(lcd_message_id, &lcd_mssg, 0U, 0U);

						lcd_mssg.hit_castle_change = 10;
						osMessageQueuePut(lcd_message_id, &lcd_mssg, 0U, 0U);
					}

					curr_time_stamp = 0;
					break;
				}

				ret = osDelay(200);
				if (ret != osOK) {
					for (;;)
						;
				}
			}
		}
	}
}

/***********************************************
 *  Woken up from timer callback, sends info to physics
 ***********************************************/
__NO_RETURN void gyro_thread(void *argument) {
	// ...
	UNUSED(argument);
	Gyro_Init();
	PHYSICS_MSGQUEUE_OBJ_t msg;

	osStatus_t ret;
	for (;;) {
		int16_t rotation = Gyro_Get_Velocity();
		if (rotation > -500 && rotation < 500) rotation = 0;  //deadzone mitigation

		msg.msg_type = GYRO;
		msg.rotation = rotation;
		ret = osMessageQueuePut(physics_message_id, &msg, 0U, 0U);
		if (ret != osOK) {
			for (;;)
				;
		}

		osDelay(100);
	}
}

/***********************************************
 *  Displays all information from railgun, satchels, platform...
 ***********************************************/
__NO_RETURN void lcd_display_thread(void *argument) {
	// ...
	UNUSED(argument);

	LCD_MSGQUEUE_OBJ_t lcd_message_obj;

	uint16_t g_position = 160;
	float rail_x = 0;
	float rail_y = 0;

	static uint16_t last_g_position = 160;
	int force_shield_activate = 20;

	float hit_castle_x_position = 0;
	float hit_castle_y_position = 0;
	int hit_castle_flag = 0;
	float hit_castle_change = 0;
	uint16_t damage = 50;

	uint16_t charge = 50;

	float satchel_x_position = 40;
	float satchel_y_position = 180;

	float end_position = 0;
	float satchel_velocity = 0;
	float starting_position = 40;
	float total_time = 6.06f;
	unsigned int seed = 1;
	int satchel_flag = 0;
	uint32_t time_stamp = 0;

	int platform_damage = 50;
	int rail_gun_done = 0;

	int force_shield = 0;

	for (;;) {
		osMessageQueueGet(lcd_message_id, &lcd_message_obj, NULL, osWaitForever);

		switch (lcd_message_obj.msg_type) {
			case RAIL_GUN:
				//180 ,40
				if (lcd_message_obj.x_position != rail_x || lcd_message_obj.y_position != rail_y) {
					rail_x = lcd_message_obj.x_position;
					rail_y = lcd_message_obj.y_position;
					damage = lcd_message_obj.damage;
					osMutexAcquire(time_stamp_mutex_id, osWaitForever);
					total_charge = total_charge - lcd_message_obj.charged_energy;
					osMutexRelease(time_stamp_mutex_id);
					rail_gun_done = lcd_message_obj.rail_gun_done;
				}
				break;
			case HIT_CASTLE:
				hit_castle_flag = 1;
				hit_castle_x_position = lcd_message_obj.hit_castle_x_position;
				hit_castle_y_position = lcd_message_obj.hit_castle_y_position;
				hit_castle_change = lcd_message_obj.hit_castle_change;
				break;
			case GYRO:

				if (lcd_message_obj.gyro_position != last_g_position) {
					g_position = lcd_message_obj.gyro_position;
					last_g_position = g_position;

					if (lcd_message_obj.platform_damage == 1) {
						platform_damage -= 10;
					}
				}

				break;
			case FORCE_SHEILD:

				osMutexAcquire(time_stamp_mutex_id, osWaitForever);
				charge = total_charge;
				osMutexRelease(time_stamp_mutex_id);
				if (force_shield == 0 && charge > 30) {
					force_shield = 1;
					osMutexAcquire(time_stamp_mutex_id, osWaitForever);
					total_charge = total_charge - force_shield_activate;
					osMutexRelease(time_stamp_mutex_id);
				} else if (force_shield == 1) {
					force_shield = 0;
				}

				break;
			default:
				for (;;)
					;
				break;
		}

		if (satchel_flag == 0) {
			end_position = 50.0f + ((float)rand_r(&seed) / (float)RAND_MAX) * 270.0f;
			satchel_velocity = (end_position - starting_position) / total_time;
			satchel_flag = 1;
		}

		satchel_y_position = 180.0f - 0.5f * 9.8f * ((float)time_stamp * 0.1f) * ((float)time_stamp * 0.1f);
		satchel_x_position = satchel_velocity * ((float)time_stamp * 0.1f) + 40.0f;

		if (force_shield == 0 && (satchel_y_position < 30 && satchel_y_position > 20) && (satchel_x_position < (last_g_position + 25) && satchel_x_position > (last_g_position - 25))) {
			platform_damage -= 10;
		}
		if ((rail_y < 30 && rail_y > 20) && (rail_x < (last_g_position + 25) && rail_x > (last_g_position - 25))) {
			platform_damage -= 10;
		}
		LCD_Clear(0, LCD_COLOR_WHITE);
		LCD_SetTextColor(LCD_COLOR_BLACK);
		LCD_SetFont(&Font16x24);
		LCD_Draw_platform(last_g_position);

		if ((rail_x != 0) && (rail_y != 0)) {
			LCD_Draw_Circle_Fill(rail_y, rail_x, 5, LCD_COLOR_BLACK);
		}

		if (satchel_x_position < 300 && satchel_y_position > 30) {
			if (force_shield == 0) {
				LCD_Draw_Circle_Fill(satchel_y_position, satchel_x_position, 5, 0xca60);
			}
			if (force_shield == 1 &&
				satchel_y_position <= 65 &&						 // Satchel is within shield height
				satchel_x_position >= (last_g_position - 25) &&	 // inside left side
				satchel_x_position <= (last_g_position + 25)) {
				//				LCD_Draw_Circle_Fill(satchel_y_position, satchel_x_position, 5, 0xca60);
				satchel_flag = 0;
				time_stamp = 0;
			}
		}

		if (damage != 0) {
			LCD_Draw_Castle_Damage(damage);
		} else {
			for (int k = 0; k < 40; k += 5) {
				LCD_Clear(0, LCD_COLOR_WHITE);
				LCD_SetTextColor(LCD_COLOR_BLACK);
				LCD_SetFont(&Font16x24);
				LCD_Draw_Castle();
				LCD_Draw_Prisoner(k);

				if (k >= 10) {
					LCD_Draw_Prisoner(k - 10);
				}
				osDelay(500);
			}

			LCD_DisplayString(70, 140, "Victory!");

			for (;;)
				;
		}

		LCD_Draw_Castle();

		if (platform_damage != 0) {
			LCD_Draw_Platform_Damage(platform_damage);	//platform_damage
		} else {
			LCD_Clear(0, LCD_COLOR_WHITE);
			LCD_SetTextColor(LCD_COLOR_BLACK);
			LCD_SetFont(&Font16x24);
			LCD_DisplayString(70, 140, "Defeated");

			for (;;)
				;
		}

		LCD_Draw_Capasitor_Charge(charge);

		if (hit_castle_flag == 1) {
			chunk1(hit_castle_x_position, hit_castle_y_position, hit_castle_change);
			chunk2(hit_castle_x_position, hit_castle_y_position, hit_castle_change);
			hit_castle_flag = 0;
		}

		osMutexAcquire(time_stamp_mutex_id, osWaitForever);
		charge = total_charge;
		osMutexRelease(time_stamp_mutex_id);

		if (force_shield == 1) {
			LCD_Draw_Force_Shield(last_g_position);
		}

		if (force_shield == 1 && (charge - 5) > 0) {
			osMutexAcquire(time_stamp_mutex_id, osWaitForever);
			total_charge -= 1;
			osMutexRelease(time_stamp_mutex_id);
		} else {
			force_shield = 0;
		}

		if (((charge + time_stamp) < 50) && (rail_gun_done == 1 || force_shield == 0)) {
			osMutexAcquire(time_stamp_mutex_id, osWaitForever);
			total_charge += 1;
			osMutexRelease(time_stamp_mutex_id);
		}

		if (satchel_y_position < 20) {
			satchel_flag = 0;
			time_stamp = 0;
		}

		time_stamp += 1;

		//		osDelay(100);
	}
}
void ApplicationInit(void) {
	LTCD__Init();
	LTCD_Layer_Init(0);

	rg_message_id = osMessageQueueNew(64, sizeof(RG_MSGQUEUE_OBJ_t), NULL);
	lcd_message_id = osMessageQueueNew(64, sizeof(LCD_MSGQUEUE_OBJ_t), NULL);
	physics_message_id = osMessageQueueNew(64, sizeof(PHYSICS_MSGQUEUE_OBJ_t), NULL);

	button_thread_id = osThreadNew(button_thread, NULL, &button_thread_attr);
	rail_gun_thread_id = osThreadNew(rail_gun_thread, NULL, &rail_gun_thread_attr);
	lcd_display_thread_id = osThreadNew(lcd_display_thread, NULL, &lcd_display_thread_attr);
	physics_thread_id = osThreadNew(physics_thread, NULL, &physics_thread_attr);
	gyro_thread_id = osThreadNew(gyro_thread, NULL, &gyro_thread_attr);
	//	satchel_thread_id = osThreadNew(satchel_thread, NULL, &satchel_thread_attr);

	gyro_semaphore_id = osSemaphoreNew(1U, 1U, NULL);

	satchel_semaphore = osSemaphoreNew(1U, 1U, NULL);

	//	physics_message_id = osMessageQueueNew(1, sizeof(PHYSICS_MSGQUEUE_OBJ_t), NULL);

	time_stamp_mutex_id = osMutexNew(&time_stamp_Mutex_attr);
	timer_id = osTimerNew(Timer_Callback, osTimerPeriodic, (void *)0, &timer_attr);	 //
	osTimerStart(timer_id, 100);

	gpio_evt_id = osEventFlagsNew(NULL);
}

void RunDemoForLCD(void) {
	LCD_Clear(0, LCD_COLOR_WHITE);
	QuickDemo();
}
/**
 * @brief Gets the current state of the button
 * @returns button state
 */
button_state_t user_button_state(void) {
	GPIO_PinState pin_state = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
	if (pin_state == GPIO_PIN_RESET)
		return BUTTON_OFF;
	else
		return BUTTON_ON;
}
/**
 * @brief Sets the event for the speed thread
 */
void EXTI0_IRQHandler(void) {
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_EXTI_ClearPending(EXTI_GPIOA, EXTI_TRIGGER_RISING_FALLING);
	uint32_t ret = osEventFlagsSet(gpio_evt_id, BUTTON_EVENT);
	if (ret != BUTTON_EVENT) {
		for (;;)
			;
	}
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}
