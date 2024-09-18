/*
 * dshot.h
 *
 *
 *  Created on: 2021. 1. 27.
 *      Author: mokhwasomssi
 * https://github.com/mokhwasomssi/stm32_hal_dshot/
 *
 */

#include "dshot.h"

#define MHZ_TO_HZ(x) 			((x) * 1000000)

#define DSHOT600_HZ     		MHZ_TO_HZ(12)
#define DSHOT300_HZ     		MHZ_TO_HZ(6)
#define DSHOT150_HZ     		MHZ_TO_HZ(3)

#define DSHOT1200_PERIOD        (0.83e-6f)       // 0.83us
#define DSHOT600_PERIOD         (1.67e-6f)       // 1.67us
#define DSHOT300_PERIOD         (3.33e-6f)       // 3.33us
#define DSHOT150_PERIOD         (6.67e-6f)       // 6.67us


extern TIM_HandleTypeDef MOTOR1_TIM;

/* Variables */
static uint32_t motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
// static uint32_t motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
// static uint32_t motor3_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
// static uint32_t motor4_dmabuffer[DSHOT_DMA_BUFFER_SIZE];


/* Static functions */
// dshot init
static void dshot_choose_type(dshot_type_e dshot_type);
static void dshot_set_timer(dshot_type_e dshot_type);
static void dshot_start_pwm();

// dshot write
static uint16_t dshot_prepare_packet(uint16_t value);
static void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value);
static void dshot_prepare_dmabuffer_all();

static uint16_t motorBitLength = 0;
static uint16_t motorBit0 = 0;
static uint16_t motorBit1 = 0;

/* Functions */
void dshot_init(dshot_type_e dshot_type)
{
  dshot_choose_type(DSHOT600);
  dshot_set_timer(dshot_type);
}

void dshot_send(uint16_t* motor_value)
{
  dshot_prepare_dmabuffer_all(motor_value);
  dshot_start_pwm();
}


/* Static functions */
static void dshot_choose_type(dshot_type_e dshot_type)
{
  switch (dshot_type)
  {
    case(DSHOT1200):
        motorBitLength = (uint16_t)(roundf(TIMER_CLOCK * DSHOT1200_PERIOD));
        break;
    case(DSHOT600):
        motorBitLength = (uint16_t)(roundf(TIMER_CLOCK * DSHOT600_PERIOD));
        break;
    case(DSHOT300):
        motorBitLength = (uint16_t)(roundf(TIMER_CLOCK * DSHOT300_PERIOD));
        break;
    case(DSHOT150):
        motorBitLength = (uint16_t)(roundf(TIMER_CLOCK * DSHOT150_PERIOD));
        
  }
  motorBit0 = motorBitLength / 3;
  motorBit1 = motorBitLength * 2 / 3;
}

static void dshot_set_timer(dshot_type_e dshot_type)
{
  // uint16_t dshot_prescaler;

   // motor1
  __HAL_TIM_SET_PRESCALER(&MOTOR1_TIM, 0);
  __HAL_TIM_SET_AUTORELOAD(&MOTOR1_TIM, motorBitLength);

//   // motor2
//   __HAL_TIM_SET_PRESCALER(MOTOR_2_TIM, dshot_prescaler);
//   __HAL_TIM_SET_AUTORELOAD(MOTOR_2_TIM, MOTOR_BITLENGTH);

//   // motor3
//   __HAL_TIM_SET_PRESCALER(MOTOR_3_TIM, dshot_prescaler);
//   __HAL_TIM_SET_AUTORELOAD(MOTOR_3_TIM, MOTOR_BITLENGTH);

//   // motor4
//   __HAL_TIM_SET_PRESCALER(MOTOR_4_TIM, dshot_prescaler);
//   __HAL_TIM_SET_AUTORELOAD(MOTOR_4_TIM, MOTOR_BITLENGTH);
}

static void dshot_start_pwm()
{
  // Start the timer channel now.
    // Enabling/disabling DMA request can restart a new cycle without PWM start/stop.
  if (HAL_TIM_PWM_Start_DMA(&MOTOR1_TIM, MOTOR_1_TIM_CHANNEL, (uint32_t)motor1_dmabuffer, DSHOT_DMA_BUFFER_SIZE))
  {
    /* Starting Error */
    Error_Handler();
  }
//   HAL_TIM_PWM_Start(MOTOR_2_TIM, MOTOR_2_TIM_CHANNEL);
//   HAL_TIM_PWM_Start(MOTOR_3_TIM, MOTOR_3_TIM_CHANNEL);
//   HAL_TIM_PWM_Start(MOTOR_4_TIM, MOTOR_4_TIM_CHANNEL);
}

static uint16_t dshot_prepare_packet(uint16_t value)
{
  uint16_t packet;
  bool dshot_telemetry = false;

  packet = (value << 1) | (dshot_telemetry ? 1 : 0);

  // compute checksum
  unsigned csum = 0;
  unsigned csum_data = packet;

  for(int i = 0; i < 3; i++)
  {
        csum ^=  csum_data; // xor data by nibbles
        csum_data >>= 4;
  }

  csum &= 0xf;
  packet = (packet << 4) | csum;

  return packet;
}

// Convert 16 bits packet to 16 pwm signal
static void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value)
{
  uint16_t packet;
  packet = dshot_prepare_packet(value);

  for(int i = 0; i < 16; i++)
  {
    motor_dmabuffer[i] = (packet & 0x8000) ? motorBit1 : motorBit0;
    packet <<= 1;
  }

  motor_dmabuffer[16] = 0;
  motor_dmabuffer[17] = 0;
}

static void dshot_prepare_dmabuffer_all(uint16_t* motor_value)
{
  dshot_prepare_dmabuffer(motor1_dmabuffer, motor_value[0]);
//   dshot_prepare_dmabuffer(motor2_dmabuffer, motor_value[1]);
//   dshot_prepare_dmabuffer(motor3_dmabuffer, motor_value[2]);
//   dshot_prepare_dmabuffer(motor4_dmabuffer, motor_value[3]);
}


