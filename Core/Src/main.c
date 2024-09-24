/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cordic.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "projectMain.h"

#include "AS5047D.h"
#include "debug_scope.h"
#include "mathutils.h"
#include "dshot.h"
#include "circBuffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Definitions of data related to this example */
  /* Definition of ADCx conversions data table size */
  /* Size of array set to ADC sequencer number of ranks converted,            */
  /* to have a rank in each array address.                                    */
  #define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)   3)
  #define VDDA_APPLI                       ((uint32_t) 3300)    /* Value of analog voltage supply Vdda (unit: mV) */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MINVOLTAGE (0)
#define MAXVOLTAGE (3300)
#define MINSPEED (0)
#define MAXSPEED (2000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Variables for ADC conversion data */
__IO uint32_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; /* ADC group regular conversion data */

uint16_t my_motor_value[4] = {2345, 0, 0, 0};

DebugCommand debugCommand = 0;
DebugScope_Handle_t debugData =
  {
        .sz = DEBUGSCOPESIZE,
        .curCh = 1,
        .id = 0,
        .startWriteFlag = false
    };
int32_t encoderAngle = 0;
float spiAngle = 0;

uint16_t ERRFL = 0;
uint16_t PROG = 0;
uint16_t DIAAGC = 0;
uint16_t ANGLEUNC = 0;

uint16_t NOP = 0;
uint16_t CORDICMAG = 0;
uint16_t ANGLECOM = 0;
uint16_t ZPOSM = 0;
uint16_t ZPOSL = 0;
uint16_t SETTINGS1 = 0;
uint16_t SETTINGS2 = 0;

int32_t timerData = 0;
float true_angle = 0.0f;

__IO uint32_t PHASE_Voltage = 0;        /* Value of voltage on GPIO pin (on which is mapped ADC channel) calculated from ADC conversion data (unit: mV) */
__IO uint32_t AVGSPEED_Voltage = 0;        /* Value of voltage on GPIO pin (on which is mapped ADC channel) calculated from ADC conversion data (unit: mV) */
__IO uint32_t AMPSPEED_Voltage = 0;        /* Value of voltage on GPIO pin (on which is mapped ADC channel) calculated from ADC conversion data (unit: mV) */

extern float frequencySERVO_1, frequencySERVO_2, frequencySERVO_3, frequencyMOTOR_MAIN, frequencyTHROTLE;
extern float widthSERVO_1, widthSERVO_2, widthSERVO_3, widthMOTOR_MAIN, widthTHROTLE;

float minFrequency = 100, maxFrequency = 500;

float minSERVO = 0.32187, maxSERVO = 0.88;
float minMOTOR = 0.4, maxMOTOR = 0.88;
float minTHROTLE = 0.4, maxTHROTLE = 0.88;
float servoAngle1 = 0.f/180.f*PI, servoAngle2 = 120.f/180.f*PI, servoAngle3 = 240.f/180.f*PI;
float servoR1 = 1, servoR2 = 1, servoR3 = 1;
float servo1Nominal = 0.47652, servo2Nominal = 0.47931, servo3Nominal = 0.47848;
static float sinS[4]={0,0,0,0};
static float cosS[4]={0,0,0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t VoltageToAVGSpeed(const uint16_t voltage);
uint16_t VoltageToAmpSpeed(const uint16_t voltage, const uint16_t curspeed);
uint16_t VoltageToPhase(const uint16_t voltage);
void servo2planeABCD(const float servo1, const float servo2, const float servo3, 
                      float *A, float *B, float *C, float *D);
uint8_t calculateFreqAndWidth(const circ_buf_t *riseData, const circ_buf_t *fallData, const float period, float *frequency, float *width);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DelayUS(uint32_t us) 
{
    for (uint32_t i = 0; i < us*180; i++)
    {
        __NOP();
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  cosS[1] = cosf(servoAngle1); cosS[2] = cosf(servoAngle2); cosS[3] = cosf(servoAngle3);
  sinS[1] = sinf(servoAngle1); sinS[2] = sinf(servoAngle2); sinS[3] = sinf(servoAngle3);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_CORDIC_Init();
  MX_TIM1_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

   /* Initiaize AS5047D */
  uint16_t nop,AGC;

  uint8_t errorFlag[20]={99, 99, 99, 99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99};
  errorFlag[0] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_NOP, &nop);
  errorFlag[1] = AS5047D_Get_AGC_Value(&AGC);
  errorFlag[2] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ERRFL, &ERRFL);
  errorFlag[3] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ERRFL, &ERRFL);
  errorFlag[4] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_PROG, &PROG);
  errorFlag[5] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ANGLEUNC, &ANGLEUNC);
  errorFlag[6] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_CORDICMAG, &CORDICMAG);
  errorFlag[7] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ANGLECOM, &ANGLECOM);
  errorFlag[8] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ZPOSM, &ZPOSM);
  errorFlag[9] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ZPOSL, &ZPOSL);
  errorFlag[10] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_SETTINGS1, &SETTINGS1);
  errorFlag[11] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_SETTINGS2, &SETTINGS2);

  errorFlag[12] = AS5047D_Get_True_Angle_Value(&true_angle);

  dshot_init(DSHOT600);

  for (int i = 0; i < 10; i++)
  {
    dshot_send(0+0); 
    HAL_Delay(1);
  }

  //ProjectMain();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  int32_t avgSpeed = 0;
  int32_t ampSpeed = 0;
  int32_t phase = 0;
  int32_t spiAngle32 = 0;
  uint16_t totalSpeed = 0;
  int32_t delta = 0;
  uint8_t debugRes = 0;
  float data[DEBUGSCOPENUMOFCH] = {0.0f, 0.0f};
  float servo1Command =0, servo2Command = 0, servo3Command = 0;
  float motorMainCommand = 0, throtleCommand = 0;
  
  DebugScopeStartWrite(&debugData);
  // HAL_ADC_Start_DMA(&hadc1, aADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE);
  HAL_TIM_Base_Start(&htim6);

  LL_TIM_EnableCounter(TIM1);
  LL_TIM_EnableCounter(TIM2);
  LL_TIM_EnableCounter(TIM3);
  LL_TIM_EnableCounter(TIM4);
   /*## Start PWM signal generation in DMA mode ############################*/ 
  while (1)
  {
    float period = 1.f/(TIMCLOCK/TIM2->PSC);;
    servo1Command = (widthSERVO_1-minSERVO)/(maxSERVO-minSERVO);//-servo1Nominal;
    servo2Command = (widthSERVO_2-minSERVO)/(maxSERVO-minSERVO);//-servo2Nominal;
    servo3Command = (widthSERVO_3-minSERVO)/(maxSERVO-minSERVO);//-servo3Nominal;

    motorMainCommand = (widthMOTOR_MAIN-minMOTOR)/(maxMOTOR-minMOTOR);
    motorMainCommand = min(motorMainCommand, 1);
    motorMainCommand = max(motorMainCommand, 0);

    throtleCommand = (widthTHROTLE-minMOTOR)/(maxTHROTLE-minTHROTLE);
    throtleCommand = min(throtleCommand, 1);
    throtleCommand = max(throtleCommand, 0);
    float A, B, C, D;
    servo2planeABCD(servo1Command, servo2Command, servo3Command, &A, &B, &C, &D);
    //float heading = atan2f(B, A);
    float heading = atan2_m(B, A);
    //float inclination = acos(C);
    float inclination = acos_nvidia(C);
    float collective = -D;
    
    if (motorMainCommand < 0.5)
    {
      totalSpeed = 0;
      dshot_send(&totalSpeed);
      HAL_Delay(1);
      continue;
    }
      
    avgSpeed = (collective-0.5)/(0.72-0.5)*2000;
    avgSpeed = (avgSpeed>1950)?2000:avgSpeed;
    avgSpeed = (avgSpeed<50)?0:avgSpeed;
    // if (AVGSPEED_Voltage > 50)
    //   avgSpeed = VoltageToAVGSpeed(AVGSPEED_Voltage);

    ampSpeed = inclination/0.40f*100;//
    ampSpeed = (ampSpeed > 80)?100:ampSpeed;
    ampSpeed = (ampSpeed < 5)?0:ampSpeed;
    ampSpeed = ampSpeed*avgSpeed*1/4/100;
    // avgSpeed = (avgSpeed<50)?0:avgSpeed;

    // if (AMPSPEED_Voltage > 50)
    //   ampSpeed = VoltageToAmpSpeed(AMPSPEED_Voltage, avgSpeed*3/4);
    
    phase = heading*180/3.14159;
    // if (PHASE_Voltage > 50)
    //   phase = VoltageToPhase(PHASE_Voltage);

    errorFlag[7] = AS5047D_Read(  AS5047_CS_GPIO_Port,   AS5047_CS_Pin, AS5047D_ANGLECOM, &ANGLECOM);
    spiAngle32 = ANGLECOM * 360 / 16384;
    errorFlag[15] = AS5047D_Get_True_Angle_Value(&spiAngle);
    if (errorFlag[7] != 0)
    {
      errorFlag[16] = AS5047D_Read(AS5047_CS_GPIO_Port, AS5047_CS_Pin, AS5047D_ERRFL, &ERRFL);
      continue;
    }

    delta = ampSpeed*sine_m(spiAngle32 + phase);
    totalSpeed = avgSpeed + delta;

    totalSpeed = min(totalSpeed, 2000);
    totalSpeed = max(totalSpeed, 0);

    dshot_send(&totalSpeed);

    data[0] = (float)spiAngle32;
    data[1] = (float)totalSpeed;
    data[2] = (float)delta;
    debugRes = DebugScopeInsertData(&debugData, data);
    if (debugRes == NO_MORE_PLACE_TO_WRITE)
    {
      DebugScopeStartWrite(&debugData);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t calculateFreqAndWidth(const circ_buf_t *riseData, const circ_buf_t *fallData, const float period, float *frequency, float *width)
{
  if (!circ_buf_is_full(riseData) || !circ_buf_is_full(fallData))
    return 0;

  scalarMeasurement_t riseDataLast, fallDataLast;
  circ_buf_last(riseData, &riseDataLast);
  circ_buf_last(fallData, &fallDataLast);

  scalarMeasurement_t riseDataFirst, fallDataFirst;
  circ_buf_first(riseData, &riseDataFirst);
  circ_buf_first(fallData, &fallDataFirst);

  float ticks = (riseDataFirst-riseDataLast)/(riseData->circBufferLen-1);
  *frequency = 1.f/(ticks*period);
  if (fallDataLast < riseDataLast)
    *width = 1.0f-(fallDataLast-riseDataLast)/ticks;
  else
    *width = (fallDataLast-riseDataLast)/ticks;


  return 1;
}

uint16_t VoltageToAVGSpeed(const uint16_t voltage)
{
  return (uint16_t) ((uint32_t)(voltage - MINVOLTAGE) * (MAXSPEED - MINSPEED)/(MAXVOLTAGE - MINVOLTAGE) + MINSPEED);
}

uint16_t VoltageToAmpSpeed(const uint16_t voltage, const uint16_t curspeed)
{
  return (uint16_t)((uint32_t)(voltage - MINVOLTAGE)* curspeed/(MAXVOLTAGE - MINVOLTAGE) );
}

uint16_t VoltageToPhase(const uint16_t voltage)
{
  return (uint16_t)((uint32_t)(voltage - MINVOLTAGE)* 360 /(MAXVOLTAGE - MINVOLTAGE)) ;
}

void servo2planeABCD(const float servo1, const float servo2, const float servo3, 
                      float *A, float *B, float *C, float *D)
{
  Point p1={.x=servoR1*cosS[1], .y=servoR1*sinS[1], .z=servo1};
  Point p2={.x=servoR2*cosS[2], .y=servoR2*sinS[2], .z=servo2};
  Point p3={.x=servoR3*cosS[3], .y=servoR3*sinS[3], .z=servo3};

  Vector v1={.x=p2.x-p1.x, .y=p2.y-p1.y, .z=p2.z-p1.z};
  Vector v2={.x=p3.x-p1.x, .y=p3.y-p1.y, .z=p3.z-p1.z};

  Vector n={.x=v1.y*v2.z-v1.z*v2.y, .y=v1.z*v2.x-v1.x*v2.z, .z=v1.x*v2.y-v1.y*v2.x};
  float norminv = 1./sqrt(n.x*n.x+n.y*n.y+n.z*n.z);
  n.x = n.x*norminv;
  n.y = n.y*norminv;
  n.z = n.z*norminv;
  *D = -(n.x*p1.x+n.y*p1.y+n.z*p1.z);
  *A = n.x;
  *B = n.y;
  *C = n.z;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
