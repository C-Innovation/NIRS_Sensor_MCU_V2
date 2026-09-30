/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <nirs_tflite_api.hpp>
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "analog_driver.h"
#include "aitflitehandle.h"
#include "usb_drd_fs.h"
#include "usbd_cdc_if.h"
#include "dac.h"
#include "tim.h"
#include "ring_buffer.h"
#include "protocol_base.h"
#include "data_structs.h"
#include "NirsDSP.h"
#include "semphr.h"
#include "nirs_contraction.h"
#include "nn/nirs_nn_runtime.h"
#include "nirs_tflite_api.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_RING_BUFFER_SIZE	(int)1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static nirs_nn_t g_nn;
static nirs_t    g_ref;           // опционально: считать оба и сравнивать
static const float k = 3.30f * 2.0f / 4095.0f;

// диагностика — выводите её по USB рядом с сырыми данными
volatile uint32_t g_ai_frames     = 0;   // обработано кадров
volatile uint32_t g_ai_nan_events = 0;   // сбросов состояния сети
volatile uint32_t g_ai_bad_invoke = 0;   // неудачных Invoke()
volatile uint16_t g_ai_flags      = 0;   // флаги ядра
volatile float    g_ai_y_nn       = 0.f; // выход сети
volatile float    g_ai_y_algo     = 0.f; // выход ядра (для сверки)
volatile float    g_ai_health     = 0.f; // здоровье установки 0..1

//ai_tflite_handle * ai_handle;
analog_driver _analog_driver;
osMessageQueueId_t  dataRawQueueHandle;
osMessageQueueId_t  dataUsbQueueHandle;
osMessageQueueId_t  dataAiQueueHandle;
UsbDataSerialazer* _UsbDataSerialazer;
NirsDSP* _NirsDSP;
RingBuffer _UsbRingBuffer(USB_RING_BUFFER_SIZE);
Deserializer* _MainDeserializer;
SemaphoreHandle_t iic2RxTxMutex;
/* USER CODE END Variables */
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,

};
/* Definitions for DspTask */
osThreadId_t DspTaskHandle;
const osThreadAttr_t DspTask_attributes = {
  .name = "DspTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,

};
/* Definitions for AiTask */
osThreadId_t AiTaskHandle;
const osThreadAttr_t AiTask_attributes = {
  .name = "AiTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,

};
/* Definitions for UsbTask */
osThreadId_t UsbTaskHandle;
const osThreadAttr_t UsbTask_attributes = {
  .name = "UsbTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,

};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartMainTask(void *argument);
void StartDspTask(void *argument);
void StartAiTask(void *argument);
void StartUsbTask(void *argument);

void OnPacketReady(const uint8_t* data, size_t len)
{
  UsbPacket_t pack ;//
  osStatus_t status = osMessageQueuePut(dataUsbQueueHandle, &pack, 0, 0);
  if (status != osOK) {

  }

}

void ExecuteInputCommand(UsbPacket_t pack)
{
	if(pack.Data[0] == NIRS_PACKET_SET_PARAM)
	{
		ParamCode_t param = (ParamCode_t)pack.Data[1];
		if(param == NIRS_ParamBrightLevel)
		{
			float proc;
			memcpy(&proc, &pack.Data[2], 4);
			_analog_driver.SetIrLedsPower(proc);
		}
	}
}

/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	_NirsDSP = new NirsDSP();
	_UsbDataSerialazer = new UsbDataSerialazer();
	_MainDeserializer = new Deserializer(_UsbRingBuffer, NIRS_HEADER, OnPacketReady);
//	ai_handle = new ai_tflite_handle();
//	ai_handle->NN_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	iic2RxTxMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  osMessageQueueAttr_t myQueue_attributes = {
    .name = "dataRawQueue"
  };
  dataRawQueueHandle = osMessageQueueNew(64, sizeof(MesurementData_t), &myQueue_attributes);
  if (dataRawQueueHandle == NULL) {
    // Handle error: Queue creation failed

  }
  osMessageQueueAttr_t usbQueue_attributes = {
    .name = "dataUsbQueue"
  };
  dataUsbQueueHandle = osMessageQueueNew(64, sizeof(UsbPacket_t), &usbQueue_attributes);
  if (dataUsbQueueHandle == NULL) {
    // Handle error: Queue creation failed

  }

  osMessageQueueAttr_t aiQueue_attributes = {
    .name = "dataAiQueue"
  };
  dataAiQueueHandle = osMessageQueueNew(64, sizeof(NirsFilteredData_t), &aiQueue_attributes);
  if (dataAiQueueHandle == NULL) {
    // Handle error: Queue creation failed

  }
  /* USER CODE END RTOS_QUEUES */
  /* creation of MainTask */
  MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

  /* creation of DspTask */
  DspTaskHandle = osThreadNew(StartDspTask, NULL, &DspTask_attributes);

  /* creation of AiTask */
  AiTaskHandle = osThreadNew(StartAiTask, NULL, &AiTask_attributes);

  /* creation of UsbTask */
  UsbTaskHandle = osThreadNew(StartUsbTask, NULL, &UsbTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartMainTask */
/**
* @brief Function implementing the MainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN MainTask */
	HAL_TIM_Base_Start_IT(&htim2);
  _analog_driver.Init();
  _analog_driver.Start();
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
  MX_USB_DRD_FS_PCD_Init();
//	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  /* Infinite loop */
  for(;;)
  {
  	Buffer_t buf = _MainDeserializer->process();
		if(buf.len > 0)
		{

			UsbPacket_t pack = _UsbDataSerialazer->GetInputPacket(&buf.data[0], buf.len);//
			osStatus_t status = osMessageQueuePut(dataUsbQueueHandle, &pack, 0, 0);
			if (status != osOK) {

			}
		}
		osDelay(1);
  }
  /* USER CODE END MainTask */
}

/* USER CODE BEGIN Header_StartDspTask */
/**
* @brief Function implementing the DspTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDspTask */
void StartDspTask(void *argument)
{
  /* USER CODE BEGIN DspTask */
	MesurementData_t mesData;

  /* Infinite loop */
  for(;;)
  {
  	osStatus_t status = osMessageQueueGet(dataRawQueueHandle, &mesData, NULL, osWaitForever);
		if (status == osOK) {

			NirsFilteredData_t fltData = _NirsDSP->ProcessLP(mesData);

			status = osMessageQueuePut(dataAiQueueHandle, &fltData, 0, 0);

      if (status != osOK) {

      }
//			fltData.Led740_3 = mesData.Led740_3;
//			fltData.Led740_4 = mesData.Led740_Bgd_3;
//
//			fltData.Led850_3 = mesData.Led850_3;
//			fltData.Led850_4 = mesData.Led850_Bgd_3;
//			fltData.Led850_1 = mesData.Led850_3;
//			fltData.Led850_2 = mesData.Led850_Bgd_3;
			UsbPacket_t pack = _UsbDataSerialazer->NirsFilteredData2UsbPacket(fltData);
			status = osMessageQueuePut(dataUsbQueueHandle, &pack, 0, 0);
//			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			if (status != osOK) {

			}

		}
    osDelay(1);
  }
  /* USER CODE END DspTask */
}

/* USER CODE BEGIN Header_StartAiTask */
/**
* @brief Function implementing the AiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAiTask */
void StartAiTask(void *argument)
{
    (void)argument;

    if (nirs_nn_setup() != 0) {
        Error_Handler();
    }
    /* Частота кадров = частота сценариев измерения (100 Гц). Все постоянные
       времени ядра и предфильтра заданы в секундах и пересчитываются от неё. */
    nirs_nn_init(&g_nn, (float)NIRS_SAMPLE_RATE_HZ);

    nirs_cfg_t cfg;
    nirs_defaults(&cfg, (float)NIRS_SAMPLE_RATE_HZ);
    nirs_init(&g_ref, &cfg);

    NirsFilteredData_t AiPack;
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x0000);

    for (;;)
    {
        // osWaitForever -> вернуться может только osOK
        if (osMessageQueueGet(dataAiQueueHandle, &AiPack, NULL, osWaitForever) != osOK)
            continue;

        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

        float v[NIRS_NN_CH] = {
            (float)AiPack.Led740_1 * k,
            (float)AiPack.Led740_2 * k,
            (float)AiPack.Led740_3 * k,
            (float)AiPack.Led740_4 * k,
            (float)AiPack.Led850_1 * k,
            (float)AiPack.Led850_2 * k,
            (float)AiPack.Led850_3 * k,
            (float)AiPack.Led850_4 * k
        };

        // 1. Ядро. Считается всегда: оно даёт признаки для сети, флаги
        //    качества сигнала и признак стабилизации.
        float y_algo = nirs_update(&g_ref, v);

        // 2. Признаки для сети — из ядра, ровно те же, что при обучении
        //    (01_build_dataset.py --core-features).
        float feat[NIRS_NN_CH];
        nirs_nn_features(&g_ref, feat);

        // 3. Сеть. Внутри уже стоит защита от нечисел в скрытом состоянии.
        float y_nn = nirs_nn_update_features(&g_nn, feat);

        // 4. Что отдавать на ЦАП. Пока датчик не на мышце — ноль, а не
        //    случайное число.
        float y_out = y_nn;
        if (!nirs_tracking(&g_ref) || !nirs_nn_ready(&g_nn))
            y_out = 0.0f;

        // Явная проверка перед приведением к целому: (uint16_t) от NaN —
        // неопределённое поведение. Условие написано «от обратного»,
        // потому что любое сравнение с NaN ложно.
        uint16_t dac = 0;
        if (y_out >= 0.0f && y_out <= 1.0f)
            dac = (uint16_t)(y_out * 4095.0f);
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac);

        g_ai_frames++;
        g_ai_nan_events = nirs_nn_nan_events(&g_nn);
        g_ai_bad_invoke = nirs_nn_bad_invokes(&g_nn);
        g_ai_flags      = nirs_flags(&g_ref);
        g_ai_health     = nirs_health(&g_ref);   // 0..1, качество контакта
        g_ai_y_nn       = y_nn;
        g_ai_y_algo     = y_algo;

        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    }
}

// ============================================================================
//  Если g_ai_nan_events продолжает расти — ищите причину здесь
// ============================================================================
//
//  Обвязка теперь самовосстанавливается, но нечисло откуда-то берётся, и это
//  симптом. По убыванию вероятности:
//
//  1. ПЕРЕПОЛНЕНИЕ СТЕКА ЗАДАЧИ. У AiTask стек 512*4 = 2 КБ, и в нём же
//     работает MicroInterpreter::Invoke(). Этого мало. Поставьте
//     .stack_size = 2048 * 4 (8 КБ) и включите в FreeRTOSConfig.h:
//
//         #define configCHECK_FOR_STACK_OVERFLOW  2
//
//     плюс vApplicationStackOverflowHook(). Переполнение стека портит
//     соседнюю память — и .bss, где лежат g_nn.state[] и арена TFLM.
//     Симптом ровно такой: «работает, потом вдруг NaN».
//
//  2. КОНТЕКСТ FPU В FreeRTOS. Cortex-M33 с FPU: в FreeRTOSConfig.h должно
//     быть configENABLE_FPU = 1, и порт должен быть ARM_CM33 (а не CM4F
//     и не CM3). Если регистры FPU не сохраняются при переключении задач,
//     значения float портятся между задачами. DspTask тоже считает во
//     float — значит переключения происходят прямо посреди арифметики.
//
//  3. -ffast-math В РЕЛИЗНОЙ СБОРКЕ. Она включает -ffinite-math-only, и
//     компилятор получает право считать, что нечисел не бывает: isnan()
//     и isfinite() сворачиваются в константу, любая защита на них исчезает
//     молча. В nirs_nn_runtime.c проверка сделана по битам экспоненты и от
//     этого не зависит, но ваш остальной код может пострадать. Проверьте
//     флаги проекта: для этой задачи -ffast-math не нужна.
//
//  4. ПОТЕРЯ КАДРОВ. DspTask кладёт в очередь с таймаутом 0 и при
//     переполнении молча теряет кадр (status != osOK, а тело if пустое).
//     Ядро и сеть считают время выборками: потери сдвигают все постоянные
//     времени. Заведите счётчик потерь и посмотрите, растёт ли он.
//
//  5. АРЕНА TFLM. Если kArenaSize урезан слишком сильно, TFLM может
//     вернуть ошибку из Invoke() (это видно по g_ai_bad_invoke) либо
//     начать писать за границу. Держите запас ~20 % от значения, которое
//     печатает arena_used_bytes().
//
//  Чтобы отличить (1)/(2) от остального: заполните g_nn.state[] известным
//  узором при старте и раз в секунду проверяйте контрольную сумму области
//  вокруг него. Если рушится память рядом — это стек.
// ============================================================================

/* USER CODE BEGIN Header_StartUsbTask */
/**
* @brief Function implementing the UsbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsbTask */
void StartUsbTask(void *argument)
{
  /* USER CODE BEGIN UsbTask */
	UsbPacket_t UsbPack;
  /* Infinite loop */
  for(;;)
  {
  	osStatus_t status = osMessageQueueGet(dataUsbQueueHandle, &UsbPack, NULL, osWaitForever);
		if (status == osOK) {
			if(UsbPack.IsInput)
			{
					ExecuteInputCommand(UsbPack);
			}
			else
			{
				while(TEMPLATE_Transmit(&UsbPack.Data[0], UsbPack.Len) != USBD_OK)
				{
					osDelay(1);
				}
			}
		}
    osDelay(1);
  }
  /* USER CODE END UsbTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

