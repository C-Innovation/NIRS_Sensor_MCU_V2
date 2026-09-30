/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usb_drd_fs.c
  * @brief   This file provides code for the configuration
  *          of the USB_DRD_FS instances.
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
  * see https://community.st.com/stm32-mcus-60/how-to-use-stmicroelectronics-classic-usb-device-middleware-with-new-stm32-families-122491
  */


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usb_drd_fs.h"

/* USER CODE BEGIN 0 */
#include "usbd_core.h"
#include "usbd_cdc_if.h"
USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef Class_Desc;
/* USER CODE END 0 */

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USB_DRD_FS init function */

void MX_USB_DRD_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_DRD_FS_Init 0 */
	hpcd_USB_DRD_FS.pData = &hUsbDeviceFS;
  /* USER CODE END USB_DRD_FS_Init 0 */

  /* USER CODE BEGIN USB_DRD_FS_Init 1 */
  SET_BIT(PWR->SVMCR, PWR_SVMCR_UVMEN);
  while(!(PWR->SVMSR&PWR_SVMSR_VDDUSBRDY));
  /* Enable VDDUSB */
  if(__HAL_RCC_PWR_IS_CLK_DISABLED())
  {
		__HAL_RCC_PWR_CLK_ENABLE();
		HAL_PWREx_EnableVddUSB();
		__HAL_RCC_PWR_CLK_DISABLE();
  }
  else
  {
  	HAL_PWREx_EnableVddUSB();
  }
  SET_BIT(PWR->VOSR, PWR_VOSR_BOOSTEN);
  /* USER CODE END USB_DRD_FS_Init 1 */
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 4;
  hpcd_USB_DRD_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_DRD_FS_Init 2 */
  if(USBD_Init(&hUsbDeviceFS, &Class_Desc, 0) != USBD_OK)
   Error_Handler();
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
   Error_Handler();
  if(USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_CDC_Template_fops) != USBD_OK)
   Error_Handler();
  if(USBD_Start(&hUsbDeviceFS) != USBD_OK)
   Error_Handler();
  /* USER CODE END USB_DRD_FS_Init 2 */

}

void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(pcdHandle->Instance==USB_DRD_FS)
  {
  /* USER CODE BEGIN USB_DRD_FS_MspInit 0 */

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USB_OTG_FS GPIO Configuration
	PA11     ------> USB_OTG_FS_DM
	PA12     ------> USB_OTG_FS_DP
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_USB;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /* USER CODE END USB_DRD_FS_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    PeriphClkInit.IclkClockSelection = RCC_CLK48CLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USB_DRD_FS clock enable */
    __HAL_RCC_USB_FS_CLK_ENABLE();

    /* USB_DRD_FS interrupt Init */
    HAL_NVIC_SetPriority(USB_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_IRQn);
  /* USER CODE BEGIN USB_DRD_FS_MspInit 1 */

  /* USER CODE END USB_DRD_FS_MspInit 1 */
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* pcdHandle)
{

  if(pcdHandle->Instance==USB_DRD_FS)
  {
  /* USER CODE BEGIN USB_DRD_FS_MspDeInit 0 */

  /* USER CODE END USB_DRD_FS_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USB_FS_CLK_DISABLE();

    /* USB_DRD_FS interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_IRQn);
  /* USER CODE BEGIN USB_DRD_FS_MspDeInit 1 */

  /* USER CODE END USB_DRD_FS_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

