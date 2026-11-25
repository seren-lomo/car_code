/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"
#include "arm_can.h"

/* USER CODE BEGIN 0 */
__IO CAN_t can = {0};
volatile HAL_StatusTypeDef can_last_tx_status = HAL_OK;
volatile uint32_t can_tx_error_count;
volatile MotorFeedback_t motor_feedback[8];
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void USER_CAN1_Filter_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdLow = CAN_ID_EXT;
    filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) Error_Handler();
}

/* Foreground only. Preserve packet order and wait for actual transmission.
 * CAN ACK only proves a bus receiver; chassis_motion checks the motor ACK.
 */
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint16_t offset = 2;
    uint8_t count, i, packet = 0;
    uint32_t mailbox, started, shift;
    can_last_tx_status = HAL_OK;
    if (cmd == 0 || len < 3U) goto error;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    while (offset < len) {
        count = (uint8_t)(len - offset);
        if (count > 7U) count = 7U;
        header.ExtId = ((uint32_t)cmd[0] << 8) | packet;
        header.DLC = count + 1U;
        data[0] = cmd[1];
        for (i = 0; i < count; ++i) data[i + 1U] = cmd[offset + i];
        started = HAL_GetTick();
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) {
            if ((uint32_t)(HAL_GetTick() - started) >= 20U) goto timeout;
        }
        if (HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox) != HAL_OK)
            goto error;
        started = HAL_GetTick();
        while (HAL_CAN_IsTxMessagePending(&hcan, mailbox) != 0U) {
            if ((uint32_t)(HAL_GetTick() - started) >= 20U) {
                HAL_CAN_AbortTxRequest(&hcan, mailbox);
                goto timeout;
            }
        }
        shift = mailbox == CAN_TX_MAILBOX0 ? 0U :
                (mailbox == CAN_TX_MAILBOX1 ? 8U : 16U);
        if ((hcan.Instance->TSR & (CAN_TSR_TXOK0 << shift)) == 0U)
            goto error;
        hcan.Instance->TSR = CAN_TSR_RQCP0 << shift;
        offset += count;
        ++packet;
    }
    return;
timeout:
    can_last_tx_status = HAL_TIMEOUT;
    ++can_tx_error_count;
    return;
error:
    can_last_tx_status = HAL_ERROR;
    ++can_tx_error_count;
}

uint8_t can_GetRxFlag(void)
{
    uint8_t flag = can.rxFrameFlag ? 1U : 0U;
    can.rxFrameFlag = false;
    return flag;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *handle)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8], addr, i;
    uint16_t magnitude;
    uint32_t position;
    if (handle != &hcan) return;
    while (HAL_CAN_GetRxFifoFillLevel(handle, CAN_RX_FIFO0) != 0U) {
        if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, data) != HAL_OK)
            break;
        if (header.IDE != CAN_ID_EXT || header.RTR != CAN_RTR_DATA ||
            header.DLC < 2U || header.DLC > 8U) continue;
        can.CAN_RxMsg = header;
        for (i = 0; i < header.DLC; ++i) can.rxData[i] = data[i];
        can.rxFrameFlag = true;
        ArmCAN_RxFrame(header.ExtId, data, (uint8_t)header.DLC);
        if ((header.ExtId & 0xFFU) != 0U || (header.ExtId >> 8) > 7U)
            continue;
        addr = (uint8_t)(header.ExtId >> 8);
        if (addr == 0U || data[header.DLC - 1U] != 0x6BU) continue;
        if (header.DLC == 3U &&
            (data[0] == 0xF3U || data[0] == 0xF6U || data[0] == 0xFEU ||
             data[0] == 0xFDU || data[0] == 0xFFU) && data[1] != 0x9FU) {
            motor_feedback[addr].ack_function = data[0];
            motor_feedback[addr].ack_status = data[1];
            ++motor_feedback[addr].ack_count;
        } else if (header.DLC == 3U && data[0] == 0x3AU) {
            motor_feedback[addr].flags = data[1];
            motor_feedback[addr].flags_tick = HAL_GetTick();
            ++motor_feedback[addr].flags_count;
        } else if (header.DLC == 5U && data[0] == 0x35U && data[1] <= 1U) {
            magnitude = ((uint16_t)data[2] << 8) | data[3];
            if (magnitude <= 32767U) {
                motor_feedback[addr].speed_rpm =
                    data[1] == 0U ? (int16_t)magnitude : -(int16_t)magnitude;
                motor_feedback[addr].speed_tick = HAL_GetTick();
                ++motor_feedback[addr].speed_count;
            }
        } else if (header.DLC == 7U && data[0] == 0x36U && data[1] <= 1U) {
            position = ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16) |
                       ((uint32_t)data[4] << 8) | data[5];
            if (position <= 2147483647U) {
                motor_feedback[addr].position = data[1] == 0U ?
                    (int32_t)position : -(int32_t)position;
                motor_feedback[addr].position_tick = HAL_GetTick();
                ++motor_feedback[addr].position_count;
            }
        }
    }
}
/* USER CODE END 1 */

