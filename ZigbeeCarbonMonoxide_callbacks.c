/***************************************************************************//**
 * @file
 * @brief Callback implementation for ZigbeeMinimal sample application.
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

// This callback file is created for your convenience. You may add application
// code to this file. If you regenerate this file over a previous version, the
// previous version will be overwritten and any code you have added will be
// lost.

#include "app/framework/include/af.h"
#include "function_control.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "app/framework/plugin/ias-zone-server/ias-zone-server.h"


uint8_t networkJoinAttempts = 0;          //入网尝试次数
uint16_t ZoneStatus;
uint8_t msgflag;
uint8_t ledfalg;
uint8_t resendtimes = 0;

/** @brief Stack Status
 *
 * This function is called by the application framework from the stack status
 * handler.  This callbacks provides applications an opportunity to be notified
 * of changes to the stack status and take appropriate action.  The return code
 * from this callback is ignored by the framework.  The framework will always
 * process the stack status after the callback returns.
 *
 * @param status   Ver.: always
 */
bool emberAfStackStatusCallback(EmberStatus status)
{
  // This value is ignored by the framework.
  if(status == EMBER_NETWORK_UP) {                                        //入网成功，要告诉MCU.

  }

  return false;
}

/** @brief Complete
 *
 * This callback is fired when the Network Steering plugin is complete.
 *
 * @param status On success this will be set to EMBER_SUCCESS to indicate a
 * network was joined successfully. On failure this will be the status code of
 * the last join or scan attempt. Ver.: always
 * @param totalBeacons The total number of 802.15.4 beacons that were heard,
 * including beacons from different devices with the same PAN ID. Ver.: always
 * @param joinAttempts The number of join attempts that were made to get onto
 * an open Zigbee network. Ver.: always
 * @param finalState The finishing state of the network steering process. From
 * this, one is able to tell on which channel mask and with which key the
 * process was complete. Ver.: always
 */
void emberAfPluginNetworkSteeringCompleteCallback(EmberStatus status,
                                                  uint8_t totalBeacons,
                                                  uint8_t joinAttempts,
                                                  uint8_t finalState)
{
  emberAfCorePrintln("%p network %p: 0x%X", "Join", "complete", status);
  emberAfCorePrintln("111111111111111111111111111111111111111 0x%X", status);
  if (0x00 == status){
      networkJoinAttempts = 0  ;                                            //入网成功后尝试次数清零。
      ledfalg=SEND_LED_SUCCESS;                                             //让灯常量一段时间
      emberEventControlSetDelayMS(StartLowLevelEventControl,0);             //发送消息到MCU
      emberEventControlSetInactive(emberAfLedJoinNetworkStatusEventControl);//关闭重复入网尝试
      emberAfCorePrintln("status = EMBER_NETWORK_UP");
  }

}

/** @brief Ok To Sleep
 *
 * This function is called by the Idle/Sleep plugin before sleeping. It is
 * called with interrupts disabled. The application should return true if the
 * device may sleep or false otherwise.
 *
 * @param durationMs The maximum duration in milliseconds that the device will
 * sleep. Ver.: always
 */
bool emberAfPluginIdleSleepOkToSleepCallback(uint32_t durationMs)
{
  /*关闭看门狗，去睡觉*/
  halInternalDisableWatchDog(MICRO_DISABLE_WATCH_DOG_KEY);
  /*开启串口中断*/
  GPIO_ExtIntConfig(gpioPortB,          //GPIO_Port_TypeDef port
                         1,             //unsigned int pin
                         0,             //unsigned int intNo
                         true,         //bool risingEdge
                         true,          //bool fallingEdge
                         true);

  //NVIC_EnableIRQ(USART1_RX_IRQn);
  return true;
}

/** @brief Wake Up
 *
 * This function is called by the Idle/Sleep plugin after sleeping.
 *
 * @param durationMs The duration in milliseconds that the device slept.
 * Ver.: always
 */
void emberAfPluginIdleSleepWakeUpCallback(uint32_t durationMs)
{
  /**/
  /*醒来，开启看门狗*/
  halInternalEnableWatchDog();
  /*关闭串口中断，进行协议交换*/
  GPIO_ExtIntConfig(gpioPortB,          //GPIO_Port_TypeDef port
                         1,             //unsigned int pin
                         0,             //unsigned int intNo
                         true,         //bool risingEdge
                         true,          //bool fallingEdge
                         false);
}

void emberAfMainInitCallback(void)
{
  GPIO_PinModeSet(GPIO_INTERRUPT_PORT, GPIO_INTERRUPT_PIN,gpioModeInput,1); //1:DOUT  1:默认高电平  0：默认低电平
  GPIO_PinModeSet(GPIO_OUT_PORT,GPIO_OUT_PIN,gpioModePushPull,1);               //叫醒设备 20ms
  GPIO_ExtIntConfig(gpioPortB,          //GPIO_Port_TypeDef port
                         1,             //unsigned int pin
                         0,             //unsigned int intNo
                         true,          //bool risingEdge
                         true,          //bool fallingEdge
                         true);         //bool enable
  GPIOINT_Init();
  CMU_ClockEnable(cmuClock_GPIO, true);
  data_initialize();

}

void emberAfMainTickCallback(void)
{
  Serial_Recv_Data();
}

void emberAfPluginGpioSensorStateChangedCallback(uint8_t newSensorState)
{
  emberAfCorePrintln("emberAfPluginGpioSensorStateChangedCallback.%d",newSensorState);
}





