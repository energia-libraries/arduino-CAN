// Copyright (c) Robert Wessels. All rights reserved.
// Derived from work by Sandeep Mistry https://github.com/sandeepmistry/arduino-CAN
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifdef ENERGIA_ARCH_TIVAC

#include "TM4C12X.h"
#include <driverlib/sysctl.h>
#include <inc/hw_can.h>
#include <inc/hw_ints.h>
#include <driverlib/interrupt.h>
/* undefine Arduino.h min and max */
#undef max
#undef min
#include <algorithm>
#include <iterator> 

static const uint8_t can_rx_pin_nums[][NUM_PINS] = {{28, 58, 59}, {17, NULL, NULL}};
static const uint8_t can_tx_pin_nums[][NUM_PINS] = {{31, 57, 60}, {18, NULL, NULL}};
static const uint8_t can_rx_pin_names[][NUM_PINS] = {{GPIO_PF0_CAN0RX, GPIO_PB4_CAN0RX, GPIO_PE4_CAN0RX}, {GPIO_PA0_CAN1RX, NULL, NULL}};
static const uint8_t can_gpio_rx_pin_nums[][NUM_PINS] = {{GPIO_PIN_0, GPIO_PIN_4, GPIO_PIN_4}, {GPIO_PIN_1, NULL, NULL}};
static const uint8_t can_tx_pin_names[][NUM_PINS] = {{GPIO_PF3_CAN0TX, GPIO_PB5_CAN0TX, GPIO_PE5_CAN0TX}, {GPIO_PA1_CAN1TX, NULL, NULL}};
static const uint8_t can_gpio_tx_pin_nums[][NUM_PINS] = {{GPIO_PIN_3, GPIO_PIN_5, GPIO_PIN_5}, {GPIO_PIN_1, NULL, NULL}};

static const uint8_t can_gpio_base_names[][NUM_PINS] = {{GPIO_PORTF_BASE, GPIO_PORTB_BASE, GPIO_PORTE_BASE}, {GPIO_PORTA_BASE, NULL, NULL}};
static const uint8_t can_sysctl_periph[][NUM_PINS] = {{SYSCTL_PERIPH_GPIOF, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOE}, {SYSCTL_PERIPH_GPIOA, NULL, NULL}};
static const uint32_t can_base[] = {CAN0_BASE, CAN1_BASE};
static const uint32_t can_sys_perf[] = {SYSCTL_PERIPH_CAN0, SYSCTL_PERIPH_CAN1};
static const uint8_t can_int[] = {INT_CAN0, INT_CAN1};
#define TX_MSG_OBJECT 2

uint8_t TM4C12XClass::_module = 0;

TM4C12XClass::TM4C12XClass() :
  CANControllerClass(),
  _loopback(false)
{
}

TM4C12XClass::TM4C12XClass(uint8_t module) :
  CANControllerClass(),
  _loopback(false)
{
  _module = module;
}

TM4C12XClass::~TM4C12XClass()
{
}

int TM4C12XClass::begin(long baudRate)
{
  tCANMsgObject sCANMessage;
  CANControllerClass::begin(baudRate);

  if(!_pins_conf) {
    setPins(58, 57);
  }
   /* Enable CAN Module */
   SysCtlPeripheralEnable(can_sys_perf[_module]);

  /* Wait for the CAN0 module to be ready. */
  while(!SysCtlPeripheralReady(can_sys_perf[_module]))
  {
  }

  /*
   * Reset the state of all the message objects and the state of the CAN
   * module to a known state.
   */
  CANInit(can_base[_module]);

  _loopback = false;

  CANBitRateSet(can_base[_module], SysCtlClockGet(), baudRate);

  CANIntRegister(can_base[_module], onInterrupt);
  CANIntEnable(can_base[_module], CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
  IntEnable(can_int[_module]);

  CANEnable(can_base[_module]);
  sCANMessage.ui32MsgID = 0;
  sCANMessage.ui32MsgIDMask = 0;
  sCANMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
  sCANMessage.ui32Flags =  MSG_OBJ_USE_ID_FILTER;
  sCANMessage.ui32MsgLen = 8;
  CANMessageSet(can_base[_module], 1, &sCANMessage, MSG_OBJ_TYPE_RX);
  return 1;
}

void TM4C12XClass::end()
{
  CANDisable(can_base[_module]);
  CANControllerClass::end();
}

int TM4C12XClass::endPacket()
{
  tCANMsgObject sCANMessage;
  uint8_t pui8MsgData[8];
  sCANMessage.ui32MsgID = _txId;
  sCANMessage.ui32MsgIDMask = 0;
  sCANMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
  sCANMessage.ui32MsgLen = _txLength;
  memcpy(pui8MsgData, _txData, _txLength);
  sCANMessage.pui8MsgData = pui8MsgData;

  CANMessageSet(can_base[_module], 2, &sCANMessage, MSG_OBJ_TYPE_TX);
  uint32_t status =  CANStatusGet(can_base[_module], CAN_STS_CONTROL);

  if(status & CAN_STATUS_TXOK) {
    return 1;
  }

  return 0;
}

int TM4C12XClass::parsePacket()
{
  tCANMsgObject sCANMessage;
  uint8_t pui8MsgData[8];

  sCANMessage.pui8MsgData = pui8MsgData;
  CANMessageGet(can_base[_module], 1, &sCANMessage, 1);

  /* Message available? */
  if(!sCANMessage.ui32Flags & MSG_OBJ_NEW_DATA) {
    return 0;
  }

  _rxExtended = (sCANMessage.ui32Flags & MSG_OBJ_EXTENDED_ID) >> MSG_OBJ_EXTENDED_ID;
  _rxRtr = (sCANMessage.ui32Flags & MSG_OBJ_REMOTE_FRAME) >> MSG_OBJ_REMOTE_FRAME;
  _rxDlc = sCANMessage.ui32MsgLen;
  _rxIndex = 0;
  _rxId = sCANMessage.ui32MsgID;

  if (_rxRtr) {
    _rxLength = 0;
  } else {
    _rxLength = _rxDlc;
    memcpy(_rxData, sCANMessage.pui8MsgData, 8);
  }

  return _rxDlc;
}

void TM4C12XClass::onReceive(void(*callback)(int))
{
  CANControllerClass::onReceive(callback);

  if(!_onReceive) {
    return;
  }

  while (parsePacket()) {
    _onReceive(available());
  }
}

int TM4C12XClass::filter(int id, int mask)
{
  return 1;
}

int TM4C12XClass::filterExtended(long id, long mask)
{
  return 1;
}

int TM4C12XClass::observe()
{
  return 1;
}

int TM4C12XClass::loopback()
{
  _loopback = true;
  HWREG(can_base[_module] + CAN_O_CTL) |= CAN_CTL_TEST;
  HWREG(can_base[_module] + CAN_O_TST) |= CAN_TST_LBACK;
  return 1;
}

int TM4C12XClass::sleep()
{
  return 1;
}

int TM4C12XClass::wakeup()
{
  return 1;
}

void TM4C12XClass::setPins(int rx, int tx)
{
  bool exists;
  uint8_t pins;
  uint8_t tx_pos;
  uint8_t rx_pos;

  /* setPins need to be called before begin() */
  if(_begun) {
    return;
  }

  if(rx == tx) {
    return;
  }

  exists = std::find(std::begin(can_rx_pin_nums[_module]), std::end(can_rx_pin_nums[_module]), rx) != std::end(can_rx_pin_nums[_module]);
  if(!exists) {
    return;
  }

  exists = std::find(std::begin(can_tx_pin_nums[_module]), std::end(can_tx_pin_nums[_module]), tx) != std::end(can_tx_pin_nums[_module]);
  if(!exists) {
    return;
  }

  rx_pos = std::distance(can_rx_pin_nums[_module], std::find(can_rx_pin_nums[_module], can_rx_pin_nums[_module] + NUM_PINS, rx));
  tx_pos = std::distance(can_tx_pin_nums[_module], std::find(can_tx_pin_nums[_module], can_tx_pin_nums[_module] + NUM_PINS, tx));

  GPIOPinConfigure(can_rx_pin_names[_module][rx_pos]);
  GPIOPinConfigure(can_tx_pin_names[_module][tx_pos]);

  GPIOPinTypeCAN(can_gpio_base_names[_module][rx_pos], can_gpio_rx_pin_nums[_module][rx_pos]);
  GPIOPinTypeCAN(can_gpio_base_names[_module][tx_pos], can_gpio_tx_pin_nums[_module][tx_pos]);
  SysCtlPeripheralEnable(can_sysctl_periph[_module][rx_pos]);
  SysCtlPeripheralEnable(can_sysctl_periph[_module][tx_pos]);
  _pins_conf = true;

}

void TM4C12XClass::dumpRegisters(Stream& out)
{
}

void TM4C12XClass::handleInterrupt()
{
  if(!_onReceive) {
    return;
  }

  while (parsePacket()) {
    _onReceive(available());
  }
}

void TM4C12XClass::onInterrupt()
{
  uint32_t ui32Status;
  
  ui32Status = CANIntStatus(can_base[_module], CAN_INT_STS_CAUSE);

  if(ui32Status == CAN_INT_INTID_STATUS) {
    ui32Status = CANStatusGet(can_base[_module], CAN_STS_CONTROL);
  } else if (ui32Status == TX_MSG_OBJECT) {
    CAN.handleInterrupt();
  }
  
  CANIntClear(can_base[_module], ui32Status);
}

TM4C12XClass CAN0(0);
TM4C12XClass CAN1(1);

#endif // ENERGIA_ARCH_TIVAC
