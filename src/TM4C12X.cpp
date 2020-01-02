// Copyright (c) Robert Wessels. All rights reserved.
// Derived from work by Sandeep Mistry https://github.com/sandeepmistry/arduino-CAN
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifdef ENERGIA_ARCH_TIVAC

#include "TM4C12X.h"
#include <driverlib/sysctl.h>
#include <inc/hw_can.h>
#include <inc/hw_ints.h>
#include <driverlib/interrupt.h>

TM4C12XClass::TM4C12XClass() :
  CANControllerClass(),
  _loopback(false)
{
}

TM4C12XClass::~TM4C12XClass()
{
}

int TM4C12XClass::begin(long baudRate)
{
  tCANMsgObject sCANMessage;
  CANControllerClass::begin(baudRate);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

  GPIOPinConfigure(GPIO_PB4_CAN0RX);
  GPIOPinConfigure(GPIO_PB5_CAN0TX);

  GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

  /* Enable CAN Module */
  SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

  /* Wait for the CAN0 module to be ready. */
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0))
  {
  }

  /*
   * Reset the state of all the message objects and the state of the CAN
   * module to a known state.
   */
  CANInit(CAN0_BASE);

  _loopback = false;

  CANBitRateSet(CAN0_BASE, SysCtlClockGet(), baudRate);

  // CANIntRegister(CAN0_BASE, _intrHandle);
  // CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
  // IntEnable(INT_CAN0);

  CANEnable(CAN0_BASE);
  sCANMessage.ui32MsgID = 0;
  sCANMessage.ui32MsgIDMask = 0;
  // sCANMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
  sCANMessage.ui32Flags =  MSG_OBJ_USE_ID_FILTER;
  sCANMessage.ui32MsgLen = 8;
  CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);
  return 1;
}

void TM4C12XClass::end()
{
  CANDisable(CAN0_BASE);
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

  CANMessageSet(CAN0_BASE, 2, &sCANMessage, MSG_OBJ_TYPE_TX);
  uint32_t status =  CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

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
  CANMessageGet(CAN0_BASE, 1, &sCANMessage, 1);

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
  HWREG(CAN0_BASE + CAN_O_CTL) |= CAN_CTL_TEST;
  HWREG(CAN0_BASE + CAN_O_TST) |= CAN_TST_LBACK;
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
}

void TM4C12XClass::dumpRegisters(Stream& out)
{
}

void TM4C12XClass::handleInterrupt()
{
}

void TM4C12XClass::onInterrupt(void* arg)
{
}

void TM4C12XClass::_intrHandle()
{
  CANIntClear(CAN0_BASE, CAN_INT_INTID_STATUS);
}

TM4C12XClass CAN;

#endif // ENERGIA_ARCH_TIVAC