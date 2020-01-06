// Copyright (c) Robert Wessels. All rights reserved.
// Derived from work by Sandeep Mistry https://github.com/sandeepmistry/arduino-CAN
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifdef ENERGIA_ARCH_TIVAC

#ifndef TM4C12X_H
#define TM4C12X_H

#include "CANController.h"
#include <driverlib/can.h>

#define NUM_PINS 3

class TM4C12XClass : public CANControllerClass {

public:
  TM4C12XClass();
  TM4C12XClass(uint8_t);
  virtual ~TM4C12XClass();
  
  virtual int begin(long baudRate);
  virtual void end();

  virtual int endPacket();

  virtual int parsePacket();

  virtual void onReceive(void(*callback)(int));

  using CANControllerClass::filter;
  virtual int filter(int id, int mask);
  using CANControllerClass::filterExtended;
  virtual int filterExtended(long id, long mask);

  virtual int observe();
  virtual int loopback();
  virtual int sleep();
  virtual int wakeup();

  void setPins(int rx, int tx);
  void dumpRegisters(Stream& out);
private:
  void reset();

  void handleInterrupt();
  static void onInterrupt();

private:
  static uint8_t _module;
  bool _pins_conf;
  bool _loopback;
  bool _begun;
  static void _intrHandle();
};

extern TM4C12XClass CAN0;
extern TM4C12XClass CAN1;
#define CAN CAN0
#endif // ENERGIA_ARCH_TIVAC
#endif // TM4C12X_H
