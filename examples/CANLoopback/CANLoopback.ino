// Copyright (c) Robert Wessels. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <CAN.h>

void setup() {
  Serial.begin(9600);

  Serial.println("CAN Receiver");
  
  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  CAN.loopback();
}

void loop() {
  sendPacket();
  delay(500);
  receivePacket();
}

void receivePacket() {
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    Serial.print("Received ");

    if (CAN.packetExtended()) {
      Serial.print("extended ");
    }

    if (CAN.packetRtr()) {
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      Serial.print(" and requested length ");
      Serial.println(CAN.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packetSize);

      while (CAN.available() > 1) {
        char c = CAN.read();
        Serial.print(c);
      }
      Serial.print(" ");
      Serial.print(CAN.read());
      Serial.println();
    }
    Serial.println("-----------------");
  }
}

void sendPacket() {
  static uint8_t i = 0;
  Serial.print("Sending packet ... ");

  CAN.beginPacket(0x12);
  CAN.write('h');
  CAN.write('e');
  CAN.write('l');
  CAN.write('l');
  CAN.write('o');
  CAN.write(i);
  CAN.endPacket();
  i++;

  Serial.println("done");
}
