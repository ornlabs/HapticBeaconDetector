/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed.h"
#include "ble/BLE.h"
#include "ble/DiscoveredCharacteristic.h"
#include "ble/DiscoveredService.h"


BLE ble;

DigitalOut alivenessLED(P0_19, 1);

bool triggerLedCharacteristic = false;
DiscoveredCharacteristic ledCharacteristic;

bool triggerBuzz = false;

int drvSetup();
int drvEffect(uint8_t effect);

void periodicCallback(void) {
    alivenessLED = !alivenessLED; /* Do blinky on LED1 while we're waiting for BLE events */

    if (triggerBuzz) {
        drvEffect(1);
    }

    triggerBuzz = false;
}

void advertisementCallback(const Gap::AdvertisementCallbackParams_t *params) {
    printf("adv peerAddr[%02x %02x %02x %02x %02x %02x] rssi %d, isScanResponse %u, AdvertisementType %u\r\n",
           params->peerAddr[5], params->peerAddr[4], params->peerAddr[3], params->peerAddr[2], params->peerAddr[1], params->peerAddr[0],
           params->rssi, params->isScanResponse, params->type);

    triggerBuzz = true;
}

void serviceDiscoveryCallback(const DiscoveredService *service) {
    if (service->getUUID().shortOrLong() == UUID::UUID_TYPE_SHORT) {
        printf("S UUID-%x attrs[%u %u]\r\n", service->getUUID().getShortUUID(), service->getStartHandle(), service->getEndHandle());
    } else {
        printf("S UUID-");
        const uint8_t *longUUIDBytes = service->getUUID().getBaseUUID();
        for (unsigned i = 0; i < UUID::LENGTH_OF_LONG_UUID; i++) {
            printf("%02x", longUUIDBytes[i]);
        }
        printf(" attrs[%u %u]\r\n", service->getStartHandle(), service->getEndHandle());
    }
}

void characteristicDiscoveryCallback(const DiscoveredCharacteristic *characteristicP) {
    printf("  C UUID-%x valueAttr[%u] props[%x]\r\n", characteristicP->getShortUUID(), characteristicP->getValueHandle(), (uint8_t)characteristicP->getProperties().broadcast());
    if (characteristicP->getShortUUID() == 0xa001) { /* !ALERT! Alter this filter to suit your device. */
        ledCharacteristic        = *characteristicP;
        triggerLedCharacteristic = true;
    }
}

void discoveryTerminationCallback(Gap::Handle_t connectionHandle) {
    printf("terminated SD for handle %u\r\n", connectionHandle);
}

void connectionCallback(const Gap::ConnectionCallbackParams_t *params) {
    if (params->role == Gap::CENTRAL) {
        ble.gattClient().onServiceDiscoveryTermination(discoveryTerminationCallback);
        ble.gattClient().launchServiceDiscovery(params->handle, serviceDiscoveryCallback, characteristicDiscoveryCallback, 0xa000, 0xa001);
    }
}

void triggerToggledWrite(const GattReadCallbackParams *response) {
    if (response->handle == ledCharacteristic.getValueHandle()) {
#if DUMP_READ_DATA
        printf("triggerToggledWrite: handle %u, offset %u, len %u\r\n", response->handle, response->offset, response->len);
        for (unsigned index = 0; index < response->len; index++) {
            printf("%c[%02x]", response->data[index], response->data[index]);
        }
        printf("\r\n");
#endif

        uint8_t toggledValue = response->data[0] ^ 0x1;
        ledCharacteristic.write(1, &toggledValue);
    }
}

void triggerRead(const GattWriteCallbackParams *response) {
    if (response->handle == ledCharacteristic.getValueHandle()) {
        ledCharacteristic.read();
    }
}

void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason) {
    printf("disconnected\r\n");
}

int main(void) {

    wait(10);

    drvSetup();

    Ticker ticker;
    ticker.attach(periodicCallback, 1);

    ble.init();
    ble.gap().onConnection(connectionCallback);
    ble.gap().onDisconnection(disconnectionCallback);

    ble.gattClient().onDataRead(triggerToggledWrite);
    ble.gattClient().onDataWrite(triggerRead);

    ble.gap().setScanParams(500, 400);
    ble.gap().startScan(advertisementCallback);

    while (true) {
        if (triggerLedCharacteristic && !ble.gattClient().isServiceDiscoveryActive()) {
            triggerLedCharacteristic = false;
            ledCharacteristic.read(); // * We could have issued this read just as easily from
                                      // * characteristicDiscoveryCallback(); but
                                      // * invoking it here demonstrates the use
                                      // * of isServiceDiscoveryActive() and also
                                      // * the fact that it is permitted to
                                      // * operate on application-local copies of
                                      // * DiscoveredCharacteristic.
        }
        ble.waitForEvent();
    }
}

/*******************************************************************************
 *
 * Parts of the following were derived from https://github.com/adafruit/Adafruit_DRV2605_Library
 *
 */

#define DRV2605_REG_STATUS 0x00
#define DRV2605_REG_MODE 0x01
#define DRV2605_REG_RTPIN 0x02
#define DRV2605_REG_WAVESEQ1 0x04
#define DRV2605_REG_WAVESEQ2 0x05
#define DRV2605_REG_GO 0x0C
#define DRV2605_REG_OVERDRIVE 0x0D
#define DRV2605_REG_SUSTAINPOS 0x0E
#define DRV2605_REG_SUSTAINNEG 0x0F
#define DRV2605_REG_BREAK 0x10
#define DRV2605_REG_AUDIOMAX 0x13
#define DRV2605_REG_FEEDBACK 0x1A
#define DRV2605_REG_CONTROL3 0x1D
#define DRV2605_REG_LIBRARY 0x03

const int DRV2605_ADDR=0x5A<<1;

I2C drv2605(P0_6, P0_7); // sda, scl

int drvSetup() {
    char output_buffer[3];
    char input_buffer[3];

    printf("Start DRV %02x\r\n", DRV2605_ADDR);

    output_buffer[0] = DRV2605_REG_STATUS;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 1) != 0) {
        printf("Error write status\r\n");
        return 1;
    }
    if (drv2605.read(DRV2605_ADDR, input_buffer, 1) != 0) {
        printf("Error read status\r\n");
        return 1;
    }
    printf("status=%02x\r\n", input_buffer[0]);

    output_buffer[0] = DRV2605_REG_MODE;
    output_buffer[1] = 0x00;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_MODE\r\n");
        return 1;
    }

    output_buffer[0] = DRV2605_REG_RTPIN;
    output_buffer[1] = 0x00;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_RTPIN\r\n");
        return 1;
    }

    output_buffer[0] = DRV2605_REG_WAVESEQ1;
    output_buffer[1] = 0x01;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_WAVESEQ1\r\n");
        return 1;
    }

    output_buffer[0] = DRV2605_REG_WAVESEQ2;
    output_buffer[1] = 0x00;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_WAVESEQ2\r\n");
        return 1;
    }

    output_buffer[0] = DRV2605_REG_OVERDRIVE;
    output_buffer[1] = 0x00;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_OVERDRIVE\r\n");
        return 1;
    }

    output_buffer[0] = DRV2605_REG_SUSTAINPOS;
    output_buffer[1] = 0x00;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_SUSTAINPOS\r\n");
        return 1;
    }

    output_buffer[0] = DRV2605_REG_SUSTAINNEG;
    output_buffer[1] = 0x00;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_SUSTAINNEG\r\n");
        return 1;
    }

    output_buffer[0] = DRV2605_REG_BREAK;
    output_buffer[1] = 0x00;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_BREAK\r\n");
        return 1;
    }

    output_buffer[0] = DRV2605_REG_BREAK;
    output_buffer[1] = 0x64;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_BREAK\r\n");
        return 1;
    }

    // set feedback
    output_buffer[0] = DRV2605_REG_FEEDBACK;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 1) != 0) {
        printf("Error write-read DRV2605_REG_FEEDBACK\r\n");
        return 1;
    }
    if (drv2605.read(DRV2605_ADDR, input_buffer, 1) != 0) {
        printf("Error read DRV2605_REG_FEEDBACK\r\n");
        return 1;
    }

    printf("feedback=%02x\r\n", input_buffer[0]);

    output_buffer[0] = DRV2605_REG_FEEDBACK;
    output_buffer[1] = input_buffer[0] & 0x7F;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_FEEDBACK\r\n");
        return 1;
    }

    output_buffer[0] = DRV2605_REG_CONTROL3;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 1) != 0) {
        printf("Error write-read DRV2605_REG_CONTROL3\r\n");
        return 1;
    }
    if (drv2605.read(DRV2605_ADDR, input_buffer, 1) != 0) {
        printf("Error read DRV2605_REG_CONTROL3\r\n");
        return 1;
    }

    printf("control3=%02x\r\n", input_buffer[0]);

    output_buffer[0] = DRV2605_REG_CONTROL3;
    output_buffer[1] = input_buffer[0] | 0x20;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_CONTROL3\r\n");
        return 1;
    }

    output_buffer[0] = DRV2605_REG_MODE;
    output_buffer[1] = 0x00;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_MODE\r\n");
        return 1;
    }

    output_buffer[0] = DRV2605_REG_LIBRARY;
    output_buffer[1] = 0x01;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_LIBRARY\r\n");
        return 1;
    }

    return 0;
}

int drvEffect(uint8_t effect) {
    char output_buffer[3];

    output_buffer[0] = DRV2605_REG_WAVESEQ1;
    output_buffer[1] = effect;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_WAVESEQ1\r\n");
        return 1;
    }
    output_buffer[0] = DRV2605_REG_WAVESEQ2;
    output_buffer[1] = 0;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_WAVESEQ2\r\n");
        return 1;
    }

    output_buffer[0] = DRV2605_REG_GO;
    output_buffer[1] = 0x01;
    if (drv2605.write(DRV2605_ADDR, output_buffer, 2) != 0) {
        printf("Error write DRV2605_REG_GO\r\n");
        return 1;
    }

    return 0;
}
