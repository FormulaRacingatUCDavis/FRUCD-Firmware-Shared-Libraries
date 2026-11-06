// test.c
#include <stdio.h>
#include <stdint.h>
#include "frucd.h"

int main(void) {
    // pretend the PEI sent: temp=35, soc=80, voltage=4660 (big byte 18, little byte 52)
    uint8_t rxData[4] = {35, 80, 18, 52};

    // OLD way (copied from your can_manager.c)
    uint8_t PACK_TEMP = rxData[0];
    uint8_t soc = rxData[1];
    int16_t pack_voltage = (rxData[2] << 8);
    pack_voltage += rxData[3];

    // NEW way
    struct frucd_diagnostic_bms_data_t m;
    int rc = frucd_diagnostic_bms_data_unpack(&m, rxData, 4);

    printf("rc=%d (0 means ok)\n", rc);
    printf("temp:    old=%d  new=%d\n", PACK_TEMP, m.pei_hi_temp);
    printf("soc:     old=%d  new=%d\n", soc, m.pei_soc);
    printf("voltage: old=%d  new=%d\n", pack_voltage, m.pei_pack_voltage);

    // too-short frame should be rejected
    printf("short frame rc=%d (should be negative)\n",
           frucd_diagnostic_bms_data_unpack(&m, rxData, 3));
    return 0;
}

// Run it with
// gcc test.c frucd.c -o test
// ./test