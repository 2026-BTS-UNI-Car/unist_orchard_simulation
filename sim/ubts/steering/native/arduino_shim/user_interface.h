/* ESP8266 NONOS SDK user_interface.h subset for the native SIM build: the reset information. */
#ifndef UBTS_SIM_USER_INTERFACE_H
#define UBTS_SIM_USER_INTERFACE_H

#include <stdint.h>

enum rst_reason {
  REASON_DEFAULT_RST = 0,      /* power on */
  REASON_WDT_RST = 1,          /* hardware watchdog */
  REASON_EXCEPTION_RST = 2,    /* exception */
  REASON_SOFT_WDT_RST = 3,     /* software watchdog */
  REASON_SOFT_RESTART = 4,     /* software restart */
  REASON_DEEP_SLEEP_AWAKE = 5, /* wake from deep sleep */
  REASON_EXT_SYS_RST = 6       /* external system reset (RST pin, USB DTR/RTS) */
};

struct rst_info {
  uint32_t reason;
  uint32_t exccause;
  uint32_t epc1;
  uint32_t epc2;
  uint32_t epc3;
  uint32_t excvaddr;
  uint32_t depc;
};

#endif /* UBTS_SIM_USER_INTERFACE_H */
