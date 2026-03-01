#ifndef PICO_CMD_HANDLER_H
#define PICO_CMD_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

#define PICO_CMD_SYNC_BYTE  0xAA
#define PICO_CMD_RAMP_MS    50

void PicoCmd_Init(void);
void PicoCmd_IRQHandler(void);
void PicoCmd_Process(void);
void PicoCmd_Heartbeat(void);

extern volatile int16_t g_pico_target_rpm;
extern volatile bool    g_pico_new_cmd;

#endif /* PICO_CMD_HANDLER_H */
