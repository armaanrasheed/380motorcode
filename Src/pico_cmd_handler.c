#include "pico_cmd_handler.h"
#include "mc_api.h"
#include "stm32g4xx_ll_usart.h"

volatile int16_t g_pico_target_rpm = 0;
volatile bool    g_pico_new_cmd    = false;

typedef enum {
  RX_WAIT_SYNC = 0,
  RX_RPM_LO,
  RX_RPM_HI,
  RX_CHECKSUM
} RxState_t;

static volatile RxState_t rx_state = RX_WAIT_SYNC;
static volatile uint8_t   rx_buf[4];

static void tx_byte(uint8_t b)
{
  while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
  LL_USART_TransmitData8(USART2, b);
}

static void tx_buf(const uint8_t *data, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++)
  {
    tx_byte(data[i]);
  }
  while (!LL_USART_IsActiveFlag_TC(USART2)) {}
}

void PicoCmd_Heartbeat(void)
{
  tx_byte(0xCC);
}

void PicoCmd_Init(void)
{
  LL_USART_EnableIT_RXNE_RXFNE(USART2);
}

void PicoCmd_IRQHandler(void)
{
  if (LL_USART_IsActiveFlag_ORE(USART2))
    LL_USART_ClearFlag_ORE(USART2);
  if (LL_USART_IsActiveFlag_FE(USART2))
    LL_USART_ClearFlag_FE(USART2);
  if (LL_USART_IsActiveFlag_NE(USART2))
    LL_USART_ClearFlag_NE(USART2);

  if (LL_USART_IsActiveFlag_RXNE_RXFNE(USART2) == 0U)
    return;

  uint8_t byte = LL_USART_ReceiveData8(USART2);

  switch (rx_state)
  {
    case RX_WAIT_SYNC:
      if (byte == PICO_CMD_SYNC_BYTE)
      {
        rx_buf[0] = byte;
        rx_state  = RX_RPM_LO;
      }
      break;

    case RX_RPM_LO:
      rx_buf[1] = byte;
      rx_state  = RX_RPM_HI;
      break;

    case RX_RPM_HI:
      rx_buf[2] = byte;
      rx_state  = RX_CHECKSUM;
      break;

    case RX_CHECKSUM:
    {
      uint8_t expected = rx_buf[0] ^ rx_buf[1] ^ rx_buf[2];
      if (byte == expected)
      {
        g_pico_target_rpm = (int16_t)((uint16_t)rx_buf[1] | ((uint16_t)rx_buf[2] << 8));
        g_pico_new_cmd    = true;
      }
      rx_state = RX_WAIT_SYNC;
      break;
    }
  }
}

void PicoCmd_Process(void)
{
  if (!g_pico_new_cmd)
    return;
  g_pico_new_cmd = false;

  int16_t rpm = g_pico_target_rpm;
  MCI_State_t state = MC_GetSTMStateMotor1();

  uint8_t dbg[4] = {0xBB, (uint8_t)state, (uint8_t)(rpm & 0xFF), (uint8_t)((rpm >> 8) & 0xFF)};
  tx_buf(dbg, 4);

  if (rpm == 0)
  {
    if (state == RUN || state == START || state == SWITCH_OVER)
      MC_StopMotor1();
    return;
  }

  switch (state)
  {
    case IDLE:
      MC_ProgramSpeedRampMotor1_F((float)rpm, 0);
      MC_StartMotor1();
      break;

    case RUN:
      MC_ProgramSpeedRampMotor1_F((float)rpm, PICO_CMD_RAMP_MS);
      break;

    case FAULT_NOW:
    case FAULT_OVER:
      MC_AcknowledgeFaultMotor1();
      break;

    default:
      break;
  }
}
