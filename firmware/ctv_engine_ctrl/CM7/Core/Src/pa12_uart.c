#include "pa12_uart.h"

#include "main.h"

extern UART_HandleTypeDef huart7; /* A1 */
extern UART_HandleTypeDef huart5; /* A2 */

#define FIFO_DEPTH 16u
#define UART_ERRORS (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)
#define UART_ERROR_CLEAR (USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF)

typedef struct {
    UART_HandleTypeDef *uart;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    bool tx_active;
} port_hw_t;

static port_hw_t hw[FCEC_PA12_PORTS];

static void flush_rx(USART_TypeDef *u) {
    u->RQR = USART_RQR_RXFRQ;
    u->ICR = UART_ERROR_CLEAR;
}

static bool start_tx(void *context, unsigned port, const uint8_t *bytes, size_t len) {
    (void)context;
    if (port >= FCEC_PA12_PORTS || hw[port].tx_active || len == 0u || len > FIFO_DEPTH) {
        return false;
    }
    USART_TypeDef *u = hw[port].uart->Instance;
    if ((u->ISR & USART_ISR_TXFE) == 0U) {
        return false; /* something is still in the FIFO */
    }
    flush_rx(u);
    u->ICR = USART_ICR_TCCF;
    HAL_GPIO_WritePin(hw[port].dir_port, hw[port].dir_pin, GPIO_PIN_SET); /* transmit */
    for (size_t i = 0; i < len; i++) {
        u->TDR = bytes[i]; /* fits: FIFO was empty and len <= 16 */
    }
    hw[port].tx_active = true;
    return true;
}

static const fcec_pa12_ops_t ops = {NULL, start_tx};

bool PA12_UART_Init(void) {
    hw[0] = (port_hw_t){&huart7, Actuator_data_dir_a1_GPIO_Port, Actuator_data_dir_a1_Pin, false};
    hw[1] = (port_hw_t){&huart5, Actuator_data_dir_a2_GPIO_Port, Actuator_data_dir_a2_Pin, false};
    for (unsigned i = 0; i < FCEC_PA12_PORTS; i++) {
        HAL_GPIO_WritePin(hw[i].dir_port, hw[i].dir_pin, GPIO_PIN_RESET); /* receive */
        if (HAL_UARTEx_EnableFifoMode(hw[i].uart) != HAL_OK) {
            return false;
        }
        flush_rx(hw[i].uart->Instance);
    }
    return true;
}

const fcec_pa12_ops_t *PA12_UART_Ops(void) {
    return &ops;
}

void PA12_UART_Poll(fcec_pa12_t *drv, uint64_t now_us) {
    for (unsigned i = 0; i < FCEC_PA12_PORTS; i++) {
        USART_TypeDef *u = hw[i].uart->Instance;
        if (hw[i].tx_active && (u->ISR & USART_ISR_TC) != 0U) {
            /* Final stop bit is out: release the line, drop any echo. */
            HAL_GPIO_WritePin(hw[i].dir_port, hw[i].dir_pin, GPIO_PIN_RESET);
            flush_rx(u);
            hw[i].tx_active = false;
            fcec_pa12_tx_done(drv, i, true, now_us);
        }
        if ((u->ISR & UART_ERRORS) != 0U) {
            u->ICR = UART_ERROR_CLEAR; /* a bad byte shows up as a bad reply */
        }
        for (unsigned n = 0; n < FIFO_DEPTH && (u->ISR & USART_ISR_RXNE_RXFNE) != 0U; n++) {
            fcec_pa12_rx_byte(drv, i, (uint8_t)u->RDR, now_us);
        }
        /* If the driver gave up on a transfer (timeout), release the line. */
        if (hw[i].tx_active && drv->port[i].phase != FCEC_PA12_PHASE_TX) {
            HAL_GPIO_WritePin(hw[i].dir_port, hw[i].dir_pin, GPIO_PIN_RESET);
            u->RQR = USART_RQR_TXFRQ;
            flush_rx(u);
            hw[i].tx_active = false;
        }
    }
}
