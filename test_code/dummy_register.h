#include <stdint.h>
#include <stdbool.h>

volatile uint8_t dummy_rx_low_reg;
volatile uint8_t dummy_rx_high_reg;
volatile uint8_t dummy_tx_low_reg;
volatile uint8_t dummy_tx_high_reg;
volatile uint8_t dummy_stat_reg;
volatile uint8_t dummy_ctrl_A_reg;
volatile uint8_t dummy_ctrl_B_reg;
volatile uint8_t dummy_ctrl_C_reg;
volatile bool dummy_mspi_en;

volatile uint8_t dummy_baud_H_reg;
volatile uint8_t dummy_baud_L_reg;

volatile uint8_t dummy_ctrl_D_reg;

volatile uint8_t dummy_debug_ctrl_reg;
volatile uint8_t dummy_irda_ctrl_reg;
volatile uint8_t dummy_ircom_tx_plen_reg;
volatile uint8_t dummy_ircom_rx_plen_reg;
