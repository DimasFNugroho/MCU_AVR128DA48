#include <stdint.h>

#define _AVR128DA64_H

#include "USART_test.h"

void init_usart(usart_register *int_state)
{
	int_state->control_A_register = REG_VAL_ZERO;
	int_state->control_B_register = REG_VAL_ZERO;
	int_state->control_C_register = REG_VAL_ZERO;
	int_state->mspi_enable = false;
	int_state->baudrate_H_register = REG_VAL_ZERO;
	int_state->baudrate_L_register = REG_VAL_ZERO;
	int_state->control_D_register = REG_VAL_ZERO;
	int_state->debug_control_register = REG_VAL_ZERO;
	int_state->irda_control_register = REG_VAL_ZERO;
	int_state->ircom_tx_pulse_len_reg = REG_VAL_ZERO;
	int_state->ircom_rx_pulse_len_reg = REG_VAL_ZERO;
};
/*
	WAIT_FOR_BREAK_OFF = 0x00,
	WAIT_FOR_BREAK_ON = 0x01,
	BREAK_DETECTED = 0x02,
	NOTHING_DETECTED = 0x04,
	INCONSISTENT_SYNCHRONIZATION_FIELD =0x08,
	USART_RECEIVE_START = 0x10,
	DATA_REGISTER_EMPTY = 0x20,
	USART_TRANSMIT_COMPLETE = 0x40,
	USART_RECEIVE_COMPLETE = 0x80

#define RX_COMPLETE_INT_FLAG 0x80
#define TX_COMPLETE_INT_FLAG 0x40
#define DATA_REG_EMPTY_INT_FLAG 0x20
#define RX_START_FRAME_INT_FLAG 0x10
#define INCONSISTENT_SYNC_INT_FLAG 0x08
#define BREAK_DETECTED_FLAG 0x02
#define WAIT_FOR_BREAK_CONTROL 0x01

*/
/** USART Status Register functions */
usart_status
get_rx_complete_int_status(usart_register* reg, uint8_t status_reg)
{
	status_reg &= RX_COMPLETE_INT_FLAG;
	if (((RX_COMPLETE_INT_FLAG & status_reg) >> 7) != 0x1)
		return NOTHING_DETECTED;
	return USART_RECEIVE_COMPLETE;
}

usart_status
get_tx_complete_int_status(usart_register* reg, uint8_t status_reg)
{
	status_reg &= TX_COMPLETE_INT_FLAG;
	if (((TX_COMPLETE_INT_FLAG & status_reg) >> 6) != 0x1)
		return NOTHING_DETECTED;
	return USART_TRANSMIT_COMPLETE;
}

usart_status
get_data_reg_empty_int_status(usart_register* reg, uint8_t status_reg)
{
	status_reg &= DATA_REG_EMPTY_INT_BITS;
	if (((DATA_REG_EMPTY_INT_BITS & status_reg) >> 5) != 0x1)
		return NOTHING_DETECTED;
	return DATA_REGISTER_EMPTY;
}

usart_status
get_rx_start_frame_int_status(usart_register* reg, uint8_t status_reg)
{
	status_reg &= RX_START_FRAME_INT_FLAG;
	if (((RX_START_FRAME_INT_FLAG & status_reg) >> 4) != 0x1)
		return NOTHING_DETECTED;
	return DATA_REGISTER_EMPTY;
}

usart_status
get_inconsistent_sync_field_int_status(usart_register* reg,
                                       uint8_t status_reg)
{
	status_reg &= INCONSISTENT_SYNC_INT_FLAG;
	if (((INCONSISTENT_SYNC_INT_FLAG & status_reg) >> 3) != 0x1)
		return NOTHING_DETECTED;
	return INCONSISTENT_SYNCHRONIZATION_FIELD;
}

usart_status
get_break_detected_status(usart_register* reg, uint8_t status_reg)
{
	status_reg &= BREAK_DETECTED_FLAG;
	if (((BREAK_DETECTED_FLAG & status_reg) >> 2) != 0x1)
		return NOTHING_DETECTED;
	return BREAK_DETECTED;
}

usart_status
wait_for_break_on(usart_register* reg, uint8_t status_reg)
{
}

usart_status
wait_for_break_off(usart_register* reg, uint8_t status_reg)
{
}

/** Control A Register Functions */
void
rx_complete_int_on(usart_register* reg)
{
	reg->control_A_register |= RX_COMPLETE_INT_BITS;
};

void
rx_complete_int_off(usart_register* reg)
{
	reg->control_A_register &= ~RX_COMPLETE_INT_BITS;
};

void
tx_complete_int_on(usart_register* reg)
{
	reg->control_A_register |= TX_COMPLETE_INT_BITS;
};

void
tx_complete_int_off(usart_register* reg)
{
	reg->control_A_register &= ~TX_COMPLETE_INT_BITS;
};

void
data_reg_empty_int_on(usart_register* reg)
{
	reg->control_A_register |= DATA_REG_EMPTY_INT_BITS;
};

void
data_reg_empty_int_off(usart_register* reg)
{
	reg->control_A_register &= ~DATA_REG_EMPTY_INT_BITS;
};

void
rx_start_frame_int_on(usart_register* reg)
{
	reg->control_A_register |= RX_START_FRAME_INT_BITS;
};

void
rx_start_frame_int_off(usart_register* reg)
{
	reg->control_A_register &= ~RX_START_FRAME_INT_BITS;
};

void
loop_back_mode_on(usart_register* reg)
{
	reg->control_A_register |= LOOP_BACK_MODE_BITS;
};

void
loop_back_mode_off(usart_register* reg)
{
	reg->control_A_register &= ~LOOP_BACK_MODE_BITS;
};

void
auto_baud_err_int_on(usart_register* reg)
{
	reg->control_A_register |= AUTO_BAUD_ERR_INT_BITS;
};

void
auto_baud_err_int_off(usart_register* reg)
{
	reg->control_A_register &= ~AUTO_BAUD_ERR_INT_BITS;
};

void
rs_485_mode_on(usart_register* reg)
{
	reg->control_A_register |= RS_485_MODE_BITS;
};

void
rs_485_mode_off(usart_register* reg)
{
	reg->control_A_register &= ~RS_485_MODE_BITS;
};

/** Control B Register Functions */
void
rx_on(usart_register* reg)
{
	reg->control_B_register |= RX_ENABLE_BITS;
};
/* This function DISABLES the USART receiver */
void
rx_off(usart_register* reg)
{
	reg->control_B_register &= ~RX_ENABLE_BITS;
};

/* This function ENABLES the USART transmitter */
void
tx_on(usart_register* reg)
{
	reg->control_B_register |= TX_ENABLE_BITS;
};
/** This function DISABLES the USART transmitter */
void
tx_off(usart_register* reg)
{
	reg->control_B_register &= ~TX_ENABLE_BITS;
};

/* This function ENABLES the Start-of-Frame detection */
void
start_frame_on(usart_register* reg)
{
	reg->control_B_register |= START_FRAME_DETECT_ENABLE_BITS;
};
/* This function DISABLES the Start-of-Frame detection */
void
start_frame_off(usart_register* reg)
{
	reg->control_B_register &= ~START_FRAME_DETECT_ENABLE_BITS;
};

/* This function ENABLES the Start-of-Frame detection */
void open_drain_mode_on(usart_register* reg)
{
	reg->control_B_register |= OPEN_DRAIN_MODE_BITS;
};

/* This function DISABLES the USART transmitter */
void open_drain_mode_off(usart_register* reg)
{
	reg->control_B_register &= ~OPEN_DRAIN_MODE_BITS;
};

/* This function CONFIGURES the receiver mode of the USART */
void rx_mode_set(usart_register* reg, rxmode mode)
{
	reg->control_B_register |= RX_MODE_CLEAR_BITS;
	if (mode == NORMAL) {
		reg->control_B_register &= RX_MODE_NORMAL_BITS;
	}
	if (mode == CLK2X) {
		reg->control_B_register &= RX_MODE_CLK2X_BITS;
	}
	if (mode == GENAUTO) {
		reg->control_B_register &= RX_MODE_GENAUTO_BITS;
	} 
	if (mode == LINAUTO) {
		reg->control_B_register &= RX_MODE_LINAUTO_BITS;
	} 
};

/* This function ENABLES the open drain mode */
void
multi_processor_comm_on(usart_register* reg)
{
	reg->control_B_register |= MULTI_PROCESSOR_COMM_BITS;
}
/* This function DISABLES the open drain mode */
void multi_processor_comm_off(usart_register* reg)
{
	reg->control_B_register &= ~MULTI_PROCESSOR_COMM_BITS;
}

/** Control C register configuration functions */
/* This function CONFIGURES the receiver mode of the USART */
void
comm_mode_set(usart_register* reg, comm_mode mode)
{
	reg->control_C_register |= COMM_MODE_CLEAR_BITS;

	if (mode == ASYNCHRONOUS) {
		reg->control_C_register &= COMM_MODE_ASYNCHRONOUS_BITS;
		reg->mspi_enable = false;
	}
	if (mode == SYNCHRONOUS) {
		reg->control_C_register &= COMM_MODE_SYNCHRONOUS_BITS;
		reg->mspi_enable = false;
	}
	if (mode == IRCOM) {
		reg->control_C_register &= COMM_MODE_IRCOM_BITS;
		reg->mspi_enable = false;
	} 
	if (mode == MSPI) {
		reg->control_C_register &= COMM_MODE_MSPI_BITS;
		reg->mspi_enable = true;
	} 
};

/* This function CONFIGURES the parity mode of the USART */
void
parity_mode_set(usart_register* reg, parity_mode mode)
{
	reg->control_C_register |= PARITY_MODE_CLEAR_BITS;
	if (mode == DISABLED) {
		reg->control_C_register &= PARITY_MODE_DISABLED_BITS;
	}
	if (mode == EVEN) {
		reg->control_C_register &= PARITY_MODE_EVEN_BITS;
	}
	if (mode == ODD) {
		reg->control_C_register &= PARITY_MODE_ODD_BITS;
	} 
};

/* This function CONFIGURES the stop bit number of the USART */
void
stop_bit_set(usart_register* reg, stop_bit mode)
{
	reg->control_C_register |= STOPBIT_CLEAR_BITS;
	if (mode == ONE) {
		reg->control_C_register &= ONE_STOPBIT_BITS;
	}
	if (mode == TWO) {
		reg->control_C_register &= TWO_STOPBIT_BITS;
	}
};

/* This function CONFIGURES the receiver mode of the USART */
void
character_size_set(usart_register* reg, character_size mode)
{
	reg->control_C_register |= CHAR_SIZE_CLEAR_BITS;
	if (mode == BIT_5) {
		reg->control_C_register &= CHAR_SIZE_5BIT_BITS;
	}
	if (mode == BIT_6) {
		reg->control_C_register &= CHAR_SIZE_6BIT_BITS;
	}
	if (mode == BIT_7) {
		reg->control_C_register &= CHAR_SIZE_7BIT_BITS;
	}
	if (mode == BIT_8) {
		reg->control_C_register &= CHAR_SIZE_8BIT_BITS;
	}
	if (mode == BIT_9L) {
		reg->control_C_register &= CHAR_SIZE_9LBIT_BITS;
	}
	if (mode == BIT_9H) {
		reg->control_C_register &= CHAR_SIZE_9HBIT_BITS;
	}
};

/* This function CONFIGURES the Master SPI USART data order to transmit MSB
 * first */
usart_config_status
data_order_MSB(usart_register* reg)
{
	if (reg->mspi_enable == true) {
		reg->control_C_register |= DATA_ORDER_CLEAR_BITS;
		reg->control_C_register &= DATA_ORDER_MSB_BITS;
		return USART_CFG_SUCCESS;
	} else {
		return USART_CFG_ACCESS_DENIED;
	}
}
/* This function CONFIGURES the Master SPI USART data order to transmit LSB
 * first */
usart_config_status
data_order_LSB(usart_register* reg)
{
	if (reg->mspi_enable == true) {
		reg->control_C_register |= DATA_ORDER_CLEAR_BITS;
		reg->control_C_register &= DATA_ORDER_LSB_BITS;
		return USART_CFG_SUCCESS;
	} else {
		return USART_CFG_ACCESS_DENIED;
	}
}
/* This function CONFIGURES the USART clock phase for leading edge sampling */
usart_config_status
clock_phase_leading(usart_register* reg)
{
	if (reg->mspi_enable == true) {
		reg->control_C_register |= CLOCK_PHASE_CLEAR_BITS;
		reg->control_C_register &= CLOCK_PHASE_LEAD_EDGE_BITS;
		return USART_CFG_SUCCESS;
	} else {
		return USART_CFG_ACCESS_DENIED;
	}
}
/* This function CONFIGURES the USART clock phase for trailing edge sampling */
usart_config_status
clock_phase_trailing(usart_register* reg)
{
	if (reg->mspi_enable == true) {
		reg->control_C_register |= CLOCK_PHASE_CLEAR_BITS;
		reg->control_C_register &= CLOCK_PHASE_TRAIL_EDGE_BITS;
		return USART_CFG_SUCCESS;
	} else {
		return USART_CFG_ACCESS_DENIED;
	}
}
#define USART_BAUD_RATE(BAUD_RATE) ((float)(64 * 4000000\
                                    / (16 * (float)BAUD_RATE)) + 0.5)
/** Baudrate register configuration functions */
/* Baudrate configuration */
/* This function CONFIGURES the receiver mode of the USART */
void
baudrate_set(usart_register* reg, usart_16bits baudrate)
{
	reg->baudrate_L_register = (uint16_t)USART_BAUD_RATE(baudrate);
	reg->baudrate_H_register = (uint16_t)USART_BAUD_RATE(baudrate) >> 8;
}

/** Control D register configuration functions */
/* These bits control the tolerance for the difference between the baud rates
 * between the two synchronizing devices when using Lin Constrained Auto-baud
 * mode */
void
auto_baud_window_set(usart_register* reg, auto_baud_window_size mode)
{
	reg->control_D_register |= AUTOBAUD_WINDOW_CLEAR_BITS;
	if (mode == WDW0) {
		reg->control_D_register &= AUTOBAUD_WINDOW_WDW0_BITS;
	}
	if (mode == WDW1) {
		reg->control_D_register &= AUTOBAUD_WINDOW_WDW1_BITS;
	}
	if (mode == WDW2) {
		reg->control_D_register &= AUTOBAUD_WINDOW_WDW2_BITS;
	}
	if (mode == WDW3) {
		reg->control_D_register &= AUTOBAUD_WINDOW_WDW3_BITS;
	}
};

/** Debug Control register function */
/* This function enables debug run */
void
debug_run_enable(usart_register* reg)
{
	reg->debug_control_register |= DEBUG_RUN_BITS;
}
/* This function disables debug run */
void
debug_run_disable(usart_register* reg)
{
	reg->debug_control_register &= ~DEBUG_RUN_BITS;
}

/** IrDA Control register function */
/* This function enables IrDA Event input */
void
irda_event_input_enable(usart_register* reg)
{
	reg->irda_control_register |= IRDA_CONTROL_REG_BITS;
}
/* This function disables IrDA Event input */
void
irda_event_input_disable(usart_register* reg)
{
	reg->irda_control_register &= ~IRDA_CONTROL_REG_BITS;
}

/** IRCOM Transmitter Pulse Length Control register function */
/* This function sets IRCOM Transmitter pulse length */
transmit_pulse_length_flag
ircom_tx_pulse_length_set(usart_register* reg,
                          transmit_pulse_length pulse_len)
{
	reg->ircom_tx_pulse_len_reg = pulse_len;
	if (pulse_len == 0x00)
		return IRCOM_TX_BAUD_RATE_3OVER6_PERIOD;
	if (pulse_len == 0xFF)
		return IRCOM_TX_PULSE_CODING_DISABLED;
	return IRCOM_TX_FIXED_PULSE_LENGTH;
}

/** IRCOM Receiver Pulse Length Control register function */
/* This function sets IRCOM Receiver pulse length */
receive_pulse_length_flag
ircom_rx_pulse_length_set(usart_register* reg,
                          receive_pulse_length pulse_len)
{
	if (pulse_len > 0x7F)
		return IRCOM_RX_UNKNOWN_FILTERING_CONFIGURATION;
	else
		reg->ircom_rx_pulse_len_reg = pulse_len;

	if (pulse_len == 0x00)
		return IRCOM_RX_FILTERING_DISABLED;
	return IRCOM_RX_FILTERING_ENABLED;
}
