#include "hex2binary.h"
#include "USART_test.h"
#include <stdio.h>

int main(void)
{
	/* create usart interface instance */
	usart_register int_state;

	/* init usart interface */
	init_usart(&int_state);

	/* control A */
#if 1
	printf("\n#######  Control Register A #######\n");
	rx_complete_int_on(&int_state);
	printf("rx_complete_int_on    : ");
	print_binary(int_state.control_A_register);

	tx_complete_int_on(&int_state);
	printf("rx_complete_int_off   : ");
	print_binary(int_state.control_A_register);

	data_reg_empty_int_on(&int_state);
	printf("data_reg_empty_int_on : ");
	print_binary(int_state.control_A_register);

	rx_start_frame_int_on(&int_state);
	printf("rx_start_frame_int_on : ");
	print_binary(int_state.control_A_register);

	loop_back_mode_on(&int_state);
	printf("loop_back_mode_on     : ");
	print_binary(int_state.control_A_register);

	auto_baud_err_int_on(&int_state);
	printf("auto_baud_err_int_on  : ");
	print_binary(int_state.control_A_register);

	rs_485_mode_on(&int_state);
	printf("rs_485_mode_on        : ");
	print_binary(int_state.control_A_register);
#endif

	printf("\n#########  Control Register B #########\n");
	/** Control B Register Functions */
	rx_on(&int_state);
	printf("rx_on                     : ");
	print_binary(int_state.control_B_register);

	rx_off(&int_state);
	printf("rx_off                    : ");
	print_binary(int_state.control_B_register);

	tx_on(&int_state);
	printf("tx_on                     : ");
	print_binary(int_state.control_B_register);

	tx_off(&int_state);
	printf("tx_off                    : ");
	print_binary(int_state.control_B_register);

#if 1
	start_frame_on(&int_state);
	printf("start_frame_on            : ");
	print_binary(int_state.control_B_register);

	start_frame_off(&int_state);
	printf("start_frame_off           : ");
	print_binary(int_state.control_B_register);

	open_drain_mode_on(&int_state);
	printf("open_drain_mode_on        : ");
	print_binary(int_state.control_B_register);

	open_drain_mode_off(&int_state);
	printf("open_drain_mode_off       : ");
	print_binary(int_state.control_B_register);

	printf("--rx mode------------------------------\n");
	rx_mode_set(&int_state, NORMAL);
	printf("rx_mode_set, NORMAL       : ");
	print_binary(int_state.control_B_register);

	rx_mode_set(&int_state, CLK2X);
	printf("rx_mode_set, CLK2X        : ");
	print_binary(int_state.control_B_register);

	rx_mode_set(&int_state, GENAUTO);
	printf("rx_mode_set, GENAUTO      : ");
	print_binary(int_state.control_B_register);

	rx_mode_set(&int_state, LINAUTO);
	printf("rx_mode_set, LINAUTO      : ");
	print_binary(int_state.control_B_register);
	printf("---------------------------------------\n");

	multi_processor_comm_on(&int_state);
	printf("multi_processor_comm_on   : ");
	print_binary(int_state.control_B_register);

	multi_processor_comm_off(&int_state);
	printf("multi_processor_comm_off  : ");
	print_binary(int_state.control_B_register);
#endif

/** Control C register configuration functions */
	printf("\n#########  Control Register C #########\n");
/* This function CONFIGURES the receiver mode of the USART */
	printf("--comm mode----------------------------\n");
	comm_mode_set(&int_state, ASYNCHRONOUS);
	printf("comm_mode_set ASYNCHRONOUS :");
	print_binary(int_state.control_C_register);

	comm_mode_set(&int_state, SYNCHRONOUS);
	printf("comm_mode_set SYNCHRONOUS  :");
	print_binary(int_state.control_C_register);

	comm_mode_set(&int_state, IRCOM);
	printf("comm_mode_set IRCOM        :");
	print_binary(int_state.control_C_register);

	comm_mode_set(&int_state, MSPI);
	printf("comm_mode_set MSPI         :");
	print_binary(int_state.control_C_register);
	printf("---------------------------------------\n");

/* This function CONFIGURES the parity mode of the USART */
	printf("--parity mode--------------------------\n");
	parity_mode_set(&int_state, DISABLED);
	printf("parity_mode_set DISABLED   :");
	print_binary(int_state.control_C_register);
	parity_mode_set(&int_state, EVEN);
	printf("parity_mode_set EVEN       :");
	print_binary(int_state.control_C_register);
	parity_mode_set(&int_state, ODD);
	printf("parity_mode_set ODD        :");
	print_binary(int_state.control_C_register);
	printf("---------------------------------------\n");

/* This function CONFIGURES the stop bit number of the USART */
	printf("--stop bit set-------------------------\n");
	stop_bit_set(&int_state, ONE);
	printf("stop_bit_set ONE           :");
	print_binary(int_state.control_C_register);

	stop_bit_set(&int_state, TWO);
	printf("stop_bit_set TWO           :");
	print_binary(int_state.control_C_register);
	printf("---------------------------------------\n");

/* This function CONFIGURES the receiver mode of the USART */
	printf("--character size set-------------------\n");
	character_size_set(&int_state, BIT_5);
	printf("character_size_set BIT_5   :");
	print_binary(int_state.control_C_register);

	character_size_set(&int_state, BIT_6);
	printf("character_size_set BIT_6   :");
	print_binary(int_state.control_C_register);

	character_size_set(&int_state, BIT_7);
	printf("character_size_set BIT_7   :");
	print_binary(int_state.control_C_register);

	character_size_set(&int_state, BIT_8);
	printf("character_size_set BIT_8   :");
	print_binary(int_state.control_C_register);

	character_size_set(&int_state, BIT_9L);
	printf("character_size_set BIT_9L  :");
	print_binary(int_state.control_C_register);

	character_size_set(&int_state, BIT_9H);
	printf("character_size_set BIT_9H  :");
	print_binary(int_state.control_C_register);
	printf("---------------------------------------\n");

/* This function CONFIGURES the Master SPI USART data order to transmit MSB
 * first */
	if (data_order_MSB(&int_state) == USART_CFG_SUCCESS) {
		printf("data_order_MSB             :");
		print_binary(int_state.control_C_register);
	} else {
		printf("data_order_MSB             :");
		printf("USART_CFG_ACCESS_DENIED\n");
	}

/* This function CONFIGURES the Master SPI USART data order to transmit LSB
 * first */
	if (data_order_LSB(&int_state) == USART_CFG_SUCCESS) {
		printf("data_order_LSB             :");
		print_binary(int_state.control_C_register);
	}	else {
		printf("data_order_LSB             :");
		printf("USART_CFG_ACCESS_DENIED\n");
	}

/* This function CONFIGURES the USART clock phase for leading edge sampling */
	if (clock_phase_leading(&int_state) == USART_CFG_SUCCESS) {
		printf("clock_phase_leading        :");
		print_binary(int_state.control_C_register);
	}	else {
		printf("clock_phase_leading        :");
		printf("USART_CFG_ACCESS_DENIED\n");
	}

/* This function CONFIGURES the USART clock phase for trailing edge sampling */
	if (clock_phase_trailing(&int_state) == USART_CFG_SUCCESS) {
		printf("clock_phase_trailing       :");
		print_binary(int_state.control_C_register);
	} else {
		printf("clock_phase_trailing       :");
		printf("USART_CFG_ACCESS_DENIED\n");
	}

/* Disable MSPI mode */
	printf(">>>> DISABLE MSPI TO TRIGGER DENIED ACCESS <<<<\n");
	comm_mode_set(&int_state, ASYNCHRONOUS);
	printf("comm_mode_set ASYNCHRONOUS :");
	print_binary(int_state.control_C_register);

	if (data_order_MSB(&int_state) == USART_CFG_SUCCESS) {
		printf("data_order_MSB             :");
		print_binary(int_state.control_C_register);
	} else {
		printf("data_order_MSB             :");
		printf("USART_CFG_ACCESS_DENIED\n");
	}

	if (data_order_LSB(&int_state) == USART_CFG_SUCCESS) {
		printf("data_order_LSB             :");
		print_binary(int_state.control_C_register);
	}	else {
		printf("data_order_LSB             :");
		printf("USART_CFG_ACCESS_DENIED\n");
	}

	if (clock_phase_leading(&int_state) == USART_CFG_SUCCESS) {
		printf("clock_phase_leading        :");
		print_binary(int_state.control_C_register);
	}	else {
		printf("clock_phase_leading        :");
		printf("USART_CFG_ACCESS_DENIED\n");
	}

	if (clock_phase_trailing(&int_state) == USART_CFG_SUCCESS) {
		printf("clock_phase_trailing       :");
		print_binary(int_state.control_C_register);
	} else {
		printf("clock_phase_trailing       :");
		printf("USART_CFG_ACCESS_DENIED\n");
	}

/** Baudrate register configuration functions */
/* Baudrate configuration */
/* This function CONFIGURES the receiver mode of the USART */
	printf("\n########### Baudrate Register ###########\n");
	baudrate_set(&int_state, 9600);
	printf("baudrate_set Low Bytes       :");
	print_binary(int_state.baudrate_L_register);
	printf("baudrate_set High Bytes      :");
	print_binary(int_state.baudrate_H_register);

/** Control D register configuration functions */
	printf("\n########### Control Register D ##########\n");
/* These bits control the tolerance for the difference between the baud rates
 * between the two synchronizing devices when using Lin Constrained Auto-baud
 * mode */
	auto_baud_window_set(&int_state, WDW0);
	printf("auto_baud_window_set WDW0    :");
	print_binary(int_state.control_D_register);

	auto_baud_window_set(&int_state, WDW1);
	printf("auto_baud_window_set WDW1    :");
	print_binary(int_state.control_D_register);

	auto_baud_window_set(&int_state, WDW2);
	printf("auto_baud_window_set WDW2    :");
	print_binary(int_state.control_D_register);

	auto_baud_window_set(&int_state, WDW3);
	printf("auto_baud_window_set WDW3    :");
	print_binary(int_state.control_D_register);

/** Debug Control register function */
	printf("\n##### Debug Control Register #####\n");
/* This function enables debug run */
	debug_run_enable(&int_state);
	printf("debug_run_enable      :");
	print_binary(int_state.debug_control_register);
/* This function disables debug run */
	debug_run_disable(&int_state);
	printf("debug_run_disable     :");
	print_binary(int_state.debug_control_register);

/** IrDA Control register function */
	printf("\n####### IrDA Control Register #######\n");
/* This function enables IrDA Event input */
	irda_event_input_enable(&int_state);
	printf("irda_event_input_enable  :");
	print_binary(int_state.irda_control_register);
/* This function disables IrDA Event input */
	irda_event_input_disable(&int_state);
	printf("irda_event_input_disable :");
	print_binary(int_state.irda_control_register);

/** IRCOM Transmitter Pulse Length Control register function */
	printf("\n####### IRCOM Transmitter Pulse Length Control Register #######\n");
		printf("ircom_tx_pulse_length_set :\n");
/* This function sets IRCOM Transmitter pulse length */
	if (ircom_tx_pulse_length_set(&int_state, 0) ==
	    IRCOM_TX_BAUD_RATE_3OVER6_PERIOD) {
		printf("IRCOM_TX_BAUD_RATE_3OVER6_PERIOD ");
		printf("val=0    :");
		print_binary(int_state.ircom_tx_pulse_len_reg);
	}

	if (ircom_tx_pulse_length_set(&int_state, 100)
	    == IRCOM_TX_FIXED_PULSE_LENGTH) {
		printf("IRCOM_TX_FIXED_PULSE_LENGTH      ");
		printf("val=100  :");
		print_binary(int_state.ircom_tx_pulse_len_reg);
	}

	if (ircom_tx_pulse_length_set(&int_state, 255)
	    == IRCOM_TX_PULSE_CODING_DISABLED) {
		printf("IRCOM_TX_PULSE_CODING_DISABLED   ");
		printf("val=255  :");
		print_binary(int_state.ircom_tx_pulse_len_reg);
	}

/** IRCOM Receiver Pulse Length Control register function */
	printf("\n####### IRCOM Receiver Pulse Length Control Register #######\n");
		printf("ircom_rx_pulse_length_set :\n");
/* This function sets IRCOM Receiver pulse length */
	if (ircom_rx_pulse_length_set(&int_state, 0x00)
	    == IRCOM_RX_FILTERING_DISABLED) {
		printf("IRCOM_RX_FILTERING_DISABLED              ");
		printf(" val=0x00    :");
		print_binary(int_state.ircom_rx_pulse_len_reg);
	}
	if (ircom_rx_pulse_length_set(&int_state, 100)
	    == IRCOM_RX_FILTERING_ENABLED) {
		printf("IRCOM_RX_FILTERING_ENABLED               ");
		printf(" val=100     :");
		print_binary(int_state.ircom_rx_pulse_len_reg);
	}
	if (ircom_rx_pulse_length_set(&int_state, 0xFF)
	    == IRCOM_RX_UNKNOWN_FILTERING_CONFIGURATION) {
		printf("IRCOM_RX_UNKNOWN_FILTERING_CONFIGURATION ");
		printf(" val=0xFF    :");
		print_binary(int_state.ircom_rx_pulse_len_reg);
	}
	return 0;
}
