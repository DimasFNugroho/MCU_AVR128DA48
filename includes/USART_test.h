#include <stdbool.h>
#include <stdint.h>

#ifdef _AVR128DA64_H

#define RX_COMPLETE_INT_FLAG 0x80
#define TX_COMPLETE_INT_FLAG 0x40
#define DATA_REG_EMPTY_INT_FLAG 0x20
#define RX_START_FRAME_INT_FLAG 0x10
#define INCONSISTENT_SYNC_INT_FLAG 0x08
#define BREAK_DETECTED_FLAG 0x02
#define WAIT_FOR_BREAK_CONTROL 0x01

/* Control A register bits setup */
#define RX_COMPLETE_INT_BITS 0x80
#define TX_COMPLETE_INT_BITS 0x40
#define DATA_REG_EMPTY_INT_BITS 0x20
#define RX_START_FRAME_INT_BITS 0x10
#define LOOP_BACK_MODE_BITS 0x08
#define AUTO_BAUD_ERR_INT_BITS 0x02
#define RS_485_MODE_BITS 0x01
#define REG_VAL_ZERO 0x00

/* Control B register bits setup */
#define RX_ENABLE_BITS 0x80
#define TX_ENABLE_BITS 0x40
#define START_FRAME_DETECT_ENABLE_BITS 0x10
#define OPEN_DRAIN_MODE_BITS 0x08

#define RX_MODE_CLEAR_BITS 0x06 
#define RX_MODE_NORMAL_BITS 0xF9 
#define RX_MODE_CLK2X_BITS 0xFB
#define RX_MODE_GENAUTO_BITS 0xFD
#define RX_MODE_LINAUTO_BITS 0xFF

#define MULTI_PROCESSOR_COMM_BITS 0x01

/* Control C register bits setup */
#define COMM_MODE_CLEAR_BITS 0xC0
#define COMM_MODE_ASYNCHRONOUS_BITS 0x3F
#define COMM_MODE_SYNCHRONOUS_BITS 0x7F
#define COMM_MODE_IRCOM_BITS 0xBF
#define COMM_MODE_MSPI_BITS 0xFF

#define PARITY_MODE_CLEAR_BITS 0x30
#define PARITY_MODE_DISABLED_BITS 0xCF
#define PARITY_MODE_EVEN_BITS 0xEF
#define PARITY_MODE_ODD_BITS 0xFF
#define STOPBIT_CLEAR_BITS 0x08 
#define ONE_STOPBIT_BITS 0xF7
#define TWO_STOPBIT_BITS 0xFF


#define CHAR_SIZE_CLEAR_BITS 0x07
#define CHAR_SIZE_5BIT_BITS 0xF8
#define CHAR_SIZE_6BIT_BITS 0xF9
#define CHAR_SIZE_7BIT_BITS 0xFA
#define CHAR_SIZE_8BIT_BITS 0xFB
#define CHAR_SIZE_9LBIT_BITS 0xFE
#define CHAR_SIZE_9HBIT_BITS 0xFF


#define DATA_ORDER_CLEAR_BITS 0x04
#define DATA_ORDER_MSB_BITS 0xFB
#define DATA_ORDER_LSB_BITS 0xFF

#define CLOCK_PHASE_CLEAR_BITS 0x02
#define CLOCK_PHASE_LEAD_EDGE_BITS 0xFD
#define CLOCK_PHASE_TRAIL_EDGE_BITS 0xFF


#define AUTOBAUD_WINDOW_CLEAR_BITS 0xC0
#define AUTOBAUD_WINDOW_WDW0_BITS 0x3F
#define AUTOBAUD_WINDOW_WDW1_BITS 0x7F
#define AUTOBAUD_WINDOW_WDW2_BITS 0xBF
#define AUTOBAUD_WINDOW_WDW3_BITS 0xFF

#define DEBUG_RUN_BITS 0x01
#define IRDA_CONTROL_REG_BITS 0x01
#endif
typedef uint8_t usart_8bits;
typedef bool usart_bool;
typedef void (*interrupt_set)(usart_8bits);

typedef enum {
	USART_CFG_SUCCESS,
	USART_CFG_FAIL,
	USART_CFG_ACCESS_DENIED
} usart_config_status;

typedef struct{
	usart_8bits rx_low_byte_register;
	usart_8bits rx_high_byte_register;
	usart_8bits tx_low_byte_register;
	usart_8bits tx_high_byte_register;
	usart_8bits status_register;
	usart_8bits control_A_register;
	usart_8bits control_B_register;
	usart_8bits control_C_register;
	usart_bool	mspi_enable;

	usart_8bits baudrate_H_register;
	usart_8bits baudrate_L_register;

	usart_8bits control_D_register;

	usart_8bits debug_control_register;
	usart_8bits irda_control_register;
	usart_8bits ircom_tx_pulse_len_reg;
	usart_8bits ircom_rx_pulse_len_reg;

	usart_8bits (*usart_transmit)(const usart_8bits *);
	usart_8bits (*usart_receive)(usart_8bits *);
} usart_register;

/* initialize the usart instance */
void init_usart(usart_register *);

/** USART Status Register functions */
/* Status register */
typedef enum {
	WAIT_FOR_BREAK_OFF = 0x00,
	WAIT_FOR_BREAK_ON = 0x01,
	BREAK_DETECTED = 0x02,
	NOTHING_DETECTED = 0x04,
	INCONSISTENT_SYNCHRONIZATION_FIELD =0x08,
	USART_RECEIVE_START = 0x10,
	DATA_REGISTER_EMPTY = 0x20,
	USART_TRANSMIT_COMPLETE = 0x40,
	USART_RECEIVE_COMPLETE = 0x80
} usart_status;

/* This function READS the Receive Complete Interrupt status */
usart_status get_rx_complete_int_status(usart_register*, uint8_t);

/* This function READS the Transmit Complete Interrupt status*/
usart_status get_tx_complete_int_status(usart_register*, uint8_t);

/* This function READS the Data Register Empty Interrupt status */
usart_status get_data_reg_empty_int_status(usart_register*, uint8_t);

/* This function READS the Receive Start Interrupt status */
usart_status get_rx_start_frame_int_status(usart_register*, uint8_t);

/* This function READS the Inconsistent Synchronization Field Interrupt
 * status */
usart_status get_inconsistent_sync_field_int_status(usart_register*, uint8_t);

/* This function READS the Break Detected status */
usart_status get_break_detected_status(usart_register*, uint8_t);

/* This function ENABLES the "Wait For Break" */
usart_status wait_for_break_on(usart_register*, uint8_t);
/* This function DISABLES the "Wait For Break" */
usart_status wait_for_break_off(usart_register*, uint8_t);

/** Control A Register configuration functions */
/* This function ENABLES the Receive Complete Interrupt */
void rx_complete_int_on(usart_register*);
/* This function DISABLES the Receive Complete Interrupt */
void rx_complete_int_off(usart_register*);

/* This function ENABLES the Transmit Complete Interrupt */
void tx_complete_int_on(usart_register*);
/* This function DISABLES the Transmit Complete Interrupt */
void tx_complete_int_off(usart_register*);

/* This function ENABLES the Data Register Empty Interrupt */
void data_reg_empty_int_on(usart_register*);
/* This function DISABLES the Data Register Empty Interrupt */
void data_reg_empty_int_off(usart_register*);

/* This function ENABLES the Receive Start Interrupt */
void rx_start_frame_int_on(usart_register*);
/* This function DISABLES the Receive Start Interrupt */
void rx_start_frame_int_off(usart_register*);

/* This function ENABLES the Data Register Empty Interrupt */
void loop_back_mode_on(usart_register*);
/* This function DISABLES the Data Register Empty Interrupt */
void loop_back_mode_off(usart_register*);

/* This function ENABLES the Data Register Empty Interrupt */
void auto_baud_err_int_on(usart_register*);
/* This function DISABLES the Data Register Empty Interrupt */
void auto_baud_err_int_off(usart_register*);

/* This function ENABLES the Data Register Empty Interrupt */
void rs_485_mode_on(usart_register*);
/* This function DISABLES the Data Register Empty Interrupt */
void rs_485_mode_off(usart_register*);

/** Control B Register configuration functions */
/* This function ENABLES the USART receiver */
void rx_on(usart_register*);
/* This function DISABLES the USART receiver */
void rx_off(usart_register*);

/* This function ENABLES the USART transmitter */
void tx_on(usart_register*);
/** This function DISABLES the USART transmitter */
void tx_off(usart_register*);

/* This function ENABLES the Start-of-Frame detection */
void start_frame_on(usart_register*);
/* This function DISABLES the Start-of-Frame detection */
void start_frame_off(usart_register*);

/* This function ENABLES the Start-of-Frame detection */
void start_frame_on(usart_register*);
/* This function DISABLES the USART transmitter */
void start_frame_off(usart_register*);

/* This function ENABLES the open drain mode */
void open_drain_mode_on(usart_register*);
/* This function DISABLES the open drain mode */
void open_drain_mode_off(usart_register*);

/* USART Receiver modes*/
typedef enum {
	/* Normal speed mode */
	NORMAL,
	/* Double-Speed mode */
	CLK2X,
	/* Generic Auto-Baud mode */
	GENAUTO,
	/* LIN Constrained Auto-Baud mode */
	LINAUTO
} rxmode;
/* This function CONFIGURES the receiver mode of the USART */
void rx_mode_set(usart_register*, rxmode);

/* This function ENABLES the open drain mode */
void multi_processor_comm_on(usart_register*);
/* This function DISABLES the open drain mode */
void multi_processor_comm_off(usart_register*);

/** Control C register configuration functions */
/* USART communcation modes */
typedef enum {
	/* Asynchronous USART */
	ASYNCHRONOUS,
	/* Synchronous USART */
	SYNCHRONOUS,
	/* Infrared Communication */
	IRCOM,
	/* Master SPI */
	MSPI
} comm_mode;
/* This function CONFIGURES the receiver mode of the USART */
void comm_mode_set(usart_register*, comm_mode);

/* USART parity modes */
typedef enum {
	/* Disabled parity */
	DISABLED,
	/* RESERVED */
	/* EVEN parity */
	EVEN,
	/* ODD parity */
	ODD
} parity_mode;
/* This function CONFIGURES the parity mode of the USART */
void parity_mode_set(usart_register*, parity_mode);

/* USART stop bit configuration */
typedef enum {
	/* one stop bit */
	ONE,
	/* two stop bit */
	TWO
} stop_bit;
/* This function CONFIGURES the stop bit number of the USART */
void stop_bit_set(usart_register*, stop_bit);

/* USART parity modes */
typedef enum {
	BIT_5,
	BIT_6,
	BIT_7,
	BIT_8,
	/* Low byte first */
	BIT_9L,
	/* High byte first */
	BIT_9H
} character_size;
/* This function CONFIGURES the receiver mode of the USART */
void character_size_set(usart_register*, character_size);

/* This function CONFIGURES the Master SPI USART data order to transmit MSB
 * first */
usart_config_status data_order_MSB(usart_register*);
/* This function CONFIGURES the Master SPI USART data order to transmit LSB
 * first */
usart_config_status data_order_LSB(usart_register*);

typedef enum {
	LEADING_EDGE_SAMPLING,
	TRAILING_EDGE_SAMPLING,
} usart_M_SPI_clock_phase;

/* This function CONFIGURES the USART clock phase for leading edge sampling */
usart_config_status clock_phase_leading(usart_register*);
/* This function CONFIGURES the USART clock phase for trailing edge sampling */
usart_config_status clock_phase_trailing(usart_register*);

/** Baudrate register configuration functions */
/* Baudrate configuration */
typedef uint16_t usart_16bits;
usart_16bits usart_baudrate;
/* This function CONFIGURES the receiver mode of the USART */
void baudrate_set(usart_register*, usart_16bits);

/** Control D register configuration functions */
/* Auto Baud Window Tolerances */
typedef enum {
	/* 32±6 (18% tolerance) */
	WDW0,
	/* 32±5 (15% tolerance) */
	WDW1,
	/* 32±7 (21% tolerance) */
	WDW2,
	/* 32±8 (25% tolerance) */
	WDW3,
} auto_baud_window_size;
/* These bits control the tolerance for the difference between the baud rates
 * between the two synchronizing devices when using Lin Constrained Auto-baud
 * mode */
void auto_baud_window_set(usart_register*, auto_baud_window_size);

/** Debug Control register function */
/* This function enables debug run */
void debug_run_enable(usart_register*);
/* This function disables debug run */
void debug_run_disable(usart_register*);

/** IrDA Control register function */
/* This function enables IrDA Event input */
void irda_event_input_enable(usart_register*);
/* This function disables IrDA Event input */
void irda_event_input_disable(usart_register*);

/** IRCOM Transmitter Pulse Length Control register function */
#define TX_FIXED_PULSE_LENGTH_MIN 0x01
#define TX_FIXED_PULSE_LENGTH_MAX 0xFE
typedef usart_8bits transmit_pulse_length;
typedef enum {
	IRCOM_TX_BAUD_RATE_3OVER6_PERIOD,
	IRCOM_TX_FIXED_PULSE_LENGTH,
	IRCOM_TX_PULSE_CODING_DISABLED
} transmit_pulse_length_flag;

/* This function disables IrDA Event input */
transmit_pulse_length_flag ircom_tx_pulse_length_set(usart_register*,
                                                     transmit_pulse_length);

/** IRCOM Receiver Pulse Length Control register function */
#define RX_FIXED_PULSE_LENGTH_MIN 0x01
#define RX_FIXED_PULSE_LENGTH_MAX 0x7F
typedef usart_8bits receive_pulse_length;
typedef enum {
	IRCOM_RX_FILTERING_DISABLED,
	IRCOM_RX_FILTERING_ENABLED,
	IRCOM_RX_UNKNOWN_FILTERING_CONFIGURATION
} receive_pulse_length_flag;

/* This function disables IrDA Event input */
receive_pulse_length_flag ircom_rx_pulse_length_set(usart_register*,
                                                    receive_pulse_length);
