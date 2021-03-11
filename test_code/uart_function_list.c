typedef unsigned char uchar;


/**
 * DESCRIPTION:
 * set to 8-N-2 format
 * set baudrate
 * 
 * CONFUSION:
 * What is myErrors ?
 */
void mcu_uart_init(uchar ch);

/**
 * DESCRIPTION:
 * Set the U2X0 bit to double the USART Transmission speed.
 * The most significant bits are set to zero.
 * Only the Less Significant Bits are used as baudrate settings.
 * Set baudrate based on "long baud".
 * 
 * CONFUSION:
 * what is ch ? it is not used anywhere in this function. Why?
 */
void mcu_uart_set_baud(uchar ch, long baud);

/**
 * DESCRIPTION:
 * set variable error equals to the global variable myErrors.
 * the global variable is set to zero.
 * the function return the value of the variable "errors"
 * 
 * CONFUSION:
 * what is ch ? it is not used anywhere in this function. Why?
 */
uchar mcu_uart_get_errors(uchar ch);

/**
 * DESCRIPTION:
 * Set the parity of the uart based on the value of "uchar parity".
 * 
 * 
 * CONFUSION:
 * what is ch ? it is not used anywhere in this function. Why?
 *
 * Syntax error (line 1180):
 * UCSR0C = (UCSR0C & ~0x30) ) | (parity_bits[parity] << UPM00);
 */
const uchar parity_bits[] = {0x00,0x03,0x02};

void mcu_uart_set_parity(uchar ch,uchar parity);

void mcu_uart_set_bits(uchar ch,uchar bits);

void mcu_uart_tx_enable(uchar ch,char state);

char mcu_uart_tx_ready(uchar ch);

void mcu_uart_tx_c(uchar ch,uchar c);

void  mcu_uart_tx_break(uchar ch,char state);

void mcu_uart_rx_enable(uchar ch,char state);

uchar mcu_uart_rx_c(uchar ch);

char mcu_uart_rx_has(uchar ch);

char mcu_uart_rx_break(uchar ch);
