/*
 * File:   main.c
 * Author: jarin
 *
 * Created on August 26, 2016, 10:23 PM
 */

#include <xc.h>

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

#define	_XTAL_FREQ  16000000
#define	BAUD_RATE   38400

unsigned char usart_rx_rdy (void);
unsigned char usart_rx_b (void);
void usart_tx_str (char * data);
void usart_tx_b (unsigned char dat);

void dly_ms(unsigned int ms);

void c2_send_bits (unsigned char data, unsigned char len);
void c2_pulse_clk (void);
unsigned char c2_read_bits (unsigned char len);

void c2_rst (void);
void c2_write_addr (unsigned char addr);
unsigned char c2_read_addr (void);
unsigned char c2_read_data (void);
void c2_write_data (unsigned char addr);

unsigned char c2_init_PI (void);
unsigned char c2_read_flash_block (unsigned int addr, unsigned char * data, unsigned char len);
unsigned char c2_poll_bit_low (unsigned char mask);
unsigned char c2_poll_bit_high (unsigned char mask);
unsigned char c2_write_flash_block (unsigned int addr, unsigned char * data, unsigned char len);
unsigned char c2_erase_device (void);

unsigned char rx_state_machine (unsigned char state, unsigned char rx_char);


#define	C2_D_O	LATCbits.LATC0
#define	C2_D_I	PORTCbits.RC0
#define	C2_D_T	TRISCbits.TRISC0

#define	C2_C_O	LATCbits.LATC1
#define	C2_C_I	PORTCbits.RC1
#define	C2_C_T	TRISCbits.TRISC1

#define	LED		LATAbits.LATA2
#define	LED_T	TRISAbits.TRISA2


unsigned int i;
unsigned char retval;
unsigned char flash_array[34],flash_array2[34],flash_array3[34];
unsigned char rx_message[250],rx_message_ptr;
unsigned char rx,main_state,bytes_to_receive,rx_state;
unsigned char flash_buffer[130];
unsigned long addr;

void main(void)
{
	TRISCbits.TRISC1 = 0;
	TRISCbits.TRISC0 = 0;
	LED_T = 0;
	OSCCON = 0x78;
	ANSELA = 0;
	ANSELC = 0;
	TXSTA = 0x24;
	RCSTA = 0x90;
	BRG16 = 0;
	SPBRGL = (_XTAL_FREQ/BAUD_RATE/16) - 1;    
	dly_ms(300);
  while (1)
    {
    if (usart_rx_rdy())
      {
      rx = usart_rx_b();
      rx_state = rx_state_machine (rx_state,rx);
      if (rx_state==3)
        {
		if (rx_message[0]==0x01)
          {
			c2_init_PI();
          usart_tx_b (0x81);
			LED = 1;
          rx_state = 0;
          }
        if (rx_message[0]==0x02)
          {
          c2_rst();
          usart_tx_b (0x82);
		  LED = 0;
          rx_state = 0;
          }
        if (rx_message[0]==0x03)
          {
          addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
          for (i=0;i<rx_message[2];i++)
            {
            flash_buffer[i] = rx_message[i+6];
            }
		  c2_write_flash_block(addr,flash_buffer,rx_message[2]);
          usart_tx_b (0x83);
          rx_state = 0;
          }
        if (rx_message[0]==0x04)
          {
          c2_erase_device();
          usart_tx_b (0x84);
          rx_state = 0;
          }
        if (rx_message[0]==0x05)
          {
          usart_tx_b (0x85);
          addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
		  c2_read_flash_block(addr,flash_buffer,rx_message[2]);
          for (i=0;i<rx_message[2];i++)
            {
            usart_tx_b (flash_buffer[i]);
            }
          rx_state = 0;
          }
		
        if (rx_message[0]==0x06)
          {
			c2_write_addr(rx_message[3]);
			c2_write_data(rx_message[4]);
          usart_tx_b (0x86);
          rx_state = 0;
          }
		
		
        if (rx_message[0]==0x07)
          {
			c2_write_addr(rx_message[3]);
			usart_tx_b (c2_read_data());
          usart_tx_b (0x87);
          rx_state = 0;
          }
		 
        }//end of rx state 3
      }      
    }
	
	/*
	flash_array2[0] = 0x75;
	flash_array2[1] = 0x80;
	flash_array2[2] = 0x00;
	
	flash_array2[3] = 0x75;
	flash_array2[4] = 0xE3;
	flash_array2[5] = 0x40;
	
	flash_array2[6] = 0x80;
	flash_array2[7] = 0xFE;
	while (1)
		{
		TRISCbits.TRISC1 = 0;
		TRISCbits.TRISC0 = 0;
		c2_init_PI();
		c2_erase_device();
		c2_read_flash_block(0,flash_array,16);
		c2_write_flash_block(0,flash_array2,16);
		c2_read_flash_block(0,flash_array3,16);
		c2_rst();
		TRISCbits.TRISC1 = 1;
		TRISCbits.TRISC0 = 1;
		}
	
*/

}


unsigned char rx_state_machine (unsigned char state, unsigned char rx_char)
{
if (state==0)
  {
    rx_message_ptr = 0;
    rx_message[rx_message_ptr++] = rx_char;
    return 1;
  }
if (state==1)
  {
    bytes_to_receive = rx_char;
    rx_message[rx_message_ptr++] = rx_char;
    if (bytes_to_receive==0) return 3;
    return 2;
  }
if (state==2)
  {
    rx_message[rx_message_ptr++] = rx_char;
    bytes_to_receive--;
    if (bytes_to_receive==0) return 3;
  }
return state;  
}




unsigned char c2_init_PI (void)
	{
	c2_rst();
	c2_write_addr(0x02);
	c2_write_data(0x02);
	c2_write_data(0x04);
	c2_write_data(0x01);
	dly_ms(20);
	return 0;
	}

#define	INBUSY		0x02
#define	OUTREADY	0x01

unsigned char c2_erase_device (void)
	{
	unsigned char retval;
	c2_write_addr(0xB4);
	c2_write_data(0x03);
	c2_poll_bit_low(INBUSY);
	c2_poll_bit_high(OUTREADY);
	retval = c2_read_data();
	c2_write_data(0xDE);
	c2_poll_bit_low(INBUSY);	
	c2_write_data(0xAD);
	c2_poll_bit_low(INBUSY);	
	c2_write_data(0xA5);
	c2_poll_bit_low(INBUSY);	
	c2_poll_bit_high(OUTREADY);
	retval = c2_read_data();
	}

unsigned char c2_write_flash_block (unsigned int addr, unsigned char * data, unsigned char len)
	{
	unsigned char retval,i;
	c2_write_addr(0xB4);
	c2_write_data(0x07);
	c2_poll_bit_low(INBUSY);
	c2_poll_bit_high(OUTREADY);
	retval = c2_read_data();
	c2_write_data(addr>>8);
	c2_poll_bit_low(INBUSY);
	c2_write_data(addr&0xFF);
	c2_poll_bit_low(INBUSY);
	c2_write_data(len);
	c2_poll_bit_low(INBUSY);	
	c2_poll_bit_high(OUTREADY);
	retval = c2_read_data();	
	for (i=0;i<len;i++)
		{
		c2_write_data(data[i] );
		c2_poll_bit_low(INBUSY);
		}	
	c2_poll_bit_high(OUTREADY);
	}

unsigned char c2_read_flash_block (unsigned int addr, unsigned char * data, unsigned char len)
	{
	unsigned char retval,i;
	c2_write_addr(0xB4);
	c2_write_data(0x06);
	c2_poll_bit_low(INBUSY);
	c2_poll_bit_high(OUTREADY);
	retval = c2_read_data();
	c2_write_data(addr>>8);
	c2_poll_bit_low(INBUSY);
	c2_write_data(addr&0xFF);
	c2_poll_bit_low(INBUSY);
	c2_write_data(len);
	c2_poll_bit_low(INBUSY);
	c2_poll_bit_high(OUTREADY);
	retval = c2_read_data();
	for (i=0;i<len;i++)
		{
		c2_poll_bit_high(OUTREADY);
		retval = c2_read_data();
		data[i] = retval;
		}
	return i;
	}


unsigned char c2_poll_bit_low (unsigned char mask)
	{
	unsigned char retval;
	retval = c2_read_addr();
	while (retval&mask) retval = c2_read_addr();
	}

unsigned char c2_poll_bit_high (unsigned char mask)
	{
	unsigned char retval;
	retval = c2_read_addr();
	while ((retval&mask)==0) retval = c2_read_addr();
	}


void c2_rst (void)
	{
	C2_C_O = 0;
	__delay_us(100);
	C2_C_O = 1;
	__delay_us(100);	
	}

void c2_write_addr (unsigned char addr)
	{
	c2_send_bits(0x0,1);
	c2_send_bits(0x3,2);
	c2_send_bits(addr,8);
	c2_send_bits(0x0,1);	
	}

unsigned char c2_read_addr (void)
	{
	unsigned char retval;
	c2_send_bits(0x0,1);
	c2_send_bits(0x2,2);
	retval = c2_read_bits(8);
	c2_send_bits(0x0,1);
	return retval;	
	}

unsigned char c2_read_data (void)
	{
	unsigned char retval;
	c2_send_bits(0x0,1);
	c2_send_bits(0x0,2);
	c2_send_bits(0x0,2);
	retval = 0;
	while (retval==0)
		retval = c2_read_bits(1);
	retval = c2_read_bits(8);
	c2_send_bits(0x0,1);
	return retval;
	}

void c2_write_data (unsigned char data)
	{
	unsigned char retval;
	c2_send_bits(0x0,1);
	c2_send_bits(0x1,2);
	c2_send_bits(0x0,2);
	c2_send_bits(data,8);
	retval = 0;
	while (retval==0)
		retval = c2_read_bits(1);
	c2_send_bits(0x0,1);
	
	}

unsigned char usart_rx_rdy (void)
{
if (RCIF)
	return 1;
else
	return 0;
}

unsigned char usart_rx_b (void)
{
return RCREG;
}

void usart_tx_str (char * data)
{
while (*data!=0) usart_tx_b(*data++);
}

void usart_tx_b (unsigned char dat)
{
while (TRMT==0);
TXREG = dat;
}
void dly_ms(unsigned int ms)
{
unsigned int i;
for (i=0;i<ms;i++) __delay_ms(1);
}


void c2_send_bits (unsigned char data, unsigned char len)
	{
	unsigned char i;
	C2_D_T = 0;
	for (i=0;i<len;i++)
		{
		if (data&0x01) C2_D_O = 1;
		else	C2_D_O = 0;
		c2_pulse_clk ();
		data = data >> 1;
		}
	}

unsigned char c2_read_bits (unsigned char len)
	{
	unsigned char i,data,mask;
	mask = 0x01 << (len-1);
	data = 0;
	C2_D_T = 1;
	for (i=0;i<len;i++)
		{
		c2_pulse_clk ();
		data = data >> 1;
		if (C2_D_I==1) data = data | mask;
		}
	C2_D_T = 0;
	return data;
	}



void c2_pulse_clk (void)
	{
	C2_C_T = 0;
	C2_C_O = 0;
	C2_C_O = 0;
	C2_C_O = 0;
	C2_C_O = 1;
	}