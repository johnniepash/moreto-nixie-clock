// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               OWIDeviceSpecific.h
* \li Compiler:           IAR EWAAVR 3.20a
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All AVRs with UART or USART module. 
*
* \li Application Note:   AVR318 - Dallas 1-Wire(R) master.
*                         
*
* \li Description:        Device specific defines that expands to correct
*                         register and bit definition names for the selected
*                         device.
*
*                         $Revision: 1.7 $
*                         $Date: Thursday, August 19, 2004 14:27:16 UTC $
****************************************************************************/




#define OWI_UART_STATCTRL_REG_A     UCSR0A
#define OWI_UART_STATCTRL_REG_B     UCSR0B
#define OWI_UART_STATCTRL_REG_C     UCSR0C
#define OWI_UART_DATA_REGISTER      UDR0
#define OWI_UART_BAUD_RATE_REG_L    UBRR0L
#define OWI_UART_BAUD_RATE_REG_H    UBRR0H

#define OWI_U2X                     U2X0
#define OWI_RXEN                    RXEN0
#define OWI_TXEN                    TXEN0
#define OWI_RXCIE                   RXCIE0
#define OWI_UCSZ1                   UCSZ01
#define OWI_UCSZ0                   UCSZ00
#define OWI_UDRIE                   UDRIE0
#define OWI_FE                      FE0
#define OWI_RXC                     RXC0

#define OWI_UART_RXC_VECT           USART_RX_vect
#define OWI_UART_UDRE_VECT          USART_UDRE_vect
