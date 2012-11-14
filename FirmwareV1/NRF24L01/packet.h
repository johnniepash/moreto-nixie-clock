/*
 * packet.h
 *
 *  Created on: 26-Apr-2009
 *      Author: Neil MacMillan
 */

#ifndef PACKET_H_
#define PACKET_H_


#include <avr/io.h>

typedef enum _at
{
	NONE,
	BRAKE_ON,	// Brake the motors.
	BRAKE_OFF,
	STOP,	// Stop the motors with zero value PWM.
	MODE_ABSOLUTE,
	MODE_INCREMENTAL,
	TOGGLE_AV,
	TOGGLE_GPS,
} ACTIONS;

/*****					Add labels for the packet types to the enumeration					*****/

typedef enum _pt
{
	TXVALUES,
	RXSTATUS,
	ACK,
} PACKET_TYPE;

/*****							Construct payload format structures							*****/

// structures must be 29 bytes long or less.

typedef struct _txv
{
	uint8_t messageid;	// 1 byte
//	uint8_t address[5];// 5 bytes
	int16_t ud_dir;	// 2 bytes
	int16_t lr_dir;	// 2 bytes
	int16_t ud_esq;	// 2 bytes
	int16_t lr_esq;	// 2 bytes
	ACTIONS action;	// 1 byte
//	uint8_t messagecontent[23];
} pf_txvalues_t;

typedef struct _rxst
{
	uint8_t messageid; // 1 byte
	uint16_t batt_v;   // 2 bytes (battery voltage)
	uint16_t batt_i;   // 2 bytes (battery load current)
	uint8_t gps_st;    // 1 byte (gps status)
	uint8_t lost_packtes; // 1 byte (lost pakets rate)
} pf_rxstatus_t;

typedef struct _ack
{
	uint8_t messageid;
	int16_t veloc_dir;
	int16_t veloc_esq;
} pf_ack_t;

/*****							Add format structures to the union							*****/

/// The application-dependent packet format.  Add structures to the union that correspond to the packet types defined
/// in the PACKET_TYPE enumeration.  The format structures may not be more than 29 bytes long.  The _filler array must
/// be included to ensure that the union is exactly 29 bytes long.
typedef union _pf
{
	uint8_t _filler[29];	// make sure the packet is exactly 32 bytes long - this array should not be accessed directly.
	pf_txvalues_t txvalues;
	pf_rxstatus_t rxstatus;
	pf_ack_t ack;
} payloadformat_t;

/*****						Leave the radiopacket_t structure alone.						*****/

typedef struct _rp
{
	PACKET_TYPE type;
	uint16_t timestamp;
	payloadformat_t payload;
} radiopacket_t;

#endif /* PACKET_H_ */
