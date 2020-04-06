/***********************************************************************************************//**
 * \file   vm_def.h
 * \brief  vendor model definition
 ***************************************************************************************************
 * <b> (C) Copyright 2018 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/
#ifndef VM_DEF_H
#define VM_DEF_H

#define VENDOR_ID				0x02FF
#define SERVER_MODEL_ID			0x1111
#define CLIENT_MODEL_ID			0x2222

#define NUM_OPCODES				4

typedef enum {
	pub_addr_get = 0x1,
	pub_addr_st,
	user_name,
	congrat
}opcodes_t;


#endif
