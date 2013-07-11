/*
 * addresses.h
 *
 *  Created on: 14.06.2010
 *      Author: tobi
 */


#ifndef COCO_ADDRESSES_H
#define COCO_ADDRESSES_H


typedef enum
{
   ADDR_FC = 1,
   ADDR_NC = 2,
   ADDR_M3 = 3
} mk_address_t;


char addr_encode(mk_address_t address);

mk_address_t addr_decode(char address_char);


#endif /* COCO_ADDRESSES_H */
