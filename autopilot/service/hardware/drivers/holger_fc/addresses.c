/*
 * addresses.c
 *
 *  Created on: 14.06.2010
 *      Author: tobi
 */


#include "addresses.h"


static const char ADDRESS_BASE = 'a';


char addr_encode(mk_address_t address)
{
   return (char)(ADDRESS_BASE + address);
}


mk_address_t addr_decode(char address_char)
{
   return address_char - ADDRESS_BASE;
}
