/*
 * crc.h
 *
 * http://mikrokopter.de/ucwiki/en/SerialProtocol#head-58f4dd9dd018e8c7f237035c08aad0269ea05f12
 *
 *  Created on: 14.06.2010
 *      Author: tobi
 */


#ifndef COCO_CRC_H
#define COCO_CRC_H


#include "frame.h"


typedef unsigned int crc_t;


crc_t calc_crc(const char *data, const unsigned int len);

int crc_ok(crc_t crc, const char *crc_chars);

int calc_crc_chars(char *crc_chars, const crc_t crc);


#endif /* COCO_CRC_H */
