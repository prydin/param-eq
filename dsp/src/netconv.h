/**
 * @file    netconv.h
 * @brief   Header file for the netconv library
 * @author  Andreas Grommek
 * @version 1.0.0
 * @date    2021-05-27
 * 
 * @section license License
 * 
 * The MIT Licence (MIT)
 * 
 * Copyright (c) 2021 Andreas Grommek
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
#ifndef netconv_h
#define netconv_h

/* 
 * Do not include Arduino.h to keep library usable for other platforms.
 * Include only standard C++ headers instead.
*/
#include <stddef.h> // needed for size_t data type
#include <stdint.h> // needed for uint8_t data type
#include <string.h> // needed for memcpy()


/* Conversions from host byte order to network byte order: htonX() */

/* Take single number as argument, return single number. */
uint64_t htonll(uint64_t hostlonglong);
uint32_t htonl(uint32_t hostlong);
uint16_t htons(uint16_t hostshort);
uint32_t htonf(float    hostfloat);
#if (__SIZEOF_DOUBLE__ == 8)
uint64_t htond(double   hostdouble);
#endif
    
/*
 * Take single number as argument, write result to array.
 * This version is probably used to fill a buffer before 
 * sending data over a network.
 */
void htonll(uint64_t hostlonglong, uint8_t netbytes[]);
void htonl(uint32_t hostlong, uint8_t netbytes[]);
void htons(uint16_t hostshort, uint8_t netbytes[]);
void htonf(float hostfloat, uint8_t netbytes[]);
#if (__SIZEOF_DOUBLE__ == 8)
void htond(double hostdouble, uint8_t netbytes[]);
#endif
    
/* Take input value from array, return single number. */
uint64_t htonll(uint8_t hostbytes[]);
uint32_t htonl(uint8_t hostbytes[]);
uint16_t htons(uint8_t hostbytes[]);
uint32_t htonf(uint8_t hostbytes[]);
#if (__SIZEOF_DOUBLE__ == 8)
uint64_t htond(uint8_t hostbytes[]);
#endif

/* Conversions from network byte order to host byte order: ntohX() */

/* Take single number as argument, return single number. */
uint64_t ntohll(uint64_t netlonglong);
uint32_t ntohl(uint32_t netlong);
uint16_t ntohs(uint16_t netshort);
float    ntohf(uint32_t netfloat);
#if (__SIZEOF_DOUBLE__ == 8)
double   ntohd(uint64_t netdouble);
#endif

/* Take single number as argument, write result to array. */
void ntohll(uint64_t netlonglong, uint8_t hostbytes[]);
void ntohl(uint32_t netlong, uint8_t hostbytes[]);
void ntohs(uint16_t netshort, uint8_t hostbytes[]);
void ntohf(uint32_t netfloat, uint8_t hostbytes[]);
#if (__SIZEOF_DOUBLE__ == 8)
void ntohd(uint64_t netdouble, uint8_t hostbytes[]);
#endif

/*
 * Take input value from array, return single number.
 * This version is probably used to convert bytes in a buffer received
 * over a network back to a single number in host byte order.
 */
uint64_t ntohll(uint8_t netbytes[]);
uint32_t ntohl(uint8_t netbytes[]);
uint16_t ntohs(uint8_t netbytes[]);
float    ntohf(uint8_t netbytes[]);
#if (__SIZEOF_DOUBLE__ == 8)
double   ntohd(uint8_t netbytes[]);
#endif

#endif // ifndef netconv_h