  /**
 * @file    netconv.cpp
 * @brief   Implementation file for the netconv library
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

/**
 * @mainpage Arduino Library for byte order conversions
 *
 * @section intro_sec Introduction
 * This library provides functions for converting number from native host
 * byte order representation to network byte order (i.e. big-endian byte
 * order) and back.
 * 
 * On POSIX systems, the functions are either defined in <arpa/inet.h> or
 * <netinet/in.h>. However, these header files are not available on Ardunino.
 * This library makes those functions (and more) available.
 * 
 * @note The implementation uses built-in functions of gcc/g++ and preprocessor
 * macros specific to gcc/g++. The library will most probably not compile with
 * any other compiler.
 * 
 * The following functions are provided for conversion from host byte order
 * to network byte order:
 * @li htonll() for long long unsigned integers (uint64_t)
 * @li htonl() for long unsigned integers (uint32_t)
 * @li htons() for short unsigned integers (uint16_t)
 * @li htonf() for IEEE 754 single precision floating point numbers (32 bit)
 * @li htond() for IEEE 754 double precision floating point numbers (64 bit)
 * 
 * The following functions are provided for conversion from network byte order
 * to host byte order:
 * @li ntohll() for long long unsigned integers (uint64_t)
 * @li ntohl() for long unsigned integers (uint32_t)
 * @li ntohs() for short unsigned integers (uint16_t)
 * @li ntohf() for IEEE 754 single precision floating point numbers (32 bit)
 * @li ntohd() for IEEE 754 double precision floating point numbers (64 bit)
 * 
 * @note The functions htond() and ntohd() are only available on platforms
 * which have a "true" IEEE 754 double precision floating point implementation 
 * with 64 bits. In particular, those functions are not available on most 
 * AVR Arduinos, where the @em double data type is only an alias for single
 * precision 32-bit @em float data type. 
 * 
 * @note htonf() and htond() take floating point numbers as arguments and return an
 * unsigned integer of the proper size. ntohf() and ntohd() take an unsingned integer
 * as parameter and return a floating point number of the proper precision. This is
 * modeled after the conversion functions introduced by Microsoft in Winsock2.h.
 * 
 * Each function is overloaded three times, i.e. there are three different
 * versions of each function.
 * 
 * @li Take the number to convert as a single parameter and return the converted number.
 * @li Extract the number to convert from an array of bytes (uint8_t data type) and return the converted number.
 * @li Take the number to convert as a single parameter and write the result of the conversion back to an array of bytes (uint8_t).
 * 
 * @warning The last variant is potentially dangerous, because there is no bounds checking for the target array.
 * 
 * 
 * @section author Author
 *
 * Andreas Grommek
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
 *
 * @version  1.0.0
 * @date     2021-05-27
 */

#include "netconv.h"


/* 
 * Note on Doxygen settings:
 * For htond() and ntohd(), which are only compiled on supported platforms,
 * we must add 
 * 
 *   PREDEFINED = __SIZEOF_DOUBLE__=8
 * 
 * to our Doxyfile. Otherwise not documentation will be generated for these
 * functions.
 */

/* Host to network conversions */

/**
 * @brief  Convert an unsigned integer (uint64_t) from host byte order
 *         to network byte order (big-endian). Do nothing if host byte
 *         order is already big-endian.
 * @param  hostlonglong
 *         Numerical value (uint64_t) in host byte order.
 * @return Numerical value (uint64_t) in network byte order.
 */
uint64_t htonll(uint64_t hostlonglong) {
    #if ( __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ )
        return __builtin_bswap64(hostlonglong);
    #elif ( __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ )
        return hostlonglong;
    #else
        #error "integer byte order could not be determined!" 
    #endif
}

/**
 * @brief  Convert an unsigned integer (uint32_t) from host byte order
 *         to network byte order (big-endian). Do nothing if host byte
 *         order is already big-endian.
 * @param  hostlong
 *         Numerical value (uint32_t) in host byte order.
 * @return Numerical value (uint32_t) in network byte order.
 */
uint32_t htonl(uint32_t hostlong) {
    #if ( __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ )
        return __builtin_bswap32(hostlong);
    #elif ( __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ )
        return hostlong;
    #else
        #error "integer byte order could not be determined!" 
    #endif
}

/**
 * @brief  Convert an unsigned integer (uint16_t) from host byte order
 *         to network byte order (big-endian). Do nothing if host byte
 *         order is already big-endian.
 * @param  hostshort
 *         Numerical value (uint16_t) in host byte order.
 * @return Numerical value (uint16_t) in network byte order.
 */
 uint16_t htons(uint16_t hostshort) {
    #if ( __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ )
        return __builtin_bswap16(hostshort);
    #elif ( __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ )
        return hostshort;
    #else
        #error "integer byte order could not be determined!" 
    #endif
}

/**
 * @brief  Convert an IEEE 754 single precision float (32 bit) from 
 *         host byte order to network (i.e. big-endian) byte order.
 *         Network byte order (aka interchange format) is defined as
 *         having the byte containing the sign bit at the lowest 
 *         memory address.
 *         Do nothing if host byte order is already big-endian.
 * @param  hostfloat
 *         Numerical value (IEEE 745 single precision 32 bit float) 
 *         in host byte order.
 * @return Numerical value as uint32_t, representing the bytes of the
 *         float in network byte order (i.e. byte with sign bit first).
 * @note   The return value as uint32_t is modeled after the non-standard
 *         implementation of htonf() by Microsoft.
 */
uint32_t htonf(float hostfloat) {
    static_assert(sizeof(uint32_t) == sizeof(float));
    uint32_t retval;
    memcpy(&retval, &hostfloat, sizeof(float));
    #if ( __FLOAT_WORD_ORDER__ == __ORDER_LITTLE_ENDIAN__ ) 
        return __builtin_bswap32(retval);
    #elif ( __FLOAT_WORD_ORDER__ == __ORDER_BIG_ENDIAN__ ) 
        return retval;
    #else
        #error "float word order could not be determined!" 
    #endif
}

/**
 * @brief  Convert an IEEE 754 double precision float (64 bit) from 
 *         host byte order to network (i.e. big-endian) byte order.
 *         Network byte order (aka interchange format) is defined as
 *         having the byte containing the sign bit at the lowest 
 *         memory address.
 *         Do nothing if host byte order is already big-endian.
 * @param  hostdouble
 *         Numerical value (IEEE 745 double precision 64 bit float) 
 *         in host byte order.
 * @return Numerical value as uint64_t, representing the bytes of the
 *         double in network byte order (i.e. byte with sign bit first).
 * @note   The return value as uint64_t is modeled after the non-standard
 *         implementation of htond() by Microsoft.
 * @note   This functions is only available on platforms supporting
 *         double precision floating point numbers, i.e. @b not on
 *         most AVR Arduinos.
 */    
#if (__SIZEOF_DOUBLE__ == 8)
uint64_t htond(double hostdouble) {
    static_assert(sizeof(uint64_t) == sizeof(double));
    uint64_t retval;
    memcpy(&retval, &hostdouble, sizeof(double));
    #if ( __FLOAT_WORD_ORDER__ == __ORDER_LITTLE_ENDIAN__ ) 
        return __builtin_bswap64(retval);
    #elif ( __FLOAT_WORD_ORDER__ == __ORDER_BIG_ENDIAN__ ) 
        return retval;
    #else
        #error "float word order could not be determined!" 
    #endif
}
#endif

/**
 * @brief   Convert an unsigned integer (uint64_t) from host byte order
 *          to network byte order (big-endian) and write the result
 *          into an array of uint8_t.
 * @param   hostlonglong
 *          Numerical value (uint64_t) in host byte order to convert.
 * @param   netbytes
 *          Pointer to an array of uint8_t. Eight bytes representing 
 *          hostlonglong in network byte order are written to this
 *          array.
 * @note    This function can be conveniently used to convert an 
 *          integer to network byte order and filling the result into
 *          a uint8_t array (e.g. a network send buffer) in a single
 *          step.
 * @warning There is no verification of buffer size. This function 
 *          puts @b eight bytes into the buffer. If the buffer is too
 *          small, a buffer overrun will occur.
 */
void htonll(uint64_t hostlonglong, uint8_t netbytes[]) {
    hostlonglong = htonll(hostlonglong);
    memcpy(netbytes, &hostlonglong, sizeof(uint64_t));
}

/**
 * @brief   Convert an unsigned integer (uint32_t) from host byte order
 *          to network byte order (big-endian) and write the result
 *          into an array of uint8_t.
 * @param   hostlong
 *          Numerical value (uint32_t) in host byte order to convert.
 * @param   netbytes
 *          Pointer to an array of uint8_t. Four bytes representing 
 *          hostlong in network byte order are written to this
 *          array.
 * @note    This function can be conveniently used to convert an 
 *          integer to network byte order and filling the result into
 *          a uint8_t array (e.g. a network send buffer) in a single
 *          step.
 * @warning There is no verification of buffer size. This function 
 *          puts @b four bytes into the buffer. If the buffer is too
 *          small, a buffer overrun will occur.
 */
 void htonl(uint32_t hostlong, uint8_t netbytes[]) {
    hostlong = htonl(hostlong);
    memcpy(netbytes, &hostlong, sizeof(uint32_t));
}

/**
 * @brief   Convert an unsigned integer (uint16_t) from host byte order
 *          to network byte order (big-endian) and write the result
 *          into an array of uint8_t.
 * @param   hostshort
 *          Numerical value (uint16_t) in host byte order to convert.
 * @param   netbytes
 *          Pointer to an array of uint8_t. Two bytes representing 
 *          hostshort in network byte order are written to this
 *          array.
 * @note    This function can be conveniently used to convert an 
 *          integer to network byte order and filling the result into
 *          a uint8_t array (e.g. a network send buffer) in a single
 *          step.
 * @warning There is no verification of buffer size. This function 
 *          puts @b two bytes into the buffer. If the buffer is too
 *          small, a buffer overrun will occur.
 */
void htons(uint16_t hostshort, uint8_t netbytes[]) {
    hostshort = htons(hostshort);
    memcpy(netbytes, &hostshort, sizeof(uint16_t));
}

/**
 * @brief   Convert an IEEE 754 single precision float (32 bit) from
 *          host byte order to network (big-endian, aka interchange 
 *          format) byte order and write the result into an array
 *          of uint8_t.
 * @param   hostfloat
 *          Numerical value to convert (float, 32 bit) in host byte order.
 * @param   netbytes
 *          Pointer to an array of uint8_t. Four bytes representing 
 *          hostfloat in network byte order are written to this
 *          array. The byte with the sign bit is written to the first
 *          byte of the buffer, i.e. at the lowest memory address.
 * @note    This function can be conveniently used to convert a floating
 *          point number to network byte order and filling the result into
 *          a uint8_t array (e.g. a network send buffer) in a single
 *          step.
 * @warning There is no verification of buffer size. This function 
 *          puts @b four bytes into the buffer. If the buffer is too
 *          small, a buffer overrun will occur.
 */
void htonf(float hostfloat, uint8_t netbytes[]) {
    static_assert(sizeof(uint32_t) == sizeof(float));
    uint32_t retval = htonf(hostfloat);
    memcpy(netbytes, &retval, sizeof(float));
}

#if (__SIZEOF_DOUBLE__ == 8)
/**
 * @brief   Convert an IEEE 754 double precision float (64 bit) from
 *          host byte order to network byte order (big-endian, aka 
 *          interchange format) and write the result into an array
 *          of uint8_t.
 * @param   hostdouble
 *          Numerical value to convert (double, 64 bit) in host byte order.
 * @param   netbytes
 *          Pointer to an array of uint8_t. Eight bytes representing 
 *          hostdouble in network byte order are written to this
 *          array. The byte with the sign bit is written to the first
 *          byte of the buffer, i.e. at the lowest memory address.
 * @note    This function can be conveniently used to convert a floating
 *          point number to network byte order and filling the result into
 *          a uint8_t array (e.g. a network send buffer) in a single
 *          step.
 * @warning There is no verification of buffer size. This function 
 *          puts @b eight bytes into the buffer. If the buffer is too
 *          small, a buffer overrun will occur.
 * @note    This functions is only available on platforms supporting
 *          double precision floating point numbers, i.e. @b not on
 *          most AVR Arduinos.
 */
void htond(double hostdouble, uint8_t netbytes[]) {
    static_assert(sizeof(uint64_t) == sizeof(double));
    uint64_t retval = htond(hostdouble);
    memcpy(netbytes, &retval, sizeof(double));
}
#endif

/**
 * @brief   Convert an integer (uint64_t) in host byte order stored
 *          in an array of uint8_t and return it as a single uint64_t
 *          in network (big-endian) byte order.
 * @param   hostbytes
 *          Pointer to an array of uint8_t, containing the number to
 *          convert in host byte order. The least significant byte is
 *          stored at the lowest memory address of the buffer. A total
 *          of eight bytes are read from the array.
 * @return  Number (uint64_t) in network byte order, to be sent over
 *          a network.
 */
 uint64_t htonll(uint8_t hostbytes[]) {
    uint64_t retval;
    memcpy(&retval, hostbytes, sizeof(uint64_t));
    return htonll(retval);
}

/**
 * @brief   Convert an integer (uint32_t) in host byte order stored
 *          in an array of uint8_t and return it as a single uint32_t
 *          in network (big-endian) byte order.
 * @param   hostbytes
 *          Pointer to an array of uint8_t, containing the number to
 *          convert in host byte order. The least significant byte is
 *          stored at the lowest memory address of the buffer. A total
 *          of four bytes are read from the array.
 * @return  Number (uint32_t) in network byte order, to be sent over
 *          a network.
 */
uint32_t htonl(uint8_t hostbytes[]) {
    uint32_t retval;
    memcpy(&retval, hostbytes, sizeof(uint32_t));
    return htonl(retval);
}

/**
 * @brief   Convert an integer (uint16_t) in host byte order stored
 *          in an array of uint8_t and return it as a single uint32_t
 *          in network (big-endian) byte order.
 * @param   hostbytes
 *          Pointer to an array of uint8_t, containing the number to
 *          convert in host byte order. The least significant byte is
 *          stored at the lowest memory address of the buffer. A total
 *          of two bytes are read from the array.
 * @return  Number (uint16_t) in network byte order, to be sent over
 *          a network.
 */
uint16_t htons(uint8_t hostbytes[]) {
    uint16_t retval;
    memcpy(&retval, hostbytes, sizeof(uint16_t));
    return htons(retval);
}

/**
 * @brief   Convert an IEEE 754 single precision float (32 bit) in 
 *          host byte order stored in an array of uint8_t and return
 *          it as a single uint32_t in network (i.e. big endian, aka 
 *          interchange) byte order.
 * @param   hostbytes
 *          Pointer to an array of uint8_t, containing the number to
 *          convert to network (i.e. big endian) byte order. A total
 *          of four bytes is read from the array.
 * @return  Number (uint32_t representing the bytes of the float) in
 *          network byte order, to be sent over a network.
 * @note    The return value is a uint32_t, not a float. The function
 *          is modeled after the non-standard implementation of 
 *          htonf() by Microsoft.     
 */
uint32_t htonf(uint8_t hostbytes[]) {
    float f;
    memcpy(&f, hostbytes, sizeof(float));
    return htonf(f); // return an uint32_t
}

#if (__SIZEOF_DOUBLE__ == 8)
/**
 * @brief   Convert an IEEE 754 double precision float (64 bit) in 
 *          host byte order stored in an array of uint8_t and return
 *          it as a single uint64_t in network (i.e. big endian, aka 
 *          interchange) byte order.
 * @param   hostbytes
 *          Pointer to an array of uint8_t, containing the number to
 *          convert to network (i.e. big endian) byte order. A total
 *          of eight bytes is read from the array.
 * @return  Number (uint64_t representing the bytes of the double) in
 *          network byte order, to be sent over a network.
 * @note    The return value is a uint64_t, not a double. The function
 *          is modeled after the non-standard implementation of 
 *          htond() by Microsoft.
 */
uint64_t htond(uint8_t hostbytes[]) {
    double d;
    memcpy(&d, hostbytes, sizeof(double));
    return htond(d); // return an uint64_t
}
#endif

/*
 * Network to host conversions. 
 */

/**
 * @brief  Convert an unsigned integer (uint64_t) from network byte
 *         order (big-endian) to host byte order. Do nothing if host
 *         byte order is already big-endian.
 * @param  netlonglong
 *         Numerical value (uint64_t) in network byte order.
 * @return Numerical value (uint64_t) in host byte order.
 */
 uint64_t ntohll(uint64_t netlonglong) {
    return htonll(netlonglong);
}

/**
 * @brief  Convert an unsigned integer (uint32_t) from network byte
 *         order (big-endian) to host byte order. Do nothing if host
 *         byte order is already big-endian.
 * @param  netlong
 *         Numerical value (uint32_t) in network byte order.
 * @return Numerical value (uint32_t) in host byte order.
 */
uint32_t ntohl(uint32_t netlong) {
    return htonl(netlong);
}

/**
 * @brief  Convert an unsigned integer (uint16_t) from network byte
 *         order (big-endian) to host byte order. Do nothing if host
 *         byte order is already big-endian.
 * @param  netshort
 *         Numerical value (uint16_t) in network byte order.
 * @return Numerical value (uint16_t) in host byte order.
 */

uint16_t ntohs(uint16_t netshort) {
    return htons(netshort);
}

/**
 * @brief  Convert an IEEE 754 single precision float (32 bit) from 
 *         network (i.e. big-endian) byte order to host byte order.
 *         Network byte order (aka interchange format) is defined as
 *         having the byte containing the sign bit at the lowest 
 *         memory address.
 *         Do nothing if host byte order is already big-endian.
 * @param  netfloat
 *         Numerical value in network byte order, stored in a uint32_t.
 * @return Single precision floating point number in host byte order.
 * @note   The parameter as uint32_t is modeled after the non-standard
 *         implementation of ntohf() by Microsoft.
 */
float ntohf(uint32_t netfloat) {
    static_assert(sizeof(uint32_t) == sizeof(float));
    // check for endiannes of float because
    // float might have another endianness than integers
    #if ( __FLOAT_WORD_ORDER__ == __ORDER_LITTLE_ENDIAN__ ) 
        netfloat = ntohl(netfloat);
    #elif ( __FLOAT_WORD_ORDER__ == __ORDER_BIG_ENDIAN__ ) 
        // do nothing, word/byte order is already ok
    #else
        #error "float word order could not be determined!" 
    #endif
    float retval;
    memcpy(&retval, &netfloat, sizeof(float));
    return retval;
}

#if (__SIZEOF_DOUBLE__ == 8)
/**
 * @brief  Convert an IEEE 754 double precision float (64 bit) from 
 *         network (i.e. big-endian) byte order to host byte order.
 *         Network byte order (aka interchange format) is defined as
 *         having the byte containing the sign bit at the lowest 
 *         memory address.
 *         Do nothing if host byte order is already big-endian.
 * @param  netdouble
 *         Numerical value in network byte order, stored in a uint64_t.
 * @return Double precision floating point number in host byte order.
 * @note   The parameter as uint64_t is modeled after the non-standard
 *         implementation of ntohd() by Microsoft.
 * @note   This functions is only available on platforms supporting
 *         double precision floating point numbers, i.e. @b not on
 *         most AVR Arduinos
 */
double ntohd(uint64_t netdouble) {
    static_assert(sizeof(uint64_t) == sizeof(double));
    #if ( __FLOAT_WORD_ORDER__ == __ORDER_LITTLE_ENDIAN__ ) 
        netdouble = ntohll(netdouble);
    #elif ( __FLOAT_WORD_ORDER__ == __ORDER_BIG_ENDIAN__ ) 
        // do nothing, word/byte order is already ok
    #else
        #error "float word order could not be determined!" 
    #endif
    double retval;
    memcpy(&retval, &netdouble, sizeof(double));
    return retval;
}
#endif

/**
 * @brief   Convert an unsigned integer (uint64_t) from network (i.e.
 *          big-endian byte order to host byte order and write the result
 *          into an array of uint8_t.
 * @param   netlonglong
 *          Numerical value (uint64_t) in network byte order to convert.
 * @param   hostbytes
 *          Pointer to an array of uint8_t. Eight bytes representing 
 *          netlonglong in host byte order are written to this
 *          array.
 * @warning There is no verification of buffer size. This function 
 *          puts @b eight bytes into the buffer. If the buffer is too
 *          small, a buffer overrun will occur.
 */
void ntohll(uint64_t netlonglong, uint8_t hostbytes[]) {
    netlonglong = ntohll(netlonglong);
    memcpy(hostbytes, &netlonglong, sizeof(uint64_t));
}

/**
 * @brief   Convert an unsigned integer (uint32_t) from network (i.e.
 *          big-endian byte order to host byte order and write the result
 *          into an array of uint8_t.
 * @param   netlong
 *          Numerical value (uint32_t) in network byte order to convert.
 * @param   hostbytes
 *          Pointer to an array of uint8_t. Four bytes representing 
 *          netlong in host byte order are written to this
 *          array.
 * @warning There is no verification of buffer size. This function 
 *          puts @b four bytes into the buffer. If the buffer is too
 *          small, a buffer overrun will occur.
 */
 void ntohl(uint32_t netlong, uint8_t hostbytes[]) {
    netlong = ntohl(netlong);
    memcpy(hostbytes, &netlong, sizeof(uint32_t));
}

/**
 * @brief   Convert an unsigned integer (uint16_t) from network (i.e.
 *          big-endian byte order to host byte order and write the result
 *          into an array of uint8_t.
 * @param   netshort
 *          Numerical value (uint16_t) in network byte order to convert.
 * @param   hostbytes
 *          Pointer to an array of uint8_t. Two bytes representing 
 *          netshort in host byte order are written to this
 *          array.
 * @warning There is no verification of buffer size. This function 
 *          puts @b two bytes into the buffer. If the buffer is too
 *          small, a buffer overrun will occur.
 */
 void ntohs(uint16_t netshort, uint8_t hostbytes[]) {
    netshort = ntohs(netshort);
    memcpy(hostbytes, &netshort, sizeof(uint16_t));
}

/**
 * @brief   Convert an unsigned integer (uint32_t), which represents 
 *          an IEEE 754 single precision floating point number, from 
 *          network (i.e. big-endian byte order to host byte order 
 *          and write the result into an array of uint8_t.
 * @param   netfloat
 *          Single precision float, represented as four bytes, packed 
 *          in a uint32_t in big-endian byte order. The byte containing
 *          the sign bit is stored at the lowest memory address of this 
 *          uint32_t.
 * @param   hostbytes
 *          Pointer to an array of uint8_t. Four bytes representing 
 *          netfloat in host byte order are written to this array.
 * @warning There is no verification of buffer size. This function 
 *          puts @b four bytes into the buffer. If the buffer is too
 *          small, a buffer overrun will occur.
 * @note   The parameter as uint32_t is modeled after the non-standard
 *         implementation of ntohf() by Microsoft.
 */
 void ntohf(uint32_t netfloat, uint8_t hostbytes[]) {
    static_assert(sizeof(uint32_t) == sizeof(float));
    float f = ntohf(netfloat);
    memcpy(hostbytes, &f, sizeof(float));        
}

#if (__SIZEOF_DOUBLE__ == 8)
/**
 * @brief   Convert an unsigned integer (uint64_t), which represents 
 *          an IEEE 754 double precision floating point number, from 
 *          network (i.e. big-endian byte order to host byte order 
 *          and write the result into an array of uint8_t.
 * @param   netdouble
 *          Double precision float, represented as eight bytes, packed 
 *          in a uint64_t in big-endian byte order. The byte containing
 *          the sign bit is stored at the lowest memory address of this 
 *          uint64_t.
 * @param   hostbytes
 *          Pointer to an array of uint8_t. Eight bytes representing 
 *          netdouble in host byte order are written to this array.
 * @warning There is no verification of buffer size. This function 
 *          puts @b eight bytes into the buffer. If the buffer is too
 *          small, a buffer overrun will occur.
 * @note   The parameter as uint64_t is modeled after the non-standard
 *         implementation of ntohd() by Microsoft.
 */
void ntohd(uint64_t netdouble, uint8_t hostbytes[]) {
    static_assert(sizeof(uint64_t) == sizeof(double));
    double d = ntohd(netdouble);
    memcpy(hostbytes, &d, sizeof(double));        
}
#endif

/**
 * @brief   Convert an integer (uint64_t) in network (i.e. big-endian)
 *          byte order stored in an array of uint8_t and return it as 
 *          a single uint64_t in host byte order.
 * @param   netbytes
 *          Pointer to an array of uint8_t, containing the number to
 *          convert to network byte order. The most significant byte is
 *          stored at the lowest memory address of the buffer. A total
 *          of eight bytes are read from the array.
 * @return  Number (uint64_t) in host byte order
 */
 uint64_t ntohll(uint8_t netbytes[]) {
    uint64_t retval;
    memcpy(&retval, netbytes, sizeof(uint64_t));
    return ntohll(retval);
}

/**
 * @brief   Convert an integer (uint32_t) in network (i.e. big-endian)
 *          byte order stored in an array of uint8_t and return it as 
 *          a single uint32_t in host byte order.
 * @param   netbytes
 *          Pointer to an array of uint8_t, containing the number to
 *          convert to network byte order. The most significant byte is
 *          stored at the lowest memory address of the buffer. A total
 *          of four bytes are read from the array.
 * @return  Number (uint32_t) in host byte order
 */
uint32_t ntohl(uint8_t netbytes[]) {
    uint32_t retval;
    memcpy(&retval, netbytes, sizeof(uint32_t));
    return ntohl(retval);
}

/**
 * @brief   Convert an integer (uint16_t) in network (i.e. big-endian)
 *          byte order stored in an array of uint8_t and return it as 
 *          a single uint16_t in host byte order.
 * @param   netbytes
 *          Pointer to an array of uint8_t, containing the number to
 *          convert to network byte order. The most significant byte is
 *          stored at the lowest memory address of the buffer. A total
 *          of two bytes are read from the array.
 * @return  Number (uint16_t) in host byte order
 */
uint16_t ntohs(uint8_t netbytes[]) {
    uint16_t retval;
    memcpy(&retval, netbytes, sizeof(uint16_t));
    return ntohs(retval);
}

/**
 * @brief   Convert an IEEE 754 single precision float (32 bit) in 
 *          network (big-endian, network interchange) byte order 
 *          stored in an array of uint8_t and return it as a single 
 *          float in host byte order.
 * @param   netbytes
 *          Pointer to an array of uint8_t, containing the number to
 *          convert to host byte order. A total of four bytes is read
 *          from the array. The byte containing the sign bit must be
 *          stored at the lowest memory address.
 * @return  Number (single precision float, 32 bit) in host byte 
 *          order.
 * @note    The function is modeled after the non-standard 
 *          implementation of ntohf() by Microsoft.     
 */
 float ntohf(uint8_t netbytes[]) {
    static_assert(sizeof(uint32_t) == sizeof(float));
    uint32_t u;
    memcpy(&u, netbytes, sizeof(float));
    return ntohf(u);
}

#if (__SIZEOF_DOUBLE__ == 8)
/**
 * @brief   Convert an IEEE 754 double precision float (64 bit) in 
 *          network (big-endian, network interchange) byte order 
 *          stored in an array of uint8_t and return it as a single 
 *          double in host byte order.
 * @param   netbytes
 *          Pointer to an array of uint8_t, containing the number to
 *          convert to host byte order. A total of eight bytes is read
 *          from the array. The byte containing the sign bit must be
 *          stored at the lowest memory address.
 * @return  Number (double precision float, 64 bit) in host byte 
 *          order.
 * @note    The function is modeled after the non-standard 
 *          implementation of ntohd() by Microsoft.     
 * @note   This functions is only available on platforms supporting
 *         double precision floating point numbers, i.e. @b not on
 *         most AVR Arduinos.
 */
double ntohd(uint8_t netbytes[]) {
    static_assert(sizeof(uint64_t) == sizeof(double));
    uint64_t u;
    memcpy(&u, netbytes, sizeof(double));
    return ntohd(u);
}
#endif