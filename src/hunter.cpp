/////////////////////////////////////////////////////////////////////////////
// Hunter Irrigation Bus Class
//
// This work is released under the Creative Commons Zero (CC0) license.
// See http://creativecommons.org/publicdomain/zero/1.0/
// 
// Modified from Esquilo library for ESP8266/Arduino for bit-banging
/////////////////////////////////////////////////////////////////////////////

#include "hunter.h"

#define RESET_HIGH_MS       325
#define RESET_LOW_MS        65
#define START_HIGH_US       900
#define START_LOW_US        190 // Adjusted slightly to compensate for CPU time

// Bit-bang intervals (us)
#define SHORT_INTERVAL      208
#define LONG_INTERVAL       1875

// UART inverted logic
#define HUNTER_ONE          0x00
#define HUNTER_ZERO         0xFF

// Initialize bitbang interface
// Can be used on any pin, but causes long blocking delays
HunterInterfaceBitbang::HunterInterfaceBitbang(uint8_t pin)
{
	hunter_pin = pin;
    pinMode(hunter_pin, OUTPUT);
}

// Description: Set a value with an arbitrary bit width to a bit position
// within a blob
// Arguments: bits - blob to write the value to
// pos - bit position within the blob
// val - value to write
// len - len in bits of the value
void HunterInterfaceBitbang::setBitfield(uint8_t * bits, uint8_t pos, uint8_t val, uint8_t len)
{
    while (len > 0) {
        if (val & 0x1) {
            bits[pos / 8] = bits[pos / 8] | 0x80 >> (pos % 8);
        } else {
            bits[pos / 8] = bits[pos / 8] & ~(0x80 >> (pos % 8));
        }
        len--;
        val = val >> 1;
        pos++;
    }
}

// Description: Write the bit sequence out of the bus
// Arguments: buffer - blob containing the bits to transmit
// extrabit - if true, then write an extra 1 bit
void HunterInterfaceBitbang::write(uint8_t * buffer, int len, bool extrabit)
{
    // Start Frame
    digitalWrite(hunter_pin, HIGH);
    delay(RESET_HIGH_MS); //milliseconds
    digitalWrite(hunter_pin, LOW);
    delay(RESET_LOW_MS); //milliseconds
    digitalWrite(hunter_pin, HIGH);
    delayMicroseconds(START_HIGH_US); //microseconds
    digitalWrite(hunter_pin, LOW);
    delayMicroseconds(START_LOW_US); //microseconds

    // Write the bits out
    for (uint8_t i = 0; i<len; i++) {
        uint8_t senduint8_t = buffer[i];
        for (uint8_t inner = 0; inner < 8; inner++) {
            // Send high order bits first
            if (senduint8_t & 0x80) {
                bitOne();
            } else {
                bitZero();
            }
            senduint8_t <<= 1;
        }
    }

    // Include an extra 1 bit
    if (extrabit) {
        bitOne();
    }

    // Write the stop pulse
    bitZero();
}

// Description: Start a zone
// Arguments: zone - zone number (1-48)
// time - time in minutes (0-240)
void HunterInterfaceBitbang::start(uint8_t zone, uint8_t time)
{
    // Start out with a base frame 
    uint8_t buffer[] = {0xff,0x00,0x00,0x00,0x10,0x00,0x00,0x04,0x00,0x00,0x01,0x00,0x01,0xb8,0x3f};

    if (zone < 1 || zone > 48) {
        return;
    }

    if (time > 240) {
        return;
    }

    // The bus protocol is a little bizzare, not sure why

    // Bits 9:10 are 0x1 for zones > 12 and 0x2 otherwise
    if (zone > 12) {
        setBitfield(buffer, 9, 0x1, 2);
    } else {
        setBitfield(buffer, 9, 0x2, 2);
    }

    // Zone + 0x17 is at bits 23:29 and 36:42
    setBitfield(buffer, 23, zone + 0x17, 7);
    setBitfield(buffer, 36, zone + 0x17, 7);

    // Zone + 0x23 is at bits 49:55 and 62:68
    setBitfield(buffer, 49, zone + 0x23, 7);
    setBitfield(buffer, 62, zone + 0x23, 7);

    // Zone + 0x2f is at bits 75:81 and 88:94
    setBitfield(buffer, 75, zone + 0x2f, 7);
    setBitfield(buffer, 88, zone + 0x2f, 7);

    // Time is encoded in three places and broken up by nibble
    // Low nibble:  bits 31:34, 57:60, and 83:86
    // High nibble: bits 44:47, 70:73, and 96:99
    setBitfield(buffer, 31, time, 4);
    setBitfield(buffer, 44, time >> 4, 4);
    setBitfield(buffer, 57, time, 4);
    setBitfield(buffer, 70, time >> 4, 4);
    setBitfield(buffer, 83, time, 4);
    setBitfield(buffer, 96, time >> 4, 4);

    // Bottom nibble of zone - 1 is at bits 109:112
    setBitfield(buffer, 109, zone - 1, 4);

    // Write the bits out of the bus
    write(buffer, sizeof(buffer), true);
}

// Description: Run a program
// Arguments: num - program number (1-4)
void HunterInterfaceBitbang::program(uint8_t num)
{
    // Start with a basic program frame
    uint8_t buffer[] = {0xff, 0x40, 0x03, 0x96, 0x09, 0xbd, 0x7f};

    if (num < 1 || num > 4) {
        return;
    }

    // Program number - 1 is at bits 31:32
    setBitfield(buffer, 31, num - 1, 2);

    write(buffer, sizeof(buffer), false);
}

// Description: writes a low bit on the bus
// Arguments: none
void HunterInterfaceBitbang::bitZero(void)
{
    digitalWrite(hunter_pin, HIGH);
    delayMicroseconds(SHORT_INTERVAL); //microseconds
    digitalWrite(hunter_pin, LOW);
    delayMicroseconds(LONG_INTERVAL); //microseconds
}

// Description: writes a high bit on the bus
// Arguments: none
void HunterInterfaceBitbang::bitOne(void)
{
    digitalWrite(hunter_pin, HIGH);
    delayMicroseconds(LONG_INTERVAL); //microseconds
    digitalWrite(hunter_pin, LOW);
    delayMicroseconds(SHORT_INTERVAL); //microseconds
}

/*
* HunterInterfaceUART using hardware UART implementation to reduce CPU usage
*/

// Initialize
HunterInterfaceUART::HunterInterfaceUART(uint8_t pin)
{
    if(pin == 2) {
        hunter_uart = &Serial1;
        hunter_uart_cfg = &U1C0;
    } else if(pin == 1) {
        hunter_uart = &Serial;
        hunter_uart_cfg = &U0C0;
    }

    hunter_uart->begin(4807, SERIAL_8N1);
    *hunter_uart_cfg |= BIT(UCTXI); // Invert UART
}

// Description: Set a value with an arbitrary bit width to a bit position
// within a blob
// Arguments: bits - blob to write the value to
// pos - bit position within the blob
// val - value to write
// len - len in bits of the value
void HunterInterfaceUART::setBitfield(uint8_t * bits, uint8_t pos, uint8_t val, uint8_t len)
{
    while (len > 0) {
        if (val & 0x01) {
            bits[pos] = HUNTER_ONE;
        } else {
            bits[pos] = HUNTER_ZERO;
        }
        len--;
        val >>= 1;
        pos++;
    }
}

// Description: Write the bit sequence out of the bus
// Arguments: buffer - blob containing the bits to transmit
void HunterInterfaceUART::write(uint8_t * buffer, int len)
{
    // Start Frame
    *hunter_uart_cfg &= ~BIT(UCTXI);
    delay(RESET_HIGH_MS); //milliseconds
    *hunter_uart_cfg ^= BIT(UCTXI);
    delay(RESET_LOW_MS); //milliseconds
    *hunter_uart_cfg ^= BIT(UCTXI);
    delayMicroseconds(START_HIGH_US); //microseconds
    *hunter_uart_cfg ^= BIT(UCTXI);
    delayMicroseconds(START_LOW_US); //microseconds

    // Write the bits out
    hunter_uart->write(buffer, len);

    // Write the stop pulse
    hunter_uart->write(HUNTER_ZERO);
}

// Description: Start a zone
// Arguments: zone - zone number (1-48)
// time - time in minutes (0-240)
void HunterInterfaceUART::start(uint8_t zone, uint8_t time)
{
    // Start out with a base frame 
    uint8_t buffer[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xFF
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x00
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x00
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x00 
        0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, // 0x10 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x00 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x00 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, // 0x04 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x00 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x00 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, // 0x01 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x00 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, // 0x01 
        0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, // 0xB8 
        0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x3F
        0x00 // Extra bit
    };

    if (zone < 1 || zone > 48 || time > 240) {
        return;
    }

    // The bus protocol is a little bizzare, not sure why

    // Bits 9:10 are 0x1 for zones > 12 and 0x2 otherwise
    if (zone > 12) {
        setBitfield(buffer, 9, 0x1, 2);
    } else {
        setBitfield(buffer, 9, 0x2, 2);
    }

    // Zone + 0x17 is at bits 23:29 and 36:42
    setBitfield(buffer, 23, zone + 0x17, 7);
    setBitfield(buffer, 36, zone + 0x17, 7);

    // Zone + 0x23 is at bits 49:55 and 62:68
    setBitfield(buffer, 49, zone + 0x23, 7);
    setBitfield(buffer, 62, zone + 0x23, 7);

    // Zone + 0x2f is at bits 75:81 and 88:94
    setBitfield(buffer, 75, zone + 0x2f, 7);
    setBitfield(buffer, 88, zone + 0x2f, 7);

    // Time is encoded in three places and broken up by nibble
    // Low nibble:  bits 31:34, 57:60, and 83:86
    // High nibble: bits 44:47, 70:73, and 96:99
    setBitfield(buffer, 31, time, 4);
    setBitfield(buffer, 44, time >> 4, 4);
    setBitfield(buffer, 57, time, 4);
    setBitfield(buffer, 70, time >> 4, 4);
    setBitfield(buffer, 83, time, 4);
    setBitfield(buffer, 96, time >> 4, 4);

    // Bottom nibble of zone - 1 is at bits 109:112
    setBitfield(buffer, 109, zone - 1, 4);

    // Write the bits out of the bus
    write(buffer, sizeof(buffer));
}

// Description: Run a program
// Arguments: num - program number (1-4)
void HunterInterfaceUART::program(uint8_t num)
{
    // Start with a basic program frame
    uint8_t buffer[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xFF 
        0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x40 
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, // 0x03 
        0x00, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0xFF, // 0x96 
        0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x00, // 0x09 
        0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, // 0xBD 
        0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x7F
    };

    if (num < 1 || num > 4) {
        return;
    }

    // Program number - 1 is at bits 31:32
    setBitfield(buffer, 31, num - 1, 2);

    write(buffer, sizeof(buffer));
}
