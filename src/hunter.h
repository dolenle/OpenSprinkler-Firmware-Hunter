/////////////////////////////////////////////////////////////////////////////
// Hunter Irrigation Bus Class
//
// This work is released under the Creative Commons Zero (CC0) license.
// See http://creativecommons.org/publicdomain/zero/1.0/
// 
// Modified from Esquilo library for ESP8266/Arduino for bit-banging
/////////////////////////////////////////////////////////////////////////////

#ifndef HUNTER_H
#define HUNTER_H

#include <Arduino.h>

// library interface description
class HunterInterface
{
	// user-accessible "public" interface
	public:
	HunterInterface(uint8_t pin);
    static void start(uint8_t zone, uint8_t time);
    static void stopAll();
    static void program(uint8_t num);
	
	private:
	static void bitOne();
    static void bitZero();
    static void write(uint8_t * buffer, int len, bool extrabit);
    static void setBitfield(uint8_t * bits, uint8_t pos, uint8_t val, uint8_t len);
    static uint8_t hunter_pin;
};

#endif
