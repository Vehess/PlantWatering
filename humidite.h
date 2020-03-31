/*
 * hulidite.h
 *
 *  Created on: 19 mars 2020
 *      Author: Vehess
 */

#ifndef HUMIDITE_H_
#define HUMIDITE_H_

DHT_t readDHT11(byte pin, float* temperature, float* humidity);
DHT_t readDHT22(byte pin, float* temperature, float* humidity);
DHT_t readDHTxx(byte pin, byte* data, unsigned long start_time, unsigned long timeout);


#endif /* HUMIDITE_H_ */
