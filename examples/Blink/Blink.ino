/*
 * Author: Franco Grassano - YAELTEX
 * Date: 18/02/2016
 * ---
 * LICENSE INFO
 * This file is part of Kilomux Arduino Library.
 *
 * Kilomux Arduino Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Kilomux Arduino Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Kilomux Arduino Library.  If not, see <http://www.gnu.org/licenses/>.
 *
 * ---
 * Description: Blink an LED.
 *              This example is for use with the Kilomux Shield.
 * 
 * Kilomux Library is available at https://github.com/Yaeltex/Kilomux-Shield/blob/master/Arduino%20Code/KilomuxShield%20Library/KilomuxShield.zip
 */
 
#include <Kilomux.h>          // Import class declaration
#include <KilomuxDefs.h>      // Import Kilomux defines

Kilomux KmShield;             // KiloMux Shield   

#define MS_ON  1000             // Number of milliseconds the LED will be on 
#define MS_OFF 1000             // Number of milliseconds the LED will be off

unsigned int ledOutput = 0;   // KiloMux output where we already connected an LED (0-15)

void setup() {
  KmShield.init();
}

void loop() {
  KmShield.digitalWriteKm(ledOutput, HIGH);     // Turn led on
  delay(MS_ON);                                 // Wait. LED is on.
  KmShield.digitalWriteKm(ledOutput, LOW);      // Now turn it off. 
  delay(MS_OFF);                                // Wait. It's off.
}

