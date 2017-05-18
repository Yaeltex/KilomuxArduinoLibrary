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
 * Description: Performs Knight Rider over shield's 16 outputs.
 *              This example is for use with the Kilomux Shield.
 * 
 * Kilomux Library is available at https://github.com/Yaeltex/Kilomux-Shield/blob/master/Arduino%20Code/KilomuxShield%20Library/KilomuxShield.zip
 */
 
#include <Kilomux.h>          // Import class declaration
#include <KilomuxDefs.h>      // Import KiloMux defines

Kilomux KmShield;             // Kilomux Shield   

#define MS_ON  40             // Number of milliseconds the LED will be on 
#define MS_OFF 4              // Number of milliseconds the LED will be off

void setup() {
  KmShield.init();
}

void loop() {
  for (int led = 0; led < NUM_OUTPUTS; led++){        // Sweeps all 16 outputs
    KmShield.digitalWriteKm(led, HIGH);               // Turn led on
    delay(MS_ON);                                     // Wait. LED is on.
    KmShield.digitalWriteKm(led, LOW);                // Now turn it off. 
    delay(MS_OFF);                                    // Wait. It's off.
  } 
}

