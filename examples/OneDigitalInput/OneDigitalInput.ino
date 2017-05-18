/*
 * Author: Franco Grassano - YAELTEX
 * Date: 18/02/2016
 * ---
 * Description: Reads button with external PullUp resistor, and sends its state over serial or MIDI. 
 *              Choose which, by commenting one of the lines, where it is indicated.
 *              This example is for use with the Kilomux Shield.
 * 
 * Kilomux Library is available at https://github.com/Yaeltex/KiloMux-Shield/blob/master/Arduino%20Code/KiloMuxShield%20Library/KiloMuxShield.zip
 */
 
#include <Kilomux.h>              // Import class declaration
#include <KilomuxDefs.h>          // Import Kilomux defines
#include <MIDI.h>                 // Import Arduino MIDI Library headers
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Settings.h>

MIDI_CREATE_DEFAULT_INSTANCE() // Create a hardware instance of the library

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define SERIAL_COMMS          // Leave uncommented (and comment the line below) to send over serial
//#define MIDI_COMMS          // Leave uncommented (and comment the line above) to send midi messages
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MIDI_CHANNEL  1        // Button will send NOTE_ON when pressed (0V/logical 0), and NOTE_OFF when released (5V/logical 1), over channel 1s over MIDI port or USB
#define BUTTON_NOTE   24       // Note 24 -> Pitch C0

Kilomux KmShield;                 // Kilomux Shield  

unsigned int buttonInput = 1;     // Shield input where we connected a button or a sensor (1-16)
                                  // Remember to add a 10K pull-up resistor (from the button input to 5V)

bool buttonState = 1, lastState = 1;  // Variables to store digital values. Initialize off.

void setup() {
  KmShield.init();                                    // Initialize Kilomux shield hardware
  
  #if defined(SERIAL_COMMS)
  Serial.begin(115200);                               // Initialize serial
  #elif defined(MIDI_COMMS)
  MIDI.begin(MIDI_CHANNEL_OMNI); MIDI.turnThruOff();  // Initialize MIDI port. Turn MIDI-thru off, which is on by default
  #endif
}

void loop() {
  buttonState = KmShield.digitalReadKm(MUX_A, buttonInput);     // Read digital value from MUX_A and channel 'buttonInput' (1-16)   
  
  if(!buttonState && lastState){                                // If we read 0, and the last time we read 1, means button was just pressed
    #if defined(SERIAL_COMMS)
    Serial.println("Button state: ON");                             // Print value at the serial monitor        
    #elif defined(MIDI_COMMS)                                                                                   
    MIDI.sendNoteOn(BUTTON_NOTE, 127, MIDI_CHANNEL);                // Send midi message "note on"              
    #endif
  }
  else if(buttonState && !lastState){                           // If we read 1, and the last time we read 0, means button was just released
    #if defined(SERIAL_COMMS)
    Serial.println("Button state: OFF");                             // Print value at the serial monitor       
    #elif defined(MIDI_COMMS)                                                                                                                                                                                                   
    MIDI.sendNoteOff(BUTTON_NOTE, 0, MIDI_CHANNEL);                  // Send midi message "note off"            
    #endif
  }
  lastState = buttonState;                                        // Update last button state.
}
