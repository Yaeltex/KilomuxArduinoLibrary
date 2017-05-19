/*
   Autor: Franco Grassano - YAELTEX
   ---
   INFORMACIÓN DE LICENCIA
   Kilo Mux Shield por Yaeltex se distribuye bajo una licencia
   Creative Commons Atribución-CompartirIgual 4.0 Internacional - http://creativecommons.org/licenses/by-sa/4.0/
   ----
   Código para el manejo de los integrados 74HC595 tomado de http://bildr.org/2011/02/74HC595/
   Librería de multiplexado (modificada) tomada de http://mayhewlabs.com/products/mux-shield-2
   Librería para el manejo del sensor de ultrasonido tomada de http://playground.arduino.cc/Code/NewPing

   Este código fue desarrollado para el KILO MUX SHIELD desarrolado en conjunto por Yaeltex y el Laboratorio del Juguete, en Buenos Aires, Argentina,
   apuntando al desarrollo de controladores MIDI con arduino.
   Está preparado para manejar 2 (expandible) registros de desplazamiento 74HC595 conectados en cadena (16 salidas digitales en total),
   y 2 multiplexores CD4067 de 16 canales cada uno (16 entradas analógicas, y 16 entradas digitales), pero es expandible en el
   caso de utilizar hardware diferente. Para ello se modifican los "define" NUM_MUX, NUM_CANALES_MUX y NUM_595s.
   NOTA: Se modificó la librería MuxShield, para trabajar sólo con 1 o 2 multiplexores. Si se necesita usar más multiplexores, descargar la librería original.

   Para las entradas analógicas, por cuestiones de ruido se recomienda usar potenciómetros o sensores con buena estabilidad, y con preferencia con valores
   cercanos o menores a 10 Kohm.
   Agradecimientos:
   - Jorge Crowe
   - Lucas Leal
   - Dimitri Diakopoulos
*/

/*
   Inclusión de librerías.
*/
#include <KM_Data.h>
#include <Kilomux.h>
#include <NewPing.h>
#include <MIDI.h>

void setup(); // Esto es para solucionar el bug que tiene Arduino al usar los #ifdef del preprocesador

//#define DEBUG
//#define MIDI_COMMS

#if defined(DEBUG)
#include <EEPROMex.h>
#endif

#if defined(MIDI_COMMS)
struct MySettings : public midi::DefaultSettings
{
  static const unsigned SysExMaxSize = 64; // Accept SysEx messages up to 1024 bytes long.
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Create a 'MIDI' object using MySettings bound to Serial.
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial, MIDI, MySettings);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif

#define STATUS_BLINK_INTERVAL 100
#define OUTPUT_BLINK_INTERVAL 300

#define MAX_BANKS        4
#define NUM_LEDS         4
#define NUM_BUTTONS      4

static const char * const modeLabels[] = {
  "Off"
  , "Note"
  , "CC"
  , "NRPN"
  , "Program Change"
  , "Shifter"
};
#define MODE_LABEL(m)   ((m) <= KMS::M_SHIFTER ? modeLabels[m] : "???")

// SysEx commands
#define CONFIG_MODE       1    // PC->hw : Activate monitor mode
#define CONFIG_ACK        2    // HW->pc : Acknowledge the config mode
#define DUMP_TO_HW        3    // PC->hw : Partial EEPROM dump from PC
#define DUMP_OK           4    // HW->pc : Ack from dump properly saved
#define EXIT_CONFIG       5    // HW->pc : Deactivate monitor mode
#define EXIT_CONFIG_ACK   6    // HW->pc : Ack from exit config mode

#define CONFIG_OFF 0
#define CONFIG_ON 1

#define LED_PORT      2

#define SHIFTER_B0    7
#define SHIFTER_B1    6
#define SHIFTER_B2    5
#define SHIFTER_B3    4

#define INTERVALO_CHECK_SHIFT_INPUTS_MS  50

// ANALOG FILTER ADJUSTMENTS
// AJUSTABLE - Si hay ruido que varía entre valores (+- 1, +- 2, +- 3...) colocar el umbral en (1, 2, 3...)
#define NOISE_THRESHOLD             1                      // Ventana de ruido para las entradas analógicas. Si entrada < entrada+umbral o 
#define NOISE_THRESHOLD_NRPN        4                      // Ventana de ruido para las entradas analógicas. Si entrada < entrada+umbral o 

#define ANALOG_UP     1
#define ANALOG_DOWN   0

// Ultrasonic sensor defines
// #define US_SENSOR_FILTER_SIZE  3         // Cantidad de valores almacenados para el filtro. Cuanto más grande, mejor el suavizado y  más lenta la lectura.

// Global data in EEPROM containing general config of inputs, outputs, US sensor, LED mode
KMS::GlobalData gD = KMS::globalData();

// Ultrasonic sensor variables
KMS::InputUS ultrasonicSensorData = KMS::ultrasound();
unsigned long minMicroSecSensor = 0;
unsigned long maxMicroSecSensor = 0;
byte pingSensorInterval = 25; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingSensorTimer;     // Holds the next ping time.
bool sensorActive = 0;                  // Inicializo inactivo (variable interna)
uint16_t usSensorPrevValue;                       // Array para el filtrado de la lectura del sensor
uint16_t uSeg = 0;                                // Contador de microsegundos para el ping del sensor.

// Running average filter variables
//uint8_t filterIndex = 0;            // Indice que recorre los valores del filtro de suavizado
//uint8_t filterCount = 0;
//uint16_t filterSum = 0;
//uint16_t filterSamples[US_SENSOR_FILTER_SIZE];

Kilomux KMShield;                                       // Objeto de la clase Kilomux
//NewPing usSensor(SensorTriggerPin, SensorEchoPin, MAX_SENSOR_DISTANCE); // Instancia de NewPing para el sensor US.

// Digital input and output states
bool digitalInputState[MAX_BANKS][NUM_MUX * NUM_MUX_CHANNELS];  // Estado de las entradas marcadas como digitales
bool buttonStateBanks[MAX_BANKS][NUM_BUTTONS];                       // Estado de los botones
byte digitalOutState[MAX_BANKS][NUM_LEDS];                      // Estado de los botones
byte currentProgram[MAX_BANKS][16] = {};

// Shifters, buttons and LEDs
byte Input, Output, Check=1;
byte prevInputsState = 0;
byte newInputsState = 0;
unsigned long antMillisCheckShiftButtons = 0;
const byte inputCommonPin = 10;

// Contadores, flags //////////////////////////////////////////////////////////////////////
byte mux, muxChannel;                       // Contadores para recorrer los multiplexores
byte numBanks, numInputs, numOutputs;
byte prevBank, currBank = 0;
bool ledModeMatrix, newBank = true, changeDigOutState, midiThru;
//bool ultrasoundPresent;
bool firstBoot = false, firstRead = true;
bool blinkStatusLEDon = 0, configMode = 0, receivingSysEx = 0;
bool outputBlinkState = 0;
unsigned long millisPrevLED = 0;               // Variable para guardar ms
static byte blinkStatusCount = 0;
uint16_t packetSize;
unsigned long prevBlinkMillis = 0;
///////////////////////////////////////////////////////////////////////////////////////////

#if defined(DEUG)
unsigned long sumTimes = 0;
unsigned long avgTimes = 0;
unsigned int numTimeReadings = 5000;
unsigned int counter = 0;
byte loopAvgCounter = 0;
unsigned long antMicrosLoop, loopTime;
#endif

void setup() {
#if defined(MIDI_COMMS)
  MIDI.begin(MIDI_CHANNEL_OMNI); // Se inicializa la comunicación MIDI.
#else
  Serial.begin(250000);                  // Se inicializa la comunicación serial.
#endif

  // Initialize Kilomux shield
  KMShield.init();
  KMShield.setADCprescaler(PS_32);    // Override initial setting
  // Initialize EEPROM handling library
  KMS::initialize();
  
  prevBlinkMillis = millis();

#if !defined(MIDI_COMMS)
  Serial.print("Kilowhat protocol: v"); Serial.println(gD.protocolVersion());
#endif
      
  ResetConfig(CONFIG_OFF);

  #if defined(DEUG)
    #if defined(MIDI_COMMS)
    delay(10000);
    LedWrite(7, HIGH); delay(200); LedWrite(7, LOW); delay(200); LedWrite(7, HIGH); delay(200); LedWrite(7, LOW);
    #else
    avgTimes = EEPROM.readLong(500);
    Serial.println(avgTimes);  
    while(1);
    #endif
  #endif
}

void loop() {
// DEBUG ////////////////////////////
#if defined(DEUG)
   antMicrosLoop = micros();
#endif
/////////////////////////////////////
#if defined(MIDI_COMMS)
  if (MIDI.read()){
    ReadMidi();
  }
#else
  if (Serial.available()){
    ReadSerial();
  }
#endif
  
  if (!firstBoot) {
    if (blinkStatusLEDon && blinkStatusCount) StatusLEDupdate();
    
    if (!receivingSysEx){
      
      UpdateDigitalOutputs();
      if (millis() - prevBlinkMillis > OUTPUT_BLINK_INTERVAL) {         // Si transcurrieron más de X ms desde la ultima actualización,
        ToggleBlinkOutputs();
      }
      
      if(millis()-antMillisCheckShiftButtons > INTERVALO_CHECK_SHIFT_INPUTS_MS){
        antMillisCheckShiftButtons = millis();
        buttonScan();
      }
      // if (ultrasoundPresent) ReadSensorUS();
      
      ReadInputs();
    }
  }
  else{
    /*
     * FIRST BOOT MODE: Status LED blinks waiting for configuration.
     */
    
    if(!(millis() % 500)){
      blinkStatusLEDon = 1;
      blinkStatusCount = 1;
    }
    if (blinkStatusLEDon && blinkStatusCount) StatusLEDupdate();
    ReadInputs(); 
  }

#if defined(DEBUG)
  loopTime = micros()-antMicrosLoop;
  if(counter < numTimeReadings){
    sumTimes += loopTime;
    counter++;
  }
  else{
    avgTimes = sumTimes/numTimeReadings;
    #if defined(MIDI_COMMS)
    EEPROM.writeLong(500, avgTimes);
    sumTimes = 0; 
    avgTimes = 0; 
    counter = 0;
    LedWrite(7, HIGH); delay(200); LedWrite(7, LOW); delay(200); LedWrite(7, HIGH); delay(200); LedWrite(7, LOW);
    while(1);
    #else
    Serial.println("");
    Serial.print("Promedio de loop times para ");Serial.print(numTimeReadings);Serial.print(" loops: ");Serial.println(avgTimes);
    #endif
  }
#endif
/////////////////////////////////////
}

/*
 *  Reload configuration from EEPROM.
 */
void ResetConfig(bool newConfig) {
  pinMode(LED_BUILTIN, OUTPUT);
  // Reload global data from EEPROM
  gD = KMS::globalData();

  currBank = 0;
  KMS::setBank(currBank);
  configMode = newConfig;
  firstRead = true;
  
  if (gD.isValid()) {
    
    if (gD.protocolVersion() == 1){
      packetSize = 57;
    }else{
      packetSize = 57; // Default.
    }

    // Off first-boot mode
    firstBoot = false;
    
    //Shifters and LEDs and buttons
    pinMode(inputCommonPin, INPUT);//Input from buttons
    KMShield.digitalWritePortsKm(0xFF,0x00);
    prevInputsState = digitalRead(inputCommonPin);

    // Load general configuration
    ledModeMatrix = gD.hasOutputMatrix();
    numBanks = gD.numBanks();
    numInputs = gD.numInputsNorm();
    numOutputs = gD.numOutputs();
    midiThru = gD.midiThru();

    #if defined(MIDI_COMMS)
    if(midiThru)
      MIDI.turnThruOn();             // Turn midi thru on, because configuration says so
    else
      MIDI.turnThruOff();            // Turn midi thru off, because configuration says so
    #endif
    
//    ultrasonicSensorData = KMS::ultrasound();
//    ultrasoundPresent = ultrasonicSensorData.mode() != KMS::M_OFF;
//    if (ultrasoundPresent) {
//      FilterClear();
//      minMicroSecSensor = ultrasonicSensorData.dist_min() * US_ROUNDTRIP_CM;
//      maxMicroSecSensor = ultrasonicSensorData.dist_max() * US_ROUNDTRIP_CM;
//      usSensor.changeMaxDistance(ultrasonicSensorData.dist_max());
//      sensorActive = false;
//      pingSensorTimer = millis() + pingSensorInterval;
//      pinMode(SensorEchoPin, INPUT_PULLUP);
//      pinMode(SensorTriggerPin, OUTPUT);
//      pinMode(ActivateSensorButtonPin, INPUT_PULLUP);
//      pinMode(ActivateSensorLedPin, OUTPUT);
//      digitalWrite(ActivateSensorLedPin, LOW);
//    }

#if !defined(MIDI_COMMS)
    Serial.print("Numero de bancos: "); Serial.println(numBanks);
    Serial.print("Numero de entradas: "); Serial.println(numInputs);
    Serial.print("Numero de salidas: "); Serial.println(numOutputs);
    Serial.print("MIDI thru? "); Serial.println(gD.midiThru()? "SI":"NO");
    Serial.print("Sensor mode: "); Serial.println(MODE_LABEL(ultrasonicSensorData.mode()));
#endif
  }
  else {
#if !defined(MIDI_COMMS)
    Serial.print("Datos en EEPROM no válidos");
#endif
    // If there is no valid data on EEPROM, enter first-boot mode.
    firstBoot = true;
    packetSize = 57;
  }  
  // Inicializar lecturas
  for (mux = 0; mux < NUM_MUX; mux++) {
    for (muxChannel = 0; muxChannel < NUM_MUX_CHANNELS; muxChannel++) {
      KMShield.muxReadings[mux][muxChannel] = KMShield.analogReadKm(mux, muxChannel);
      KMShield.muxPrevReadings[mux][muxChannel] = KMShield.analogReadKm(mux, muxChannel);
      //Serial.print("Mux actual: ");Serial.print(KMShield.muxReadings[mux][muxChannel]); Serial.print(" Mux prev: ");Serial.println(KMShield.muxPrevReadings[mux][muxChannel]);
    }
  }
  for (byte bank = 0; bank < MAX_BANKS; bank++) {
    for (byte i = 0; i < NUM_MUX * NUM_MUX_CHANNELS; i++) {
      digitalInputState[bank][i] = 0;
    }
    for (byte i = 0; i < NUM_BUTTONS; i++) {
      buttonStateBanks[bank][i] = 0;
      digitalOutState[bank][i] = 0;
    }
    for (byte i = 0; i < 16; i++) {
      currentProgram[bank][i] = 0;
    }
  }
  changeDigOutState = true;
}

void StatusLEDupdate() {
  static bool lastLEDState = LOW;
  static unsigned long millisPrev = 0;
  static bool firstTime = true;

  if (firstTime) {
    firstTime = false;
    millisPrev = millis();
  }
  if (millis() - millisPrev > STATUS_BLINK_INTERVAL) {
    millisPrev = millis();
    LedWrite(7, !lastLEDState);
    //digitalWrite(13, !lastLEDState);
    lastLEDState = !lastLEDState;
    if (lastLEDState == LOW) blinkStatusCount--;
    if (!blinkStatusCount) {
      blinkStatusLEDon = 0;
      firstTime = true;
    }
  }
  return;
}

void UpdateDigitalOutputs() {
  uint16_t dOut = 0;
  
  if (newBank || changeDigOutState) {
    changeDigOutState = false;

    if(newBank && !configMode){                                                  // If new bank flag is on
      newBank = false;                                              // turn it off
      for (int bankLED = 0; bankLED < MAX_BANKS; bankLED++){        // and cycle through the LEDs to check which bank is on and which one is off
        if(currBank == bankLED){                                    // If the current bank matches this LED
          LedWrite(7-bankLED, HIGH);                                  // turn this LED on.  (7-LED) because they are numbered backwards and LED 0 is on output 7 and so on...
        }else{                                                      // If this LED does not match the current bank selected,
          LedWrite(7-bankLED, LOW);                                   // turn this LED off.
        }
      }
    }

    // Turn normal LEDs on or OFF
    for (dOut = 0; dOut < NUM_LEDS; dOut++) {                       // Cycle the rest of the LEDs for Miniblock
      if (digitalOutState[currBank][dOut] == OUT_ON)          // If this bank has this LED on, because the bank changed and it was originally on, or because it arrived a matching MIDI message
        LedWrite(3-dOut, HIGH);                                 // turn this LED on. (3-LED) because they are numbered backwards, LED 0 is on output 3 and so on...
      else if (digitalOutState[currBank][dOut] == OUT_OFF)    // If this LED is meant to be off,
        LedWrite(3-dOut, LOW);                                  // turn this LED off.
    }
  }
}

void ToggleBlinkOutputs(void) {
  uint16_t dOut = 0;
  for (dOut = 0; dOut < NUM_LEDS; dOut++) {                       // Cycle through every LED
    if (digitalOutState[currBank][dOut] == OUT_BLINK) {     // If this output is set to blink
      if (outputBlinkState) {                                 // and it was on
        LedWrite(3-dOut, LOW);                                  // turn it off.
      }
      else {                                                  // If it was off.
        LedWrite(3-dOut, HIGH);                                 // turn it on.
      }
    }
  }
  prevBlinkMillis = millis();                     // Update millis counter for next blink.
  outputBlinkState = !outputBlinkState;           // Update LED blink state
}

#if defined(MIDI_COMMS)
// Lee el canal midi, note y velocity, y actualiza el estado de los leds.
void ReadMidi(void) {
  switch (MIDI.getType()) {
    case midi::NoteOn:
      HandleNotes(); break;
    case midi::NoteOff:
      HandleNotes(); break;
    case midi::SystemExclusive:
      int sysexLength = 0;
      const byte *pMsg;
    
      sysexLength = (int) MIDI.getSysExArrayLength();
      pMsg = MIDI.getSysExArray();

      char sysexID[3];
      sysexID[0] = (char) pMsg[1];
      sysexID[1] = (char) pMsg[2];
      sysexID[2] = (char) pMsg[3];

      char command = pMsg[4];

      if (sysexID[0] == 'Y' && sysexID[1] == 'T' && sysexID[2] == 'X') {

        if (command == CONFIG_MODE && !configMode) {           // Enter config mode
          blinkStatusLEDon = 1;
          blinkStatusCount = 1;
          const byte configAckSysExMsg[5] = {'Y', 'T', 'X', CONFIG_ACK, 0};
          MIDI.sendSysEx(5, configAckSysExMsg, false);
          ResetConfig(CONFIG_ON);
        }
        else if (command == EXIT_CONFIG && configMode) {           // Enter config mode
          blinkStatusLEDon = 1;
          blinkStatusCount = 1;
          const byte configAckSysExMsg[5] = {'Y', 'T', 'X', EXIT_CONFIG_ACK, 0};
          MIDI.sendSysEx(5, configAckSysExMsg, false);
          ResetConfig(CONFIG_OFF);
        }
        else if (command == DUMP_TO_HW) {           // Save dump data
          if (!receivingSysEx) {
            receivingSysEx = 1;
            MIDI.turnThruOff();
          }

          KMS::io.write(packetSize * pMsg[5], pMsg + 6, sysexLength - 7); // pMsg has index in byte 6, total sysex packet has max.
          // |F0, 'Y' , 'T' , 'X', command, index, F7|
          blinkStatusLEDon = 1;
          blinkStatusCount = 1;

          if (sysexLength < packetSize + 7) { // Last message?
            receivingSysEx = 0;
            blinkStatusLEDon = 1;
            blinkStatusCount = 3;
            const byte dumpOkMsg[5] = {'Y', 'T', 'X', DUMP_OK, 0};
            MIDI.sendSysEx(5, dumpOkMsg, false);
            const byte configAckSysExMsg[5] = {'Y', 'T', 'X', EXIT_CONFIG_ACK, 0};
            MIDI.sendSysEx(5, configAckSysExMsg, false);
            ResetConfig(CONFIG_OFF);
          }
        }
      }
      break;
  }
}
#else
void ReadSerial(){
  char inChar = (char) Serial.read();
  if (inChar == 'c' && !configMode) {
    //configMode = true;
    ResetConfig(CONFIG_ON);
  }
  else if (inChar == 'x' && configMode) {
    //configMode = false;
    ResetConfig(CONFIG_OFF);
  }
}
#endif

#if defined(MIDI_COMMS)
void HandleNotes() {
  byte data1, data2, channel;
  data1 = MIDI.getData1();
  data2 = MIDI.getData2();
  channel = MIDI.getChannel();    // Channel 1-16
  if (!configMode) {
    for (byte outputIndex = 0; outputIndex < NUM_LEDS; outputIndex++) {
      KMS::Output outputData = KMS::output(outputIndex);
      if (data1 == outputData.param() && channel == outputData.channel()) {
        if (MIDI.getType() == midi::NoteOn) {
          if (outputData.blink() && data2 >= outputData.blink_min() && data2 <= outputData.blink_max()) {
            digitalOutState[currBank][outputIndex] = OUT_BLINK;
            outputBlinkState = false;
          } else if (outputData.blink() && data2 && (data2 < outputData.blink_min() || data2 > outputData.blink_max())) {
            digitalOutState[currBank][outputIndex] = OUT_ON;
          } else if (outputData.blink() && !data2) {
            digitalOutState[currBank][outputIndex] = OUT_OFF;
          } else {       // blink off
            if (data2) {
              digitalOutState[currBank][outputIndex] = OUT_ON;
            } else {
              digitalOutState[currBank][outputIndex] = OUT_OFF;
            }
          }
        } else if (MIDI.getType() == midi::NoteOff) {
          if (!data2) {
            digitalOutState[currBank][outputIndex] = OUT_OFF;
          }
        }
        changeDigOutState = true;
        break;
      }
    }
  }
  else {
    if (data2)
      LedWrite(3-data1, HIGH);
    else
      LedWrite(3-data1, LOW);
  }
}
#endif

void buttonScan(void){ 
  static bool buttonPrevState[8] = {0,0,0,0,0,0,0,0};
  bool buttonPressed = digitalRead(inputCommonPin);   // Read common pin to know if there is a button pressed
  if(buttonPressed){                                  // if common pin is HIGH, then there is actually a button pressed
    Check = 1;                                          // First button to check is on pin 0 (Check is 00000001 right here) 
    for(int j=0; j<8; j++){                             // then, to know which button was pressed, we need to cycle through every shift reg. pin
      KMShield.digitalWritePortsKm(Check, Output);        // set a 1 to only one of the outputs
    
      if(digitalRead(inputCommonPin) == HIGH){             // If common pin is still HIGH, then this button was pressed
        bitWrite(newInputsState, j, 1);                     // Update inputs state variable with a 1 on bit j, which is the current output pin
      }
      else{
        bitWrite(newInputsState, j, 0);                     // else (if common pin is low), this button was not pressed
      }

      Check = Check<<1;                                   // Shift the 1 one position to the left, so we check the next button. (Check is now 00000001 << j)
    }// for
    KMShield.digitalWritePortsKm(0xFF, Output);         // After the whole check to every button, set all outputs to HIGH, so we can detect a new press
  }
  else{
    newInputsState = 0;
  }

  // Check then new state for buttons and act accordingly
  for(int buttonPin = 0; buttonPin < 8; buttonPin++){                                 
    bool buttonState = bitRead(newInputsState, buttonPin);
        
    if(buttonState != buttonPrevState[buttonPin]){
      if(buttonPin == SHIFTER_B0 || buttonPin == SHIFTER_B1 || buttonPin == SHIFTER_B2 || buttonPin == SHIFTER_B3){
        byte bankPressed = 7-buttonPin;
        static bool bankButtonPressed;
        currBank = KMS::bank();
        if (buttonState && currBank != bankPressed && bankPressed <= KMS::realBanks()) {
          prevBank = currBank;
          KMS::setBank(bankPressed);
          currBank = bankPressed;
          newBank = true;
          #if !defined(MIDI_COMMS)
          Serial.println("");
          Serial.print("Current Bank: "); Serial.print(KMS::bank());
          Serial.print("\t   Previous bank: "); Serial.println(prevBank);
          #endif
        }
      }else{                                                      // Not shifter
        byte buttonPressed = 3-buttonPin;                           // Button 0 is on pin 3, button 1 is on pin 2, button 2 is on pin 1, button 3 is on pin 0.
        byte inputIndex = 32+buttonPressed;
        KMS::InputNorm inputData = KMS::input(inputIndex);    // Button configuration will always be at position 32, 33, 34 and 35 for the bank.
        if (inputData.mode() != KMS::M_OFF) {
          bool toggle = inputData.toggle();                           // is the button set as toggle?
          if (buttonState) {    // If the button was pressed
            buttonStateBanks[currBank][buttonPressed] = !buttonStateBanks[currBank][buttonPressed];     // MODO TOGGLE: Cambia de 0 a 1, o viceversa
                                                                                                        // MODO NORMAL: Cambia de 0 a 1
            InputChanged(inputIndex, inputData, !buttonStateBanks[currBank][buttonPressed]);
            //digitalOutState[currBank][buttonPressed] = OUT_ON;
          }
          else if (!buttonState && !toggle) {
            buttonStateBanks[currBank][buttonPressed] = 0;
            InputChanged(inputIndex, inputData, !buttonStateBanks[currBank][buttonPressed]);
           // digitalOutState[currBank][buttonPressed] = OUT_OFF;
          }
        }
       // changeDigOutState = true;
      }
      buttonPrevState[buttonPin] = buttonState;
    }
  }
}

bool LedState(byte led){
  return bitRead(Output, led);
}
void LedWrite(byte led, byte state){
  bitWrite(Output, led, state);
  KMShield.digitalWritePortsKm(0xFF, Output); 
}

//void ReadSensorUS() {
//  static uint16_t prevMillisUltraSensor = 0;            // Variable usada para almacenar los milisegundos desde el último Ping al sensor
//  static bool activateSensorButtonPrevState = HIGH;     // Inicializo inactivo (entrada activa baja)
//  static bool activateSensorButtonState = HIGH;         // Inicializo inactivo (entrada activa baja)
//  static bool sensorLEDState = LOW;                     // Inicializo inactivo (salida activa alta)
//
//  // Este codigo verifica si se presionó el botón y activa o desactiva el sensor cada vez que se presiona
//  activateSensorButtonState = digitalRead(ActivateSensorButtonPin);
//  if (activateSensorButtonState == LOW && activateSensorButtonPrevState == HIGH) {  // Si el botón previamente estaba en estado alto, y ahora esta en estado bajo, quiere decir que paso de estar no presionado a presionado (activo bajo)
//    activateSensorButtonPrevState = LOW;                                // Actualizo el estado previo
//    sensorActive = !sensorActive;                      // Activo o desactivo el sensor
//    sensorLEDState = !sensorLEDState;                                // Cambio el estado del LED
//    digitalWrite(ActivateSensorLedPin, sensorLEDState);               // Y actualizo la salida
//  }
//  else if (activateSensorButtonState == HIGH && activateSensorButtonPrevState == LOW) { // Si el botón previamente estaba en estado bajo, y ahora esta en estado alto, quiere decir que paso de estar presionado a no presionado
//    activateSensorButtonPrevState = HIGH;                                  // Actualizo el estado previo
//  }
//  
//  if(firstRead){
//    sensorLEDState = LOW; 
//    activateSensorButtonPrevState = HIGH;
//    activateSensorButtonState = HIGH;
//    return;
//  }
//  
//  if (sensorActive) {                                     // Si el sensor está activado
//    if (millis() >= pingSensorTimer) {   // y transcurrió el delay minimo entre lecturas
//      pingSensorTimer += pingSensorInterval;
//      usSensor.ping_timer(EchoCheck);                           // Sensar el tiempo que tarda el pulso de ultrasonido en volver. Se recibe el valor el us.
//    }
//  }else
//    pingSensorTimer = millis() + pingSensorInterval;
//  
//}
//
//void EchoCheck(){
//  uint8_t rc = usSensor.check_timer();
//  if(rc != 0){
//    uSeg = usSensor.ping_result;
//    uSeg = constrain(uSeg, minMicroSecSensor, maxMicroSecSensor);
//    uint16_t sensorRange = maxMicroSecSensor - minMicroSecSensor;
//    uint16_t usSensorValue = map(uSeg, minMicroSecSensor, maxMicroSecSensor, 0, sensorRange);  
//    
//    byte mode = ultrasonicSensorData.mode();
//    byte midiChannel = ultrasonicSensorData.channel();
//    byte param = ultrasonicSensorData.param_fine();
//    byte minMidi = ultrasonicSensorData.param_min_coarse();
//    byte maxMidi = ultrasonicSensorData.param_max_coarse();
//    
//    if (minMidi < maxMidi)
//      usSensorValue = map(usSensorValue,  0,
//                                          sensorRange,
//                                          mode == KMS::M_NRPN ? minMidi << 7        : minMidi,
//                                          mode == KMS::M_NRPN ? (maxMidi << 7) | 0x7F : maxMidi);
//    else
//      usSensorValue = map(usSensorValue,  0,
//                                          sensorRange,
//                                          mode == KMS::M_NRPN ? (minMidi << 7) | 0x7F : minMidi,
//                                          mode == KMS::M_NRPN ? maxMidi << 7        : maxMidi);
//
//    // FILTRO DE MEDIA MÓVIL PARA SUAVIZAR LA LECTURA
//    usSensorValue = FilterGetNewAverage(usSensorValue);
//    
//    // Detecto si cambió el valor filtrado, para no mandar valores repetidos
//    if (usSensorValue != usSensorPrevValue) {
//      usSensorPrevValue = usSensorValue;
//      #if defined(MIDI_COMMS)
//      if(configMode){
//        MIDI.sendControlChange(100, usSensorValue, 1);
//      }
//      else{
//        switch (mode) {
//          case (KMS::M_NOTE):
//            MIDI.sendNoteOn(param, usSensorValue, midiChannel); break;
//          case (KMS::M_CC):
//            MIDI.sendControlChange(param, usSensorValue, midiChannel); break;
//          case KMS::M_NRPN:
//            MIDI.sendControlChange( 101, ultrasonicSensorData.param_coarse(), midiChannel);
//            MIDI.sendControlChange( 100, ultrasonicSensorData.param_fine(), midiChannel);
//            MIDI.sendControlChange( 6, (usSensorValue >> 7) & 0x7F, midiChannel);
//            MIDI.sendControlChange( 38, (usSensorValue & 0x7F), midiChannel); break;
//          default: break;
//        }
//      }
//      #else
//        Serial.print("Channel: "); Serial.print(midiChannel); Serial.print("\t");
//        Serial.print("Modo: "); Serial.print(MODE_LABEL(mode)); Serial.print("\t");
//        Serial.print("Parameter: "); Serial.print((ultrasonicSensorData.param_coarse() << 7) | ultrasonicSensorData.param_fine()); Serial.print("  Valor: "); Serial.println(usSensorValue);
//      #endif
//    }
//  }
//  else{
//    
//  }
//}
//
//// Running average filter (from RunningAverage arduino lib, but without the lib)
//uint16_t FilterGetNewAverage(uint16_t newVal){
//  filterSum -= filterSamples[filterIndex];
//  filterSamples[filterIndex] = newVal;
//  filterSum += filterSamples[filterIndex];
//  filterIndex++;
//  if (filterIndex == US_SENSOR_FILTER_SIZE) filterIndex = 0;  // faster than %
//  // update count as last otherwise if( _cnt == 0) above will fail
//  if (filterCount < US_SENSOR_FILTER_SIZE) 
//    filterCount++;
//  if (filterCount == 0) 
//    return NAN;
//  return filterSum / filterCount;
//}

//void FilterClear(){
//  filterCount = 0;
//  filterIndex = 0;
//  filterSum = 0;
//  for (uint8_t i = 0; i < US_SENSOR_FILTER_SIZE; i++){
//    filterSamples[i] = 0; // keeps addValue simpler
//  }
//}

/*
   Esta función lee todas las entradas análógicas y/o digitales y almacena los valores de cada una en la matriz 'lecturas'.
   Compara con los valores previos, almacenados en 'lecturasPrev', y si cambian, y llama a las funciones que envían datos.
*/
void ReadInputs() {
  static uint16_t currAnalogValue = 0, prevAnalogValue = 0;
  for (int inputIndex = 0; inputIndex < numInputs-4; inputIndex++) {
    KMS::InputNorm inputData = KMS::input(inputIndex);
    mux = inputIndex < 16 ? MUX_A : MUX_B;           // MUX 0 or 1
    muxChannel = inputIndex % NUM_MUX_CHANNELS;         // CHANNEL 0-15
    if (inputData.mode() != KMS::M_OFF) {
      if (inputData.AD() == KMS::T_ANALOG) {
        KMShield.muxReadings[mux][muxChannel] = KMShield.analogReadKm(mux, muxChannel);           // Si es NRPN leer entradas analógicas 'KMShield.analogReadKm(N_MUX,N_CANAL)'
        
        if (!firstRead && KMShield.muxReadings[mux][muxChannel] != KMShield.muxPrevReadings[mux][muxChannel]) {  // Si lo que leo no es ruido
          InputChanged(inputIndex, inputData, KMShield.muxReadings[mux][muxChannel]);               // Enviar mensaje.
        }
        else{
          KMShield.muxPrevReadings[mux][muxChannel] = KMShield.muxReadings[mux][muxChannel];         // Almacenar lectura actual como anterior, para el próximo ciclo
          continue;
        }
        KMShield.muxPrevReadings[mux][muxChannel] = KMShield.muxReadings[mux][muxChannel];         // Almacenar lectura actual como anterior, para el próximo ciclo
      }
      else if (inputData.AD() == KMS::T_DIGITAL) {
        // CÓDIGO PARA LECTURA DE ENTRADAS DIGITALES
        KMShield.muxReadings[mux][muxChannel] = KMShield.digitalReadKm(mux, muxChannel, PULLUP);      // Leer entradas digitales 'KMShield.digitalReadKm(N_MUX, N_CANAL)'
        bool toggle = inputData.toggle();
        //Serial.print("Mux: "); Serial.print(mux); Serial.print("  Channel: "); Serial.println(channel);
        if (KMShield.muxReadings[mux][muxChannel] != KMShield.muxPrevReadings[mux][muxChannel]) {     // Me interesa la lectura, si cambió el estado del botón,
          KMShield.muxPrevReadings[mux][muxChannel] = KMShield.muxReadings[mux][muxChannel];             // Almacenar lectura actual como anterior, para el próximo ciclo
          
          if (firstRead) continue;
          byte digInputIndex = muxChannel + mux * NUM_MUX_CHANNELS;
          if (!KMShield.muxReadings[mux][muxChannel]) {
            
            digitalInputState[currBank][digInputIndex] = !digitalInputState[currBank][digInputIndex]; // MODO TOGGLE: Cambia de 0 a 1, o viceversa
            // MODO NORMAL: Cambia de 0 a 1
            InputChanged(inputIndex, inputData, !digitalInputState[currBank][digInputIndex]);
          }
          else if (KMShield.muxReadings[mux][muxChannel] && !toggle) {
            digitalInputState[currBank][digInputIndex] = 0;
            InputChanged(inputIndex, inputData, !digitalInputState[currBank][digInputIndex]);
          }
        }
      }
    }
  }
  if(firstRead) firstRead = false;
}

void InputChanged(int numInput, const KMS::InputNorm &inputData, uint16_t rawValue) {
  byte mode = inputData.mode();
  bool analog = inputData.AD();       // 1 is analog
  byte param = inputData.param_fine();
  byte channel = inputData.channel();
  int16_t minMidi = inputData.param_min();
  int16_t maxMidi = inputData.param_max();
  int16_t minMidiNRPN, maxMidiNRPN;
  uint16_t maxMinDiff;
  uint16_t mapValue, constrainedValue;
  uint16_t noiseTh;
  static uint16_t prevValue[NUM_MUX * NUM_MUX_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static uint16_t prevRawValue[NUM_MUX * NUM_MUX_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};                                                           

  #define CONST_LOW_LIMIT   3
  #define CONST_HIGH_LIMIT  1020
  
  if(!configMode){
    if (analog) {
      constrainedValue = constrain(rawValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT);
      if (IsNoise(constrainedValue, prevRawValue[numInput], numInput, 2))   // Pre-filter with raw 10 bit value
        return;
      prevRawValue[numInput] = constrainedValue;  
      Serial.print("Raw value: "); Serial.print(constrainedValue); Serial.println();
      
      if (mode == KMS::M_NRPN){
        minMidiNRPN = pgm_read_word_near(KMS::nrpn_min_max + minMidi);
        maxMidiNRPN = pgm_read_word_near(KMS::nrpn_min_max + maxMidi);

        mapValue = map(constrainedValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT, minMidiNRPN, maxMidiNRPN); 
        
        if(minMidi < maxMidi){
          maxMinDiff = maxMidiNRPN - minMidiNRPN;
          //if (mapValue == maxMidiNRPN-1 && maxMinDiff > 10000) mapValue += 1; 
        }
        else{
          maxMinDiff = minMidiNRPN - maxMidiNRPN ;
          //if (mapValue == minMidiNRPN-1 && maxMinDiff > 10000) mapValue += 1;
        }
        
        noiseTh = maxMinDiff >> 8;          // divide range to get noise threshold. Max th is 127/4 = 64 : Min th is 0.
        
        if (IsNoise(mapValue, prevValue[numInput], numInput, noiseTh)) 
          return;
      }else{
        
        if(minMidi < maxMidi)
          mapValue = map(constrainedValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT, minMidi, maxMidi); 
        else
          mapValue = map(constrainedValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT, minMidi, maxMidi); 
        int maxMinDiff = maxMidi - minMidi;
        noiseTh = abs(maxMinDiff) >> 6;          // divide range to get noise threshold. Max th is 127/64 = 2 : Min th is 0.
        if (IsNoise(mapValue, prevValue[numInput], numInput, 0)) 
          return;
      }
      prevValue[numInput] = mapValue;   // Save value to previous data array
    }
    else {      // DIGITAL INPUTS
      if (mode == KMS::M_NRPN){
        minMidiNRPN = pgm_read_word_near(KMS::nrpn_min_max + minMidi);
        maxMidiNRPN = pgm_read_word_near(KMS::nrpn_min_max + maxMidi);
        if (constrainedValue)   mapValue = minMidiNRPN;   // If value is != 0, then button is off
        else                    mapValue = maxMidiNRPN;   // If value is == 0, the button is on (active LOW)
      }else{
        if (constrainedValue)   mapValue = minMidi;   // If value is != 0, then button is off
        else                    mapValue = maxMidi;   // If value is == 0, the button is on (active LOW)
      }
    }  
  }
  
  // TX LED ON
  // LedWrite(0, HIGH);
#if defined(MIDI_COMMS)
  if (configMode) { // CONFIG MODE MESSAGES
    mapValue = map(constrainedValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT, 0, 127);
    if (IsNoise(mapValue, prevValue[numInput], numInput, 1)) 
      return;
    prevValue[numInput] = mapValue;   // Save value to previous data array
    MIDI.sendControlChange( numInput, mapValue, 1);
  }
  else {            // NORMAL MODE
    switch (mode) {
      case (KMS::M_NOTE):
        MIDI.sendNoteOn(param, mapValue, channel); break;
      case (KMS::M_CC):
        MIDI.sendControlChange(param, mapValue, channel); break;
      case KMS::M_NRPN:
        MIDI.sendControlChange( 101, inputData.param_coarse(), channel);
        MIDI.sendControlChange( 100, inputData.param_fine(), channel);
        MIDI.sendControlChange( 6, (mapValue >> 7) & 0x7F, channel);
        MIDI.sendControlChange( 38, (mapValue & 0x7F), channel); break;
      case KMS::M_PROGRAM_MINUS:
        if(currentProgram[currBank][channel-1] > 0){
          currentProgram[currBank][channel-1]--;
          MIDI.sendProgramChange(currentProgram[currBank][channel-1], channel); 
        } break;
      case KMS::M_PROGRAM:
        if (!analog && mapValue > 0) {
          MIDI.sendProgramChange( param, channel);
        } 
        else if (analog){
          MIDI.sendProgramChange( mapValue, channel);
        } break;
      case KMS::M_PROGRAM_PLUS:
        if(currentProgram[currBank][channel-1] < 127){
          currentProgram[currBank][channel-1]++;
          MIDI.sendProgramChange(currentProgram[currBank][channel-1], channel);
        } break;
      default: break;
    }
  }
#else 
  if (configMode) { // CONFIG MODE MESSAGES
    mapValue = map(constrainedValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT, 0, 127);
    if (IsNoise(mapValue, prevValue[numInput], numInput, 1)) 
      return;
    prevValue[numInput] = mapValue;   // Save value to previous data array  
  }
  Serial.print("Channel: "); Serial.print(channel); Serial.print("\t");
  Serial.print("Tipo: "); Serial.print(inputData.AD() ? "Analog" : "Digital"); Serial.print("\t");
  Serial.print("Min: "); Serial.print(mode == KMS::M_NRPN ? minMidiNRPN : minMidi); Serial.print("\t");
  Serial.print("Max: "); Serial.print(mode == KMS::M_NRPN ? maxMidiNRPN : maxMidi); Serial.print("\t");
  Serial.print("Modo: "); Serial.print(MODE_LABEL(mode)); Serial.print("\t");
  Serial.print("Parameter: "); Serial.print((inputData.param_coarse() << 7) | inputData.param_fine()); 
  Serial.print("\tValor: "); Serial.print(mapValue); Serial.print("\tValor original: "); Serial.print(constrainedValue);  Serial.print("\tValor anterior: "); Serial.println(prevValue[numInput]); 
#endif
  // TX LED OFF
  //LedWrite(0, LOW);
  
  return;
}

/*
   Funcion para filtrar  el ruido analógico de los pontenciómetros. Filtro por histéresis que analiza si el valor crece o decrece, y en el caso de un cambio de dirección,
   decide si es ruido o no, si hubo un cambio superior al valor anterior más el umbral de ruido.

   Recibe: -
*/
uint16_t IsNoise(uint16_t currentValue, uint16_t prevValue, uint16_t input, byte noiseTh) {
  static bool upOrDown[NUM_MUX * NUM_MUX_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  if (upOrDown[input] == ANALOG_UP) {
    if (currentValue > prevValue) {            // Si el valor está creciendo, y la nueva lectura es mayor a la anterior,
      return 0;                        // no es ruido.
    }
    else if (currentValue < prevValue - noiseTh) { // Si el valor está creciendo, y la nueva lectura menor a la anterior menos el UMBRAL
      upOrDown[input] = ANALOG_DOWN;                                     // se cambia el estado a DECRECIENDO y
      return 0;                                                               // no es ruido.
    }
  }
  if (upOrDown[input] == ANALOG_DOWN) {
    if (currentValue < prevValue) { // Si el valor está decreciendo, y la nueva lectura es menor a la anterior,
      return 0;                                        // no es ruido.
    }
    else if (currentValue > prevValue + noiseTh) {  // Si el valor está decreciendo, y la nueva lectura mayor a la anterior mas el UMBRAL
      upOrDown[input] = ANALOG_UP;                                       // se cambia el estado a CRECIENDO y
      return 0;                                                               // no es ruido.
    }
  }
  return 1;         // Si todo lo anterior no se cumple, es ruido.
}
