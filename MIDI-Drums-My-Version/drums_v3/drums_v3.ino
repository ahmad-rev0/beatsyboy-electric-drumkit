/*
 * ArduinoMidiDrums - Modified Version
 * 
 * Modifications by: ahmad-rev0 | https://github.com/ahmad-rev0/
 *
 * Original code: ArduinoMidiDrums by Evan Kale
 * Source: https://github.com/evankale/ArduinoMidiDrums
 *
 * 
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
*/


#include <SoftwareSerial.h>

// Piezo defines
#define NUM_PIEZOS 9

#define SNARE_THRESHOLD 100  // anything < TRIGGER_THRESHOLD is treated as 0
#define HITOM_THRESHOLD 90
#define FLRTOM_THRESHOLD 120
#define MIDTOM_THRESHOLD 90
#define LCYM_BOW_THRESHOLD 80
#define RCYM_BOW_THRESHOLD 70
#define KICK_THRESHOLD 70
#define HHAT_BOW_THRESHOLD 90
#define HHAT_PEDAL_THRESHOLD 70
#define START_SLOT 0  // first analog slot of piezos

// MIDI note defines for each trigger
#define SNARE_NOTE 38
#define HITOM_NOTE 48
#define FLRTOM_NOTE 43
#define LCYM_BOW_NOTE 49
#define RCYM_BOW_NOTE 51
#define KICK_NOTE 36
#define HHAT_BOW_NOTE 46
#define MIDTOM_NOTE 47
#define RCYM_BELL_NOTE 53
#define LCYM_BELL_NOTE 53
#define RCRASH_CHOKE_NOTE 58
#define HHAT_HALF_NOTE 44 
#define HHAT_CLOSE_NOTE 42
#define HHAT_PEDAL_NOTE 65

// MIDI defines
#define NOTE_ON_CMD 0x90
#define NOTE_OFF_CMD 0x80
#define MAX_MIDI_VELOCITY 127

// Program defines
// ALL TIME MEASURED IN MILLISECONDS
#define SIGNAL_BUFFER_SIZE 100
#define PEAK_BUFFER_SIZE 30
#define MAX_TIME_BETWEEN_PEAKS 20
#define MIN_TIME_BETWEEN_NOTES 50
#define DEBOUNCE_TIME 0  // Minimum interval between valid hits

// map that holds the mux slots of the piezos
unsigned short slotMap[NUM_PIEZOS];

// map that holds the respective note to each piezo
unsigned short noteMap[NUM_PIEZOS];

// map that holds the respective threshold to each piezo
unsigned short thresholdMap[NUM_PIEZOS];

// Ring buffers to store analog signal and peaks
short currentSignalIndex[NUM_PIEZOS];
short currentPeakIndex[NUM_PIEZOS];
unsigned short signalBuffer[NUM_PIEZOS][SIGNAL_BUFFER_SIZE];
unsigned short peakBuffer[NUM_PIEZOS][PEAK_BUFFER_SIZE];

boolean noteReady[NUM_PIEZOS];
unsigned short noteReadyVelocity[NUM_PIEZOS];
boolean isLastPeakZeroed[NUM_PIEZOS];

unsigned long lastPeakTime[NUM_PIEZOS];
unsigned long lastNoteTime[NUM_PIEZOS];

// Velocity multipliers for all piezos
float multipliers[NUM_PIEZOS] = {
  30.0,  // RCYM_BOW
  1.0,  // FLRTOM
  30.0,  // HHAT_BOW
  1.0,  // MIDTOM
  30.0,  // LCYM_BOW
  1.0,  // SNARE
  30.0,  // HHAT_PEDAL
  1.0,  // HITOM
  30.0   // KICK
};

void setup() {
  Serial.begin(115200); // Use a high baud rate for better performance

  // Adjust ADC prescaler for faster analogRead
#if defined(ADCSRA)
  ADCSRA = (ADCSRA & 0xF8) | 0x04; // Set prescaler to 16
#endif

  // initialize globals
  for (short i = 0; i < NUM_PIEZOS; ++i) {
    currentSignalIndex[i] = 0;
    currentPeakIndex[i] = 0;
    memset(signalBuffer[i], 0, sizeof(signalBuffer[i]));
    memset(peakBuffer[i], 0, sizeof(peakBuffer[i]));
    noteReady[i] = false;
    noteReadyVelocity[i] = 0;
    isLastPeakZeroed[i] = true;
    lastPeakTime[i] = 0;
    lastNoteTime[i] = 0;
    slotMap[i] = START_SLOT + i;
  }

  thresholdMap[0] = RCYM_BOW_THRESHOLD;
  thresholdMap[5] = SNARE_THRESHOLD;
  thresholdMap[1] = FLRTOM_THRESHOLD;
  thresholdMap[3] = MIDTOM_THRESHOLD;
  thresholdMap[4] = LCYM_BOW_THRESHOLD;
  thresholdMap[6] = HHAT_PEDAL_THRESHOLD;
  thresholdMap[8] = KICK_THRESHOLD;
  thresholdMap[7] = HITOM_THRESHOLD;
  thresholdMap[2] = HHAT_BOW_THRESHOLD;

  noteMap[0] = RCYM_BOW_NOTE;
  noteMap[1] = FLRTOM_NOTE; 
  noteMap[3] = MIDTOM_NOTE;
  noteMap[4] = LCYM_BOW_NOTE;
  noteMap[5] = SNARE_NOTE;
  noteMap[6] = HHAT_PEDAL_NOTE;
  noteMap[2] = HHAT_BOW_NOTE;
  noteMap[7] = HITOM_NOTE;
  noteMap[8] = KICK_NOTE;
}

void loop() {
  unsigned long currentTime = millis();

  // Piezo sensing loop
  for (short i = 0; i < NUM_PIEZOS; ++i) {
    unsigned short newSignal = analogRead(slotMap[i]);
    signalBuffer[i][currentSignalIndex[i]] = newSignal;

    if (newSignal < thresholdMap[i]) {
      if (!isLastPeakZeroed[i] && (currentTime - lastPeakTime[i]) > MAX_TIME_BETWEEN_PEAKS) {
        recordNewPeak(i, 0);
      } else {
        short prevSignalIndex = currentSignalIndex[i] - 1;
        if (prevSignalIndex < 0) prevSignalIndex = SIGNAL_BUFFER_SIZE - 1;
        unsigned short prevSignal = signalBuffer[i][prevSignalIndex];
        unsigned short newPeak = 0;

        while (prevSignal >= thresholdMap[i]) {
          if (signalBuffer[i][prevSignalIndex] > newPeak) {
            newPeak = signalBuffer[i][prevSignalIndex];
          }
          prevSignalIndex--;
          if (prevSignalIndex < 0) prevSignalIndex = SIGNAL_BUFFER_SIZE - 1;
          prevSignal = signalBuffer[i][prevSignalIndex];
        }

        if (newPeak > 0) {
          recordNewPeak(i, newPeak);
        }
      }
    }

    currentSignalIndex[i]++;
    if (currentSignalIndex[i] == SIGNAL_BUFFER_SIZE) currentSignalIndex[i] = 0;
  }
}

void recordNewPeak(short slot, short newPeak) {
  isLastPeakZeroed[slot] = (newPeak == 0);

  unsigned long currentTime = millis();
  lastPeakTime[slot] = currentTime;
  peakBuffer[slot][currentPeakIndex[slot]] = newPeak;

  short prevPeakIndex = currentPeakIndex[slot] - 1;
  if (prevPeakIndex < 0) prevPeakIndex = PEAK_BUFFER_SIZE - 1;
  unsigned short prevPeak = peakBuffer[slot][prevPeakIndex];

  if (newPeak > prevPeak && (currentTime - lastNoteTime[slot]) > MIN_TIME_BETWEEN_NOTES) {
    noteReady[slot] = true;
    if (newPeak > noteReadyVelocity[slot])
      noteReadyVelocity[slot] = newPeak;
  } else if (newPeak < prevPeak && noteReady[slot]) {
    if ((currentTime - lastNoteTime[slot]) > DEBOUNCE_TIME) { // Debouncing check
      noteFire(noteMap[slot], noteReadyVelocity[slot], slot);
      noteReady[slot] = false;
      noteReadyVelocity[slot] = 0;
      lastNoteTime[slot] = currentTime;
    }
  }

  currentPeakIndex[slot]++;
  if (currentPeakIndex[slot] == PEAK_BUFFER_SIZE) currentPeakIndex[slot] = 0;
}

void noteFire(unsigned short note, unsigned short velocity, short slot) {
  // Apply multipliers
  velocity = min(velocity * multipliers[slot], (float)MAX_MIDI_VELOCITY);

  if (velocity > MAX_MIDI_VELOCITY)
    velocity = MAX_MIDI_VELOCITY;

  midiNoteOn(note, velocity);
  midiNoteOff(note, velocity);
}

void midiNoteOn(byte note, byte midiVelocity) {
  Serial.write(NOTE_ON_CMD);
  Serial.write(note);
  Serial.write(midiVelocity);
}

void midiNoteOff(byte note, byte midiVelocity) {
  Serial.write(NOTE_OFF_CMD);
  Serial.write(note);
  Serial.write(midiVelocity);
}
