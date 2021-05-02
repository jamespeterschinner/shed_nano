#include <avr/pgmspace.h>
#include <Encoder.h>
#include <Arduino.h>
#include <SPI.h>
#include "cie1931.h"

int CIE_ARRAY_SIZE = 495;
int LIGHTS_PUSHBUTTON_PIN = 14;          // the number of the input pin Light momentary
int LIGHTS_RELAY_PIN = 15;        // the number of the output pin top relay
int LATCH_PIN = 10;               // Pin used to latch the registers of the TLC5947 16
int ENCODER_A_PIN = 2;
int ENCODER_B_PIN = 3;

class TLC5947 {
  private:
    uint8_t grayscaleData[36] = {0}; // This will initialize all elements to zero
    SPISettings spiSettings{};
    uint8_t latchPin;
  public:
    TLC5947(uint32_t baud, uint8_t latchPin) : spiSettings(baud, MSBFIRST, SPI_MODE0), latchPin(latchPin) {

    }

    void begin() {
      // Set up SPI communication to TLC
      memset(grayscaleData, 255, 36);
      updateBrightness();
    }

    void selectBrightness(uint8_t channel, uint16_t brightness) {
      // Get the bit position of the channel
      uint16_t bitPosition = channel * 12;
      // Get the index of the channel in the tlc grayscale array
      uint8_t tlcGSArrayIndex = bitPosition / 8;

      // put the brightness in the array.
      // For this, check if the channel is an even or uneven number
      if ((channel & 0x1) == 0) {
        grayscaleData[tlcGSArrayIndex] = brightness & 0xFF;
        uint8_t temp = grayscaleData[tlcGSArrayIndex + 1];
        temp &= 0xF0;
        temp |= (brightness >> 8) & 0x0F;
        grayscaleData[tlcGSArrayIndex + 1] = temp;
      } else {
        uint8_t temp = grayscaleData[tlcGSArrayIndex];
        temp &= 0x0F;
        temp |= (brightness & 0x0F) << 4;
        grayscaleData[tlcGSArrayIndex] = temp;
        grayscaleData[tlcGSArrayIndex + 1] = (brightness >> 4) & 0xFF;
      }
    }

    void pulseLatch() const {
      digitalWrite(latchPin, HIGH);
      digitalWrite(latchPin, LOW);
    }

    void updateBrightness() const {
      SPIClass::beginTransaction(spiSettings);
      for (int8_t i = 35; i >= 0; i--) {
        SPIClass::transfer(grayscaleData[i]);
      }
      pulseLatch();
    }


    void updateAll(uint16_t brightness) {
      noInterrupts();
      for (int8_t i = 23; i >= 0; i--) {
        selectBrightness(i, brightness);
      }
      updateBrightness();
      interrupts();
    }


};

class Delay {
  private:
    unsigned long initial_time = 0;
    unsigned long delay_time_ms{};
    bool timing = false;
    bool doneTiming = false;
  public:
    void start(unsigned long delay_ms) {
      timing = true;
      doneTiming = false;
      delay_time_ms = delay_ms;
      initial_time = millis();
    }

    bool timerTiming() {
      done();
      return timing;

    }

    bool inlineDelay(long delay_ms) {
      timing = true;
      delay_time_ms = delay_ms;
      if ((initial_time == 0) || millis() - initial_time > delay_time_ms) {
        initial_time = millis();
        return true;
      } else {
        return false;
      }
    }

    bool done() {
      if (doneTiming) {
        return true;
      } else if (timing && millis() - initial_time > delay_time_ms) {
        timing = false;
        doneTiming = true;
        initial_time = 0;
        return true;
      } else {
        return false;
      }
    }

};

class RisingEdge {
  private:
    int previous_value = LOW;
  public:
    bool risingEdge(int current_value) {
      if (previous_value == LOW && current_value == HIGH) {
        previous_value = current_value;
        return true;
      } else {
        previous_value = current_value;
        return false;
      }
    }
};

class Toggle {
  private:
    bool state = false;
  public:
    bool toggle() {
      state = !state;
      return state;
    }

};

class Change {
  private:
    bool initial = false;
    int value{};
  public:
    bool valueChanged(int newValue) {
      if (!initial) {
        initial = true;
        value = newValue;
        return true;
      } else if (newValue != value) {
        value = newValue;
        return true;
      } else {
        return false;
      }

    }
};


class Dimmer {
  private:

    bool softStartRunning = false;
    int cieIndexSetPoint = 0;
    int cieIndex = 0;
    bool fineControl = false;
    Change pwmChange;
    Delay softStartOnRate;
    Delay tlcUpdateDelay;
    Delay rotationSpeed;


  public:

    void off() {
      cieIndex = 0;
      cieIndexSetPoint = 0;
      softStartRunning = false;

    }

    [[nodiscard]] uint16_t cieToPwm() const {
      return pgm_read_word_near(PWM_CORRECTION + cieIndex);
    }


    void softStart() {
      softStartRunning = true;
    }

    void readEncoder(uint32_t change) {
      if (change != 0) {
        softStartRunning = false;
        if (rotationSpeed.inlineDelay(10)) {
          fineControl = true;
          cieIndexSetPoint +=  change * 4;
        }
        else {
          fineControl = false;
          cieIndexSetPoint +=  change * 12;
        }
        cieIndexSetPoint = constrain(cieIndexSetPoint, 50, CIE_ARRAY_SIZE - 1); // 50 == minimum dimmness
      }

    }

    void control(TLC5947 & tlc) {
      if (cieIndexSetPoint > 300 && softStartRunning) {
        softStartRunning = false;
      }
      if (softStartRunning && softStartOnRate.inlineDelay(5)) {
        cieIndexSetPoint++;
        cieIndex++;
      }

      int diff = abs(cieIndexSetPoint - cieIndex);
      if (diff != 0 && tlcUpdateDelay.inlineDelay(constrain(60 - diff*1.5, 5, 60))) {
        if (cieIndex < cieIndexSetPoint) cieIndex++;
        if (cieIndex > cieIndexSetPoint) cieIndex--;
      }

      if (pwmChange.valueChanged(cieIndex)) tlc.updateAll(cieToPwm());

    }
};


TLC5947 tlc(30000000, 10);
Encoder encoder(2, 3);
Delay lightsDebounce;
Delay lddDelay;
Delay relayOffDelay;
RisingEdge relayOffSignal;
RisingEdge lightsPb;
Toggle lightsEnabled;
RisingEdge softOnSignal;
Dimmer dimmer;

void setup() {
  // Set up SPI communication to TLC
  SPIClass::begin();

  // Initialize TLC latch pin
  pinMode(LATCH_PIN, OUTPUT);
  digitalWrite(LATCH_PIN, LOW);

  pinMode(LIGHTS_PUSHBUTTON_PIN, INPUT);
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);

  pinMode(LIGHTS_RELAY_PIN, OUTPUT);
  digitalWrite(LIGHTS_RELAY_PIN, LOW);
  tlc.begin();


  Serial.begin(9600);

}

void loop() {

  static bool enableLights = false;

  if (lightsPb.risingEdge(digitalRead(LIGHTS_PUSHBUTTON_PIN))) {
    if (lightsDebounce.inlineDelay(200)) {

      enableLights = lightsEnabled.toggle();
      if (enableLights) {
        digitalWrite(LIGHTS_RELAY_PIN, HIGH);
        lddDelay.start(20);
      } else {
        dimmer.off();
        relayOffDelay.start(50);
      }

    }
  }


  if (softOnSignal.risingEdge(lddDelay.done())) {
    dimmer.softStart();
  } else if (relayOffSignal.risingEdge(relayOffDelay.done())) {
    digitalWrite(LIGHTS_RELAY_PIN, LOW);
  } else if (enableLights &&
             lightsDebounce.done()) {
    dimmer.readEncoder(encoder.readAndReset());
  }


  dimmer.control(tlc);

}
