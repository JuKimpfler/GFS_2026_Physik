#include "SimpleLink.h"

// ══════════════════════════════════════════════════════════════════════════
//  SimpleLinkSender  –  Arduino Nano RP2040 Connect
// ══════════════════════════════════════════════════════════════════════════

float myRound(float value, byte decimals = 0) {
  constexpr unsigned long int powTen[] {1, 10, 100, 1000, 10000, 100000};
  constexpr byte powTenMax {sizeof(powTen) / sizeof(powTen[0])};

  decimals = (decimals < 0 || decimals > (powTenMax - 2)) ? 0 : decimals;
  unsigned long int factor = powTen[decimals + 1];

  long int iValue {value * factor};  // Narrowing Warning!
  if (iValue != 0) { iValue = ((iValue > 0) ? iValue + 5 : iValue - 5) / 10; }
  return static_cast<float>(iValue) * 10 / factor;
}


SimpleLinkSender::SimpleLinkSender(uint8_t pin)
    : _pin(pin) {}

void SimpleLinkSender::begin() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
}

bool SimpleLinkSender::send(float value) {
    int vorzeichen = 0;
    if(value < 0){
        vorzeichen = 1;
    }

    if(abs(value) > 4){return false;}

    float value_round = myRound(abs(value),3);

    int bits[12];     

    uint16_t wert = (uint16_t)(value_round * 1000.0 + 0.5);

    for (int i = 0; i < 12; i++) {
        bits[i] = (wert >> i) & 0x01;
    }
    
    digitalWrite(_pin,HIGH);
    delay(1);
    digitalWrite(_pin,LOW);
    delayMicroseconds(500);

    digitalWrite(_pin,vorzeichen);
    delayMicroseconds(500);

    for ( int i = 0 ; i<12 ; i++){
        digitalWrite(_pin,bits[i]);
        delayMicroseconds(500);
    }

    digitalWrite(_pin,LOW);
}


// ══════════════════════════════════════════════════════════════════════════
//  SimpleLinkReceiver  –  Raspberry Pi Pico
// ══════════════════════════════════════════════════════════════════════════

SimpleLinkReceiver::SimpleLinkReceiver(uint8_t pin)
    : _pin(pin), _value(0.0f), _valid(false) {}

bool SimpleLinkReceiver::receive() {
    int bits[12]; 

    

    uint16_t rekonstruierteGanzzahl = 0;

    for (int i = 0; i < 12; i++) {
    if (bits[i] == 1) {
        rekonstruierteGanzzahl |= (1 << i);
    }
    }

    float ursprunglicheZahl = rekonstruierteGanzzahl / 1000.0;
}

void SimpleLinkReceiver::begin() {
    pinMode(_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pin), receive, RISING);
}

float SimpleLinkReceiver::getValue() const { return _value; }
bool  SimpleLinkReceiver::isValid()  const { return _valid; }
