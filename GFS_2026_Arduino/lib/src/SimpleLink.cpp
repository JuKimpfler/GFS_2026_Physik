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


bool SimpleLinkSender::send(float value) {
    int vorzeichen = 0;
    if(value < 0){
        vorzeichen = 1;
    }

    if(abs(value) > 4){return false;}

    float value_round = myRound(abs(value),3);

    int bits[12] = {0,0,0,0,0,0,0,0,0,0,0,0};     

    uint16_t wert = (uint16_t)(value_round * 1000.0 + 0.5);

    for (int i = 0; i < 12; i++) {
        bits[i] = (wert >> i) & 0x01;
    }
    
    digitalWrite(_pin,HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds(15*Faktor);
    digitalWrite(_pin,LOW);
    digitalWrite(LED_BUILTIN, LOW);
    delayMicroseconds(10*Faktor);

    digitalWrite(_pin,vorzeichen);
    digitalWrite(LED_BUILTIN, vorzeichen);
    delayMicroseconds(10*Faktor);

    for ( int i = 0 ; i<12 ; i++){
        digitalWrite(_pin,bits[i]);
        digitalWrite(LED_BUILTIN, bits[i]);
        delayMicroseconds(10*Faktor);
    }

    digitalWrite(_pin,LOW);
    digitalWrite(LED_BUILTIN, LOW);
    return true;
}

