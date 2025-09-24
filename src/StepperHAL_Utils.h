#pragma once

#include <Arduino.h>
#include "PinNames.h"  // Assicurati che PA_0, PB_5 ecc. siano definiti
#include "stm32f4xx_hal.h"

// ─────────────────────────────────────────────
// Funzioni utility
// ─────────────────────────────────────────────
String pinName(uint8_t pin);
String timerName(TIM_TypeDef* timer);








