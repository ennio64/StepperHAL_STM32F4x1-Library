# StepperHAL_STM32F4x1

## Descrizione

StepperHAL è una libreria C++ progettata come strato di astrazione hardware (Hardware Abstraction Layer - HAL) per il controllo di motori stepper sulla famiglia di microcontrollori STM32F4x1.  
La libreria è stata sviluppata per semplificare la gestione del movimento, offrendo un'interfaccia intuitiva e configurabile che si integra con il framework Arduino e l'HAL di STMicroelectronics.

## Description

StepperHAL is a C++ library designed as a Hardware Abstraction Layer (HAL) for controlling stepper motors on STM32F4x1 microcontrollers.  
It simplifies motion control by providing an intuitive and configurable interface that integrates with both the Arduino framework and STMicroelectronics' HAL.

---

## ✨ Funzionalità Principali / Key Features

- **Controllo flessibile**  
  Gestione del movimento a velocità costante o con profili di accelerazione/decelerazione.  
  **Flexible control**: Supports constant-speed motion and acceleration/deceleration profiles.

- **Profili di movimento**  
  Supporto per profili trapezoidali e S-curve, abilitabili via configurazione.  
  **Motion profiles**: Trapezoidal and S-curve profiles for smooth and precise motion.

- **Configurazione estesa**  
  Parametri motore e pin gestiti centralmente (passi/giro, microstep, mm/giro).  
  **Extended configuration**: Centralized setup of motor parameters and pin mapping.

- **Debug e diagnostica**  
  Messaggi dettagliati via seriale per monitoraggio e troubleshooting.  
  **Debug & diagnostics**: Serial output for detailed runtime feedback and troubleshooting.

---

## ✨ Requisiti / Requirements

### 🖥️ Software

- **Framework Arduino per STM32**  
  Basato su Arduino_Core_STM32 (stm32duino).  
  **Arduino framework for STM32**: Built on Arduino_Core_STM32 (stm32duino).

- **IDE consigliato**  
  Arduino IDE o Visual Studio Code con PlatformIO.  
  **Recommended IDE**: Arduino IDE or VS Code with PlatformIO.

### 🔩 Hardware

- **Microcontrollore STM32F4x1**  
  Ottimizzato per questa famiglia, compatibile con modifiche minime su altri STM32.  
  **STM32F4x1 microcontroller**: Optimized for this family, adaptable to other STM32 variants.

- **Driver per motori passo-passo**  
  Es. A4988, DRV8825, collegati ai pin definiti in `StepperHAL_Config.h`.  
  **Stepper motor driver**: e.g., A4988, DRV8825, connected via `StepperHAL_Config.h`.

- **Motori passo-passo**  
  Supporta NEMA17, NEMA23 o simili.  
  **Stepper motors**: Supports NEMA17, NEMA23 and similar models.

- **Scheda di sviluppo**  
  Es. STM32 "Black Pill" o Nucleo.  
  **Development board**: e.g., STM32 "Black Pill" or Nucleo.

---

