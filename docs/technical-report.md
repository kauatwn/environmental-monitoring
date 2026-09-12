# Relatório Técnico: Sistema de Monitoramento Ambiental

## 1. Visão Geral do Sistema

O projeto consiste em um firmware desenvolvido em C++ para o Arduino Uno, destinado ao monitoramento contínuo das condições de temperatura e luminosidade em uma sala de equipamentos críticos. O sistema processa dados de sensores analógicos em tempo real e aciona sinalizações visuais (LEDs) e sonoras (Buzzer) de acordo com regras de negócio e limiares de segurança pré-estabelecidos, além de transmitir relatórios de telemetria periodicamente via monitor Serial.

## 2. Arquitetura de Software e Modelagem do Circuito

O código foi estruturado de forma procedural e modular em C++, decomposto em funções especializadas com responsabilidade única, evitando complexidade desnecessária e classes redundantes, em plena conformidade com as diretrizes acadêmicas.

Um dos grandes diferenciais deste projeto é a implementação da abstração de hardware por meio de **Compilação Condicional** (`#define SIMULATOR_TINKERCAD` / `#define SIMULATOR_WOKWI`), permitindo que a mesma base de código atenda às duas modelagens de circuito:

- **Modelagem Tinkercad (Padrão da Entrega - Componentes Discretos):**
  - **Sensor de Temperatura:** Sensor analógico linear **TMP36** (5V, A1, GND), com resposta linear de 10 mV/°C e offset de 500 mV a 0 °C.
  - **Sensor de Luminosidade:** Fotorresistor LDR montado em divisor de tensão **Pull-Down** com resistor de **10 kΩ** aterrado. No escuro, a tensão no pino A0 diminui (ADC < 300).
- **Modelagem Wokwi (Simulação Local - Módulos Integrados):**
  - **Sensor de Temperatura:** Módulo com termistor **NTC** B3950 (VCC, GND, OUT em A1), cuja temperatura é calculada no firmware pela Equação Beta de Steinhart-Hart.
  - **Sensor de Luminosidade:** Módulo LDR com malha interna **Pull-Up** (VCC, GND, AO em A0). No escuro, a tensão no pino A0 aumenta (ADC > 700).

## 3. Sinalização Visual e Desafio PWM

A classificação da temperatura determina o estado do painel de LEDs:

- **Até 25°C (Operação Normal):** O LED verde permanece acionado de forma estática.
- **Entre 25°C e 30°C (Estado de Atenção):** O LED amarelo é acionado.
- **Entre 30°C e 35°C (Aproximação Crítica):** Para cumprir o desafio proposto, o sistema desliga o LED amarelo e passa a atuar no pino digital 6 utilizando modulação por largura de pulso (PWM). Uma função de interpolação matemática (`calculate_pwm_duty`) mapeia a temperatura atual para ajustar dinamicamente o _duty cycle_ do `analogWrite()` entre 30 e 255, resultando em um brilho gradativo do LED vermelho.
- **Acima de 35°C (Estado Crítico):** O LED vermelho opera em intensidade máxima (100% PWM).

## 4. Interface e Alarme Sonoro

O alarme acústico atua como um sistema de falha dupla, sendo disparado caso a temperatura ultrapasse 35°C ou o ambiente fique escuro (indicando falha de iluminação ou acesso anômalo).

- Foi adicionada uma interface de silenciador via botão _pushbutton_ conectado ao pino 7.
- Para evitar disparos indesejados, o botão utiliza a resistência de elevação interna (`INPUT_PULLUP`) e passa por um filtro de ruído mecânico (_debounce_) temporizado implementado inteiramente via software.
