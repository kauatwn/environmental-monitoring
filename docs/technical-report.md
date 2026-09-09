# Relatório Técnico: Sistema de Monitoramento Ambiental

## 1. Visão Geral do Sistema

O projeto consiste em um firmware desenvolvido em C++ para o Arduino Uno, destinado ao monitoramento contínuo das condições de temperatura e luminosidade em uma sala de equipamentos críticos. O sistema processa dados de sensores analógicos em tempo real e aciona sinalizações visuais (LEDs) e sonoras (Buzzer) de acordo com regras de negócio e limiares de segurança pré-estabelecidos, além de transmitir relatórios de telemetria periodicamente via monitor Serial.

## 2. Arquitetura de Software e Hardware

O código foi estruturado de forma procedural e modular em C++, decomposto em funções especializadas com responsabilidade única, evitando complexidade desnecessária e classes redundantes, em plena conformidade com as diretrizes acadêmicas. Um grande diferencial deste projeto é a implementação da abstração de hardware por meio de constantes de configuração, permitindo flexibilidade na montagem física:

- **Sensor de Temperatura:** O sistema foi configurado para interpretar os dados do sensor linear TMP36 nativo do Tinkercad, desativando temporariamente a lógica do termistor NTC.
- **Sensor de Luminosidade:** A aquisição de luz é feita via LDR configurado em um circuito divisor de tensão do tipo _Pull-Down_, utilizando um resistor de **10 kΩ** aterrado, devidamente mapeado no código-fonte.

## 3. Sinalização Visual e Desafio PWM

A classificação da temperatura determina o estado do painel de LEDs:

- **Até 25°C (Operação Normal):** O LED verde permanece acionado de forma estática.
- **Entre 25°C e 30°C (Estado de Atenção):** O LED amarelo é acionado.
- **Entre 30°C e 35°C (Aproximação Crítica):** Para cumprir o desafio proposto, o sistema desliga o LED amarelo e passa a atuar no pino digital 6 utilizando modulação por largura de pulso (PWM). Uma função de interpolação matemática (`calculate_pwm_duty`) mapeia a temperatura atual para ajustar dinamicamente o _duty cycle_ do `analogWrite()`, resultando em um brilho gradativo do LED vermelho.
- **Acima de 35°C (Estado Crítico):** O LED vermelho opera em intensidade máxima.

## 4. Interface e Alarme Sonoro

O alarme acústico atua como um sistema de falha dupla, sendo disparado caso a temperatura ultrapasse 35°C ou o ambiente fique escuro (indicando falha de iluminação ou acesso anômalo).

- Foi adicionada uma interface de silenciador via botão _pushbutton_ conectado ao pino 7.
- Para evitar disparos indesejados, o botão utiliza a resistência de elevação interna (`INPUT_PULLUP`) e passa por um filtro de ruído mecânico (_debounce_) temporizado implementado inteiramente via software.
