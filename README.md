# Sistema de Monitoramento Ambiental

> **Projeto Prático de Sistemas Embarcados (Arduino Uno)**  
> _Monitoramento de temperatura e luminosidade em salas de equipamentos críticos utilizando Arduino Uno, com sinalização adaptativa (PWM) e motor de regras lógicas._

## Contexto Acadêmico

Este repositório contém a solução desenvolvida para a **atividade prática proposta pelo professor**. O objetivo do exercício foi criar um sistema de monitoramento ambiental utilizando o simulador **Tinkercad**, consolidando os seguintes conceitos obrigatórios da disciplina:

- Leitura de entradas analógicas (LDR e Sensor de Temperatura) e digitais (Botão).
- Uso prático de estruturas de decisão (`if/else`) e operadores lógicos (`&&` e `||`).
- Gerenciamento de estados através de variáveis booleanas nativas.
- Modularização do código-fonte em funções/classes organizadas para estruturar o programa.
- Exibição contínua de telemetria no Monitor Serial a 9600 bps.
- **Desafio Técnico:** Implementação de modulação por largura de pulso (PWM) usando `analogWrite()` para controle de brilho gradativo no LED vermelho conforme a temperatura se aproxima da condição crítica.

## 1. O Problema e o Contexto Operacional

Salas de equipamentos críticos (como servidores e centros de processamento de dados) exigem estabilidade ambiental constante. A elevação descontrolada da temperatura pode danificar o maquinário físico, enquanto a ausência inesperada de luz pode indicar falhas elétricas locais ou intrusão não autorizada em horários não operacionais.

Este projeto fornece uma camada de detecção automatizada, capaz de sinalizar visualmente a degradação térmica do ambiente através de três LEDs, controlar o brilho do LED vermelho via PWM durante a aproximação de risco e acionar um alarme acústico (Buzzer) sob condições críticas, permitindo intervenção humana rápida e guiada.

## 2. Matriz de Estados e Regras de Disparo

A classificação das grandezas lidas pelos sensores rege o comportamento dos atuadores visuais e sonoros de forma determinística:

| Condição Térmica                 | Condição de Luz  | LED Verde (D2) | LED Amarelo (D4) |   LED Vermelho (D6 PWM)    | Buzzer de Alerta (D8)     |
| :------------------------------- | :--------------- | :------------: | :--------------: | :------------------------: | :------------------------ |
| **Normal (Até 25°C)**            | Clara / Moderada |   **LIGADO**   |    Desligado     |       Desligado (0%)       | Inativo                   |
| **Atenção (25°C a 30°C)**        | Clara / Moderada |   Desligado    |    **LIGADO**    |       Desligado (0%)       | Inativo                   |
| **Aprox. Crítica (30°C a 35°C)** | Clara / Moderada |   Desligado    |    Desligado     | **Brilho Gradativo (PWM)** | Inativo                   |
| **Crítico (Acima de 35°C)**      | Qualquer         |   Desligado    |    Desligado     |  **Brilho Máximo (100%)**  | **ATIVO** (se habilitado) |
| Qualquer                         | **Escura**       | Conforme temp. |  Conforme temp.  |       Conforme temp.       | **ATIVO** (se habilitado) |

> **Controle Manual do Operador (Silenciador / Mute):** Através da entrada digital ligada a um botão (_pushbutton_ no pino D7), o usuário pode habilitar ou desabilitar o alarme sonoro a qualquer momento, sem desligar os LEDs de alerta. Cada pressionamento inverte o estado da variável booleana de controle do buzzer, permitindo silenciar o ruído enquanto a equipe atua na resolução do problema físico. O botão conta com filtro de repique mecânico (_debounce_) via software para evitar múltiplos acionamentos espúrios.

## 3. O Desafio Técnico: Sinalização Visual Dinâmica (PWM)

Para atender ao desafio prático de fornecer um alerta visual intuitivo antes que o sistema entre em colapso total, o brilho do LED vermelho aumenta gradativamente conforme a temperatura se aproxima da condição crítica.

O LED vermelho foi conectado ao pino digital **D6 (`~6`)**, provido de temporizador de hardware para geração de PWM nativo. A temperatura medida na faixa de aproximação crítica (entre 30°C e 35°C) dita o ciclo de trabalho (_duty cycle_) aplicado via `analogWrite()`:

- **Função de Interpolação Linear (`calculate_pwm_duty`):** Mapeia a temperatura atual para ajustar o valor do PWM de forma suave entre 30 e 255:
  $$\text{PWM} = 30 + \frac{T - 30}{35 - 30} \times (255 - 30)$$
- **Transição Límpida de Estados:** Em temperaturas de até 30°C, apenas o LED amarelo permanece aceso. Ao ultrapassar 30°C, o LED amarelo é apagado e o LED vermelho passa a responder progressivamente via PWM, eliminando o ruído de acendimento simultâneo de múltiplos LEDs de alerta.
- **Intensidade Máxima no Ponto Crítico:** Ao ultrapassar 35°C, o pino comuta para 100% de brilho contínuo (`PWM = 255`) e o alarme acústico é disparado.

## 4. Pinout e Conexões do Circuito (Hardware)

O circuito foi projetado e otimizado para o **Arduino Uno R3**, mapeando entradas e saídas de forma direta:

| Componente                        | Pino Arduino |    Tipo de I/O    | Função no Sistema                                               | Configuração Física / Circuito                    |
| :-------------------------------- | :----------: | :---------------: | :-------------------------------------------------------------- | :------------------------------------------------ |
| **Sensor de Luz (LDR)**           |     `A0`     | Entrada Analógica | Leitura de Luminosidade                                         | Divisor de tensão com resistor pull-down de 10 kΩ |
| **Sensor de Temp. (TMP36 / NTC)** |     `A1`     | Entrada Analógica | Leitura de Temperatura                                          | Conexão direta (TMP36) ou divisor 10 kΩ (NTC)     |
| **LED Verde**                     |     `D2`     |   Saída Digital   | Indicador de Condição Normal ($\le 25^\circ\text{C}$)           | Resistor limitador de 220 Ω no cátodo             |
| **LED Amarelo**                   |     `D4`     |   Saída Digital   | Indicador de Estado de Atenção ($25\text{--}30^\circ\text{C}$)  | Resistor limitador de 220 Ω no cátodo             |
| **LED Vermelho**                  |   `D6 (~)`   |     Saída PWM     | Indicador Crítico e PWM Gradual ($30\text{--}35^\circ\text{C}$) | Resistor limitador de 220 Ω no cátodo             |
| **Pushbutton**                    |     `D7`     |  Entrada Digital  | Botão Silenciador do Alarme (Mute)                              | Resistor interno pull-up (`INPUT_PULLUP`)         |
| **Buzzer Piezoelétrico**          |     `D8`     |   Saída Digital   | Emissão de Alerta Sonoro (1000 Hz)                              | Conexão direta ao pino D8 e ao GND comum          |

## 5. Portabilidade e Abstração de Hardware (Tinkercad vs. Wokwi)

Um dos diferenciais estruturais deste firmware é a sua arquitetura agnóstica em relação à montagem física. Através de constantes de configuração no início do código-fonte, o sistema adapta sua modelagem matemática para funcionar perfeitamente em diferentes simuladores, compensando as diferenças elétricas entre módulos integrados e componentes discretos.

Para alternar entre os ambientes de simulação, basta ajustar as seguintes `constexpr` booleanas no arquivo principal:

### Configuração para Wokwi (Padrão do Repositório)

O simulador Wokwi utiliza nativamente um módulo NTC (que exige a Equação do Parâmetro Beta) e um módulo LDR com divisor de tensão interno fixado em modo Pull-Up.

```cpp
// Mantém as flags como verdadeiras para a simulação no Wokwi
constexpr bool use_ntc_sensor = true;
constexpr bool ldr_pullup_mode = true;
```

### Configuração para Tinkercad

No Tinkercad, o hardware disponível requer o uso do sensor de temperatura linear TMP36 e a montagem manual do divisor de tensão do LDR na protoboard. A montagem adotada utiliza um resistor de 10 kΩ aterrado, caracterizando um circuito Pull-Down.

```cpp
// Altera as flags para falsas antes de iniciar a simulação no Tinkercad
constexpr bool use_ntc_sensor = false;
constexpr bool ldr_pullup_mode = false;
```

Essa camada de abstração demonstra como o software pode ser desacoplado das variações físicas de hardware, permitindo transições fluidas entre prototipagem virtual e testes em bancada real sem a necessidade de reescrever a lógica de controle.
