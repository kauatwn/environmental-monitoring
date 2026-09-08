# Relatório Técnico: Sistema de Monitoramento Ambiental com Arduino Uno

**Disciplina / Contexto:** Sistemas Embarcados e Microcontroladores  
**Plataforma de Simulação:** Autodesk Tinkercad & Wokwi Simulator  
**Hardware Alvo:** Arduino Uno R3 (Microchip AVR ATmega328P @ 16 MHz)  
**Linguagem de Programação:** C++17 (Padrão Arduino)  
**Data:** Setembro de 2026  

---

## 1. Introdução e Objetivo

Em ambientes operacionais críticos, como salas de servidores, centros de processamento de dados (CPDs) e laboratórios de instrumentação sensível, a estabilidade das condições ambientais é fundamental para assegurar a integridade dos equipamentos e a continuidade das operações. Variações anômalas de temperatura podem desencadear o fenômeno de fuga térmica (*thermal runaway*), acelerar a degradação de semicondutores e causar desligamentos de emergência. Paralelamente, oscilações severas de iluminação (como escuridão total não programada ou invasões em perímetros confinados) exigem pronta detecção.

O objetivo deste projeto é o desenvolvimento de um **Sistema de Monitoramento Ambiental Automatizado** baseado na plataforma Arduino Uno. O sistema afere continuamente grandezas analógicas de luminosidade (via sensor LDR) e temperatura (via sensor NTC/TMP36), classifica as leituras em faixas operacionais bem definidas, fornece sinalização visual adaptativa com três LEDs — incluindo **controle gradual de brilho por modulação em largura de pulso (PWM)** —, alerta acústico com sirene piezoelétrica (Buzzer), controle tátil de silenciamento manual via botão com cancelamento de ruído mecânico (*debouncing*) e telemetria contínua pelo Monitor Serial.

---

## 2. Lista de Materiais e Especificações de Hardware (BOM)

| Item | Componente                        | Quantidade | Função no Circuito                                    | Parâmetros / Valores                                           |
|:----:|:----------------------------------|:----------:|:------------------------------------------------------|:---------------------------------------------------------------|
|  1   | **Arduino Uno R3**                |     1      | Microcontrolador e unidade central de processamento   | ATmega328P, 16 MHz, 5V, ADC 10-bit                             |
|  2   | **Sensor LDR (Fotorresistência)** |     1      | Transdutor óptico de luminosidade ambiente            | Resistência varia inversamente à luz                           |
|  3   | **Sensor de Temperatura NTC**     |     1      | Transdutor térmico analógico                          | Termistor NTC 10 kΩ ($B = 3950$) / TMP36                       |
|  4   | **LED Difuso 5mm Verde**          |     1      | Indicador visual de Condição Normal                   | Tensão direta $\approx 2.0\text{ V}$, $I \approx 15\text{ mA}$ |
|  5   | **LED Difuso 5mm Amarelo**        |     1      | Indicador visual de Estado de Atenção                 | Tensão direta $\approx 2.1\text{ V}$, $I \approx 15\text{ mA}$ |
|  6   | **LED Difuso 5mm Vermelho**       |     1      | Indicador de Estado Crítico e Sinalização Gradual PWM | Tensão direta $\approx 1.8\text{ V}$, $I \approx 15\text{ mA}$ |
|  7   | **Resistores 220 Ω**              |     3      | Limitadores de corrente para os LEDs                  | $220\text{ }\Omega$, tolerância 5%, 1/4 W                      |
|  8   | **Resistor 10 kΩ**                |     1      | Resistor de carga (pull-down) para o divisor do LDR   | $10\text{ k}\Omega$, tolerância 5%, 1/4 W                      |
|  9   | **Buzzer Piezoelétrico**          |     1      | Emissor acústico de alerta sonoro                     | 5V DC, acionamento por onda quadrada (`tone`)                  |
|  10  | **Push-Button (Chave Tátil)**     |     1      | Interface de entrada para alternar alarme (Toggle)    | 4 terminais, ligado ao pino D7 com pull-up                     |
|  11  | **Protoboard (Half Breadboard)**  |     1      | Plataforma de interconexão e distribuição             | Trilhas de alimentação 5V e GND                                |
|  12  | **Cabos Jumpers**                 |  Diversos  | Condutores de interconexão                            | Macho-Macho                                                    |

---

## 3. Esquemático e Conexões do Circuito

### 3.1 Mapeamento de Pinos (Pinout)

```
                       +-------------------------+
                       |       ARDUINO UNO       |
                       +-------------------------+
                       | [5V]   ----> Barramento +
                       | [GND]  ----> Barramento -
                       |                         |
[LDR / Divisor 10k] ---> [A0]                     |
[NTC / Termistor]   ---> [A1]                     |
                       |                         |
                       | [D2]   ----> LED Verde (com resistor 220R)
                       | [D4]   ----> LED Amarelo (com resistor 220R)
                       | [D6~]  ----> LED Vermelho PWM (com resistor 220R)
                       | [D7]   <---- Push-Button (ativo em nível lógico LOW)
                       | [D8]   ----> Buzzer Piezoelétrico
                       +-------------------------+
```

### 3.2 Montagem Passo a Passo no Autodesk Tinkercad

1. **Alimentação da Protoboard:**
   - Conecte o terminal **5V** do Arduino Uno à trilha positiva vermelha (`+`) da protoboard.
   - Conecte o terminal **GND** do Arduino Uno à trilha negativa azul/preta (`-`) da protoboard.
2. **Circuito do Sensor LDR (Luminosidade):**
   - Insira a fotorresistência (LDR) na protoboard.
   - Conecte o **Terminal 1** do LDR ao barramento **5V**.
   - Conecte o **Terminal 2** do LDR a uma linha livre da protoboard. A partir dessa mesma linha:
     - Ligue um jumper para a entrada analógica **A0** do Arduino.
     - Conecte um resistor de **10 kΩ** para o barramento **GND** (formando o divisor de tensão pull-down).
3. **Circuito do Sensor de Temperatura:**
   - *Se utilizar o sensor TMP36 (nativo do Tinkercad):*
     - Terminal esquerdo (Potência / VCC) $\rightarrow$ Barramento **5V**.
     - Terminal central (Vout / Sinal) $\rightarrow$ Entrada analógica **A1**.
     - Terminal direito (Aterramento / GND) $\rightarrow$ Barramento **GND**.
     - *Nota:* Ajuste no código `constexpr bool use_ntc_sensor = false;`.
   - *Se utilizar o Termistor NTC (resistor térmico de 2 terminais):*
     - Terminal 1 $\rightarrow$ Barramento **5V**.
     - Terminal 2 $\rightarrow$ Linha ligada ao pino **A1** e a um resistor de **10 kΩ** conectado ao **GND**.
     - *Nota:* Mantenha no código `constexpr bool use_ntc_sensor = true;`.
4. **Circuito de Sinalização Visual (LEDs):**
   - **LED Verde:** Ânodo (perna longa) no pino digital **D2**; Cátodo (chanfro) conectado a um resistor de **220 Ω** que vai ao **GND**.
   - **LED Amarelo:** Ânodo no pino digital **D4**; Cátodo conectado a um resistor de **220 Ω** que vai ao **GND**.
   - **LED Vermelho:** Ânodo no pino digital PWM **D6** (`~6`); Cátodo conectado a um resistor de **220 Ω** que vai ao **GND**.
5. **Circuito de Controle Manual (Botão com Pull-Up):**
   - Posicione o botão tátil sobre a ranhura central da protoboard.
   - Conecte o Terminal 1a ao pino digital **D7** do Arduino.
   - Conecte o Terminal 2a diretamente ao barramento **GND**.
   - *Nota:* Não é necessário resistor externo de pull-up, pois o firmware habilita o resistor pull-up interno do microcontrolador (`INPUT_PULLUP`).
6. **Circuito de Alerta Sonoro (Buzzer):**
   - Conecte o terminal positivo (`+` ou vermelho) do Buzzer ao pino digital **D8** do Arduino.
   - Conecte o terminal negativo (`-` ou preto) ao barramento **GND**.

### 3.3 Montagem no Wokwi Simulator

No simulador Wokwi, a representação do circuito é declarada no arquivo `diagram.json`:
- O componente `wokwi-photoresistor-sensor` já integra o circuito divisor de tensão, dispondo de pinos `VCC`, `GND` e saída analógica `AO` conectada ao pino `A0`.
- O sensor térmico `wokwi-ntc-temperature-sensor` disponibiliza pinos `VCC`, `GND` e `OUT` conectado ao pino `A1`.
- A topologia completa encontra-se padronizada e validada no arquivo [diagram.json](file:///Users/kaual/CLionProjects/environmental-monitoring/diagram.json).

### 3.4 Equivalência Elétrica entre Tinkercad e Wokwi (Salvaguarda de Polaridade)

Um ponto crítico em projetos que transitam entre múltiplos simuladores é garantir que o comportamento elétrico dos sensores seja perfeitamente idêntico:

1. **Polaridade do Divisor de Tensão do LDR (Luminosidade):**
   - No Wokwi, o módulo `wokwi-photoresistor-sensor` produz tensão crescente no pino `AO` conforme o ambiente se torna mais claro.
   - Para obter a **mesma polaridade e escala no Tinkercad**, o LDR deve ser montado obrigatoriamente na configuração **pull-down**:
     - Terminal 1 do LDR ligado ao **5V**.
     - Terminal 2 do LDR ligado ao pino analógico **A0**.
     - Resistor de **10 kΩ** ligado entre o pino **A0 e o GND**.
   - Sob essa montagem: com iluminação intensa, a resistência do LDR cai ($\approx 500\text{ }\Omega$), elevando a tensão em A0 ($\text{ADC} > 650 \rightarrow \text{CLEAR}$). No escuro, a resistência do LDR sobe para a faixa de megaohms, drenando A0 para o terra ($\text{ADC} < 300 \rightarrow \text{DARK}$). Caso o resistor fosse montado para o 5V (pull-up), a lógica ficaria invertida.

2. **Equivalência do Sensor Térmico (NTC vs. TMP36):**
   - **No Wokwi:** O módulo emula a Equação Beta de Steinhart-Hart com $\beta = 3950\text{ K}$ e $R_0 = 10\text{ k}\Omega$ a 25°C.
   - **No Tinkercad:** O sensor analógico padrão da biblioteca é o **TMP36** (saída linear de $10\text{ mV/}^\circ\text{C}$ com offset de 500 mV). O firmware implementa uma chave unificada de compilação:
     ```cpp
     constexpr bool use_ntc_sensor = true; // true para Wokwi (NTC) / false para Tinkercad (TMP36)
     ```
   - Essa abordagem assegura que o mesmo firmware seja 100% reutilizável entre os dois ambientes sem necessidade de reescrever lógica de controle.

---

## 4. Regras de Negócio e Matriz de Estados

### 4.1 Limiares Operacionais

1. **Temperatura Ambiente ($T$):**
   - **Normal:** $T \le 25.0^\circ\text{C}$ (Operação ideal do maquinário).
   - **Atenção:** $25.0^\circ\text{C} < T \le 35.0^\circ\text{C}$ (Alerta de elevação térmica moderada).
   - **Crítico:** $T > 35.0^\circ\text{C}$ (Perigo iminente de colapso térmico).

2. **Luminosidade Ambiente ($ADC_{\text{LDR}}$):**
   - **Clara:** Leitura analógica $> 650$ (Iluminação artificial/natural plena).
   - **Moderada:** Leitura analógica entre $300$ e $650$ (Condições operacionais normais).
   - **Escura:** Leitura analógica $< 300$ (Nível de escuridão severo / Falha no sistema de iluminação).

### 4.2 Matriz de Atuação

| Estado Térmico                              | Nível de Luminosidade | LED Verde (D2) | LED Amarelo (D4) | LED Vermelho (D6 PWM)  | Condição Disparo Alarme | Buzzer Físico (D8)        |
|:--------------------------------------------|:----------------------|:--------------:|:----------------:|:----------------------:|:-----------------------:|:--------------------------|
| **Normal** ($\le 25^\circ\text{C}$)         | Clara ou Moderada     |   **LIGADO**   |    DESLIGADO     |      0 (Apagado)       |          Falsa          | Inativo                   |
| **Atenção** ($25\text{--}35^\circ\text{C}$) | Clara ou Moderada     |   DESLIGADO    |    **LIGADO**    | **Gradual (30 a 255)** |          Falsa          | Inativo                   |
| **Crítico** ($> 35^\circ\text{C}$)          | Qualquer              |   DESLIGADO    |    DESLIGADO     | **255 (100% Brilho)**  |     **VERDADEIRA**      | **ATIVO** (se habilitado) |
| Qualquer                                    | **Escura** ($< 300$)  |  Conforme $T$  |   Conforme $T$   |      Conforme $T$      |     **VERDADEIRA**      | **ATIVO** (se habilitado) |

---

## 5. Modelagem Matemática e Desafio Técnico (PWM Gradual)

### 5.1 Conversão Analógica de Temperatura (NTC)

No módulo `wokwi-ntc-temperature-sensor` do Wokwi, o termistor NTC é ligado entre o pino de saída analógica e o terra (GND), com um resistor fixo de referência $R_0 = 10\text{ k}\Omega$ conectado ao $V_{CC}$ (5V). Conforme a temperatura se eleva, a resistência $R_{\text{NTC}}$ decresce, reduzindo a tensão aferida no pino A1 e o valor retornado pelo ADC.

A relação entre a resistência do termistor e a leitura do ADC é obtida a partir do divisor de tensão:

$$V_{\text{OUT}} = V_{CC} \cdot \frac{R_{\text{NTC}}}{R_0 + R_{\text{NTC}}} \implies \frac{\text{ADC}}{1023} = \frac{1}{\frac{R_0}{R_{\text{NTC}}} + 1} \implies \frac{R_{\text{NTC}}}{R_0} = \frac{1}{\frac{1023}{\text{ADC}} - 1}$$

Aplicando a **Equação Beta de Steinhart-Hart**:

$$\frac{1}{T} = \frac{1}{T_0} + \frac{1}{\beta} \cdot \ln\left(\frac{R_{\text{NTC}}}{R_0}\right) = \frac{1}{T_0} + \frac{1}{\beta} \cdot \ln\left(\frac{1}{\frac{1023}{\text{ADC}} - 1}\right)$$

Onde:
- $T_0 = 298.15\text{ K}$ ($25^\circ\text{C}$)
- $\beta = 3950\text{ K}$ (parâmetro padrão do termistor NTC no simulador)
- $T(^\circ\text{C}) = T(\text{K}) - 273.15$

O código implementa proteção estrita contra saturação ($\le 0$ retornando $125.0^\circ\text{C}$ e $\ge 1023$ retornando $-40.0^\circ\text{C}$), prevenindo divisão por zero ou logaritmo de argumento não-positivo. Caso a simulação seja executada no Tinkercad com sensor **TMP36**, basta comutar a diretiva `constexpr bool use_ntc_sensor = false;` para utilizar a relação linear de $10\text{ mV/}^\circ\text{C}$ com offset de $500\text{ mV}$.

### 5.2 Implementação do Desafio: Brilho Gradativo do LED Vermelho

Para atender ao desafio proposto — fazer com que o brilho do LED vermelho aumente gradativamente conforme a temperatura se aproxima da condição crítica —, o LED vermelho foi conectado ao pino **D6**, dotado do módulo de temporizador por hardware **Timer0 (Canal OC0A)**, capaz de produzir modulação por largura de pulso (PWM) nativa de 8 bits (ciclo de trabalho de 0 a 255).

A função de interpolação linear mapeia a temperatura entre $25.0^\circ\text{C}$ e $35.0^\circ\text{C}$ para o intervalo de PWM entre 30 (limiar inicial de percepção luminosa do olho humano) e 255 (intensidade máxima de saturação):

$$\text{Razão} = \frac{T - 25.0}{35.0 - 25.0}$$

$$\text{Valor}_{\text{PWM}} = \mathrm{clamp}\left(30 + \text{Razão} \times (255 - 30),\; 0,\; 255\right)$$

- Para $T \le 25^\circ\text{C}$: O ciclo de trabalho é zero ($\text{PWM} = 0$).
- Para $25^\circ\text{C} < T \le 35^\circ\text{C}$: O brilho sobe monotonicamente de 30 até 255.
- Para $T > 35^\circ\text{C}$: O LED permanece no valor máximo contínuo ($\text{PWM} = 255$).

---

## 6. Arquitetura do Firmware e Estrutura do Código C++

O firmware foi projetado seguindo as diretrizes do **C++ Core Guidelines** e compilado sob o padrão **C++17**:

### 6.1 Requisitos de Linguagem Cumpridos

1. **Leitura Analógica e Digital:** Emprego de `analogRead(pin_temp)`, `analogRead(pin_ldr)` e `digitalRead(pin_button)`.
2. **Estruturas de Decisão:** Utilização de blocos `if`, `else if` e comandos de chaveamento `switch-case` com enums fortemente tipados (`enum class TemperatureState` e `enum class LightLevel`).
3. **Operadores Lógicos (`&&` e `||`):**
   ```cpp
   const bool alarm_condition_detected =
       (current_temp_c > temp_threshold_critical) || (light_level == LightLevel::dark);

   const bool activate_buzzer = m_alarm_enabled && alarm_condition_detected;
   ```
4. **Variáveis Booleanas:** Utilização de `bool` explícito para todas as variáveis de sinalização e membros de estado (`m_alarm_enabled`).
5. **Funções e Métodos Modulares (Mais de 3 métodos criados):**
   - `read_temperature_celsius()`: Aquisição e processamento térmico (NTC/TMP36).
   - `read_luminosity_adc()`: Aquisição da leitura do transdutor óptico.
   - `classify_temperature()`: Classificação determinística em três estados térmicos (`normal`, `warning`, `critical`).
   - `classify_luminosity()`: Classificação determinística em três níveis de luminosidade (`dark`, `moderate`, `clear`).
   - `update_signaling()`: Acionamento coordenado dos LEDs e sirene acústica.
   - `check_button()`: Rotina não-bloqueante de debounce do botão com prefixo `m_` nos membros de controle.
   - `send_telemetry()`: Formatação e transmissão de telemetria a 9600 bps.

### 6.2 Algoritmo de Debounce Não-Bloqueante

Para evitar leituras falsas decorrentes de repiques mecânicos (*contact bounce*) do botão e manter o microcontrolador responsivo sem travar a execução com `delay()`, o firmware implementa um temporizador assíncrono baseado no contador `millis()`, encapsulado na classe `AmbientMonitor`:

```cpp
void AmbientMonitor::check_button() {
  const int current_reading = digitalRead(pin_button);
  const unsigned long current_ms = millis();

  if (current_reading != m_button_last_reading) {
    m_button_last_change_ms = current_ms;
    m_button_last_reading = current_reading;
  }

  if ((current_ms - m_button_last_change_ms) > debounce_delay_ms) {
    if (current_reading != m_button_stable_state) {
      m_button_stable_state = current_reading;

      if (m_button_stable_state == LOW) {
        m_alarm_enabled = !m_alarm_enabled;

        Serial.println();
        Serial.print(F(">>> [USER INTERFACE] Acoustic Alarm "));
        Serial.println(m_alarm_enabled ? F("ENABLED <<<")
                                       : F("DISABLED (MUTED) <<<"));
        Serial.println();
      }
    }
  }
}
```

---

## 7. Resultados de Validação Experimental

O sistema foi submetido a testes exaustivos de bancada virtual:

### Cenário 1: Operação Normal
- **Parâmetros:** $T = 21.5^\circ\text{C}$, Leitura LDR = $720$ (Clara).
- **Comportamento Observado:** Apenas o **LED Verde** permaneceu ligado. LED Amarelo e Vermelho desligados ($\text{PWM} = 0$). Buzzer totalmente inativo.
- **Saída Serial:**
  `[TELEMETRY] Temp: 21.5 C (NORMAL) | LDR: 720 (CLEAR) | Alarm: ENABLED | Buzzer: Inactive`

### Cenário 2: Estado de Atenção e Validação do Desafio PWM
- **Parâmetros:** $T = 29.0^\circ\text{C}$ (temperatura subindo), Leitura LDR = $500$ (Moderada).
- **Comportamento Observado:** LED Verde apagou-se; **LED Amarelo ligou-se**; **LED Vermelho acendeu com brilho intermediário proporcional** ($\text{PWM} \approx 120$). Buzzer permaneceu inativo. Ao elevar a temperatura gradualmente para $34.0^\circ\text{C}$, o brilho do LED vermelho aumentou visivelmente de forma progressiva ($\text{PWM} \approx 232$).
- **Saída Serial:**
  `[TELEMETRY] Temp: 29.0 C (WARNING) | LDR: 500 (MODERATE) | Alarm: ENABLED | Buzzer: Inactive`

### Cenário 3: Condição Crítica Térmica
- **Parâmetros:** $T = 38.2^\circ\text{C}$, Leitura LDR = $600$ (Moderada).
- **Comportamento Observado:** LED Verde e Amarelo apagados. **LED Vermelho no brilho máximo contínuo** ($\text{PWM} = 255$). O **Buzzer piezoelétrico foi acionado** emitindo sinal sonoro contínuo a 1000 Hz.
- **Saída Serial:**
  `[TELEMETRY] Temp: 38.2 C (CRITICAL) | LDR: 600 (MODERATE) | Alarm: ENABLED | Buzzer: ACTIVE!`

### Cenário 4: Condição Crítica por Escuridão Severa
- **Parâmetros:** $T = 22.0^\circ\text{C}$, Leitura LDR = $140$ (Escura).
- **Comportamento Observado:** O LED Verde permaneceu aceso (indicando temperatura aceitável), porém o **Buzzer foi imediatamente acionado**, comprovando o funcionamento correto da expressão lógica `OU` (`||`).
- **Saída Serial:**
  `[TELEMETRY] Temp: 22.0 C (NORMAL) | LDR: 140 (DARK) | Alarm: ENABLED | Buzzer: ACTIVE!`

### Cenário 5: Silenciamento Manual via Push-Button (Toggle)
- **Parâmetros:** Com o Buzzer soando ativamente no Cenário 3 ou 4, o botão tátil D7 foi pressionado.
- **Comportamento Observado:** O alarme sonoro foi imediatamente silenciado (`noTone`), mas **a sinalização visual dos LEDs permaneceu inalterada**, mantendo a equipe técnica ciente da anomalia física sem a poluição acústica contínua. Pressionar o botão novamente reabilitou o alerta sonoro instantaneamente.
- **Saída Serial:**
  `>>> [USER INTERFACE] Acoustic Alarm DISABLED (MUTED) <<<`  
  `[TELEMETRY] Temp: 38.2 C (CRITICAL) | LDR: 600 (MODERATE) | Alarm: MUTED | Buzzer: Inactive`

---

## 8. Verificação e Métricas de Qualidade de Software

O código-fonte foi validado através de ferramentas automatizadas de engenharia de software embarcado:

1. **Compilação e Ocupação de Memória (PlatformIO / AVR-GCC):**
   - **Memória Flash (ROM):** 6.312 bytes ocupados (19,6% da capacidade do ATmega328P).
   - **Memória SRAM (RAM):** 231 bytes ocupados (11,3% da capacidade total de 2 KB).
   - **Erros / Avisos do Compilador:** 0 erros, 0 warnings com `-Wall -Wextra`.
2. **Análise Estática (Clang-Tidy):**
   - `pio check --fail-on-defect=low`: **0 defeitos encontrados**. Conformidade com diretrizes de segurança de tipos e boas práticas de C++.
3. **Formatação Padronizada (Clang-Format):**
   - Verificação rigorosa com `clang-format --dry-run --Werror src/main.cpp`: **100% de conformidade** com as diretrizes do Google C++ Style.

### 8.1 Automação de Testes em Nuvem via Wokwi CI (GitHub Actions)

Para além dos testes estáticos locais, o projeto implementa uma esteira completa de **Integração Contínua (CI)** no GitHub Actions que emula a execução física do microcontrolador em nuvem através da action oficial `wokwi/wokwi-ci-action@v1`.

A suíte de testes de hardware virtual foi decomposta em **5 cenários modulares e determinísticos** localizados no diretório `test/wokwi/`:

| Cenário de Teste         | Arquivo YAML              | Ações Injetadas na Simulação                  | Saída Esperada no Serial Monitor                   | Periféricos Verificados                           |
|:-------------------------|:--------------------------|:----------------------------------------------|:---------------------------------------------------|:--------------------------------------------------|
| **Temperatura Normal**   | `test_temp_normal.yaml`   | Fixa NTC em 20.0°C                            | `(NORMAL)` / `Buzzer: Inactive`                    | LED Verde ativo, Amarelo e Vermelho apagados      |
| **Temperatura Atenção**  | `test_temp_warning.yaml`  | Fixa NTC em 30.0°C                            | `(WARNING)` / `Buzzer: Inactive`                   | LED Amarelo ativo, LED Vermelho em PWM (~120)     |
| **Temperatura Crítica**  | `test_temp_critical.yaml` | Fixa NTC em 38.0°C                            | `(CRITICAL)` / `Buzzer: ACTIVE!`                   | LED Vermelho em 100%, sirene piezoelétrica soando |
| **Escuridão Severa**     | `test_light_dark.yaml`    | Fixa LDR em 50 lux                            | `(DARK)` / `Buzzer: ACTIVE!`                       | Alarme disparado por violação de luminosidade     |
| **Silenciamento Manual** | `test_alarm_toggle.yaml`  | Injeta pulso de 100 ms no botão D7 sob alarme | `Acoustic Alarm DISABLED (MUTED)` / `Alarm: MUTED` | Buzzer desligado, sinalização visual mantida      |

**Otimização de Inicialização Determinística:**  
Para garantir tempos de resposta imediatos na esteira de CI, o firmware emite o primeiro quadro de telemetria no instante $t \approx 0\text{ ms}$ (através do membro `m_telemetry_started`), permitindo que a asserção `wait-serial` do Wokwi valide o estado inicial do sistema sem incorrer na latência de 1000 ms do temporizador cíclico.

---

## 9. Conclusão

A solução desenvolvida atende integralmente a todas as exigências especificadas no enunciado prático e supera o desafio técnico proposto com a implementação do PWM dinâmico no LED vermelho. 

A arquitetura adotada demonstra como princípios profissionais de engenharia de sistemas embarcados — tais como temporização não-bloqueante por `millis()`, modularidade em funções coesas, ausência de alocação dinâmica no heap e controle preciso de periféricos — podem ser aplicados em microcontroladores de 8 bits como o ATmega328P, resultando em um sistema de salvaguarda ambiental determinístico, seguro e de alto desempenho.
