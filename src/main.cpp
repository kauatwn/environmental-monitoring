/*
 * Sistema de Monitoramento Ambiental com Arduino Uno
 *
 * Descrição do Projeto:
 * Firmware para monitoramento contínuo de temperatura e luminosidade em uma sala de equipamentos críticos.
 * O sistema avalia as condições ambientais em tempo real, aciona sinalizações visuais e sonoras conforme
 * faixas pré-definidas e transmite dados de telemetria periodicamente via porta serial.
 *
 * Regras de Negócio e Comportamento Operacional:
 * 1. Temperatura:
 *    - Até 25.0 °C: Operação normal (LED verde ligado).
 *    - 25.1 °C a 30.0 °C: Estado de atenção (LED amarelo ligado).
 *    - 30.1 °C a 35.0 °C: Aproximação crítica com brilho gradativo do LED vermelho via PWM (30 a 255).
 *    - Acima de 35.0 °C: Estado crítico (LED vermelho em 100% de brilho).
 * 2. Luminosidade:
 *    - Classificada em Clara, Moderada ou Escura.
 *    - A condição escura indica falha de iluminação ou acesso irregular à sala, gerando condição de alarme.
 * 3. Alarme Sonoro (Buzzer):
 *    - Disparado se a temperatura for superior a 35.0 °C OU se o ambiente estiver no escuro.
 *    - O botão permite ao operador habilitar ou desabilitar o alarme sonoro (função silenciador).
 * 4. Compatibilidade:
 *    - Suporte transparente aos simuladores Wokwi e Tinkercad, com opções para NTC ou TMP36 e
 *      circuitos divisores de tensão em modo Pull-Up ou Pull-Down para o LDR.
 */

#include <Arduino.h>
#include <math.h>

namespace {
// Mapeamento de pinos do hardware
constexpr uint8_t pin_ldr = A0;        // Entrada analógica: sensor de luz (LDR)
constexpr uint8_t pin_temp = A1;       // Entrada analógica: sensor de temperatura (NTC ou TMP36)
constexpr uint8_t pin_button = 7;      // Entrada digital: botão de controle com pull-up interno
constexpr uint8_t pin_led_green = 2;   // Saída digital: LED verde (condição normal)
constexpr uint8_t pin_led_yellow = 4;  // Saída digital: LED amarelo (estado de atenção)
constexpr uint8_t pin_led_red = 6;     // Saída PWM: LED vermelho (aproximação crítica e alerta)
constexpr uint8_t pin_buzzer = 8;      // Saída digital: buzzer piezoelétrico

// Configurações dos sensores e compatibilidade de hardware:
// - true:  Termistor NTC (padrão no Wokwi e disponível no Tinkercad).
// - false: TMP36 analógico linear (comum em montagens tradicionais no Tinkercad).
constexpr bool use_ntc_sensor = true;

// Parâmetros do termistor NTC para a Equação de Steinhart-Hart / Parâmetro Beta (B3950):
// R0 = 10 kOhm a 25 °C (298.15 K)
constexpr float ntc_beta = 3950.0F;
constexpr float ntc_t0_kelvin = 298.15F;
constexpr float absolute_zero_celsius = 273.15F;

// Configuração do circuito divisor de tensão do sensor LDR:
// - true:  Modo Pull-Up (módulo Wokwi e montagem 5V -> Resistor 10k -> A0 -> LDR -> GND).
//          No escuro, a resistência do LDR aumenta e a tensão em A0 sobe (ADC alto).
// - false: Modo Pull-Down (montagem 5V -> LDR -> A0 -> Resistor 10k -> GND no Tinkercad).
//          No escuro, a resistência do LDR aumenta e a tensão em A0 cai (ADC baixo).
constexpr bool ldr_pullup_mode = true;

// Limiares de luminosidade para o modo Pull-Up (Wokwi / Tinkercad Pull-Up)
constexpr int light_threshold_dark_pullup = 700;   // Leitura analógica > 700 indica ambiente escuro
constexpr int light_threshold_clear_pullup = 300;  // Leitura analógica < 300 indica ambiente claro

// Limiares de luminosidade para o modo Pull-Down (Tinkercad Pull-Down)
constexpr int light_threshold_dark_pulldown = 300;   // Leitura analógica < 300 indica ambiente escuro
constexpr int light_threshold_clear_pulldown = 700;  // Leitura analógica > 700 indica ambiente claro

// Limiares operacionais de temperatura
constexpr float temp_threshold_normal = 25.0F;    // Normal: <= 25.0 °C
constexpr float temp_threshold_approach = 30.0F;  // Atenção: 25.0 °C a 30.0 °C
constexpr float temp_threshold_critical = 35.0F;  // Aproximação e crítico: > 30.0 °C e > 35.0 °C

// Limites do PWM para o LED vermelho durante a aproximação crítica (30.0 °C a 35.0 °C)
constexpr int pwm_min_duty = 30;   // Valor mínimo para garantir condução e visibilidade do LED
constexpr int pwm_max_duty = 255;  // Ciclo de trabalho máximo (100% de brilho)

// Temporizações e parâmetros acústicos
constexpr unsigned long telemetry_interval_ms = 1000;  // Intervalo de transmissão serial (1 segundo)
constexpr unsigned long debounce_delay_ms = 50;        // Janela de estabilização do botão (50 ms)
constexpr unsigned int buzzer_frequency_hz = 1000;     // Frequência do som de alerta no buzzer (Hz)

// Variáveis de estado global do sistema
bool buzzer_enabled = true;
int last_button_reading = HIGH;
int stable_button_state = HIGH;
unsigned long last_button_change_ms = 0;

unsigned long last_telemetry_ms = 0;
bool telemetry_started = false;

// Leitura da temperatura e conversão para Celsius conforme o sensor configurado
float read_temperature_celsius() {
  const int raw_adc = analogRead(pin_temp);

  // Conversão para o sensor linear TMP36:
  // Tensão (V) = ADC * (5.0 / 1024.0)
  // Temperatura (°C) = (Tensão - 0.5) * 100.0
  if (!use_ntc_sensor) {
    constexpr float adc_to_voltage = 5.0F / 1024.0F;
    const float voltage_v = static_cast<float>(raw_adc) * adc_to_voltage;
    return (voltage_v - 0.5F) * 100.0F;
  }

  // Tratamento de limites físicos do termistor NTC (evita saturação e divisões por zero)
  if (raw_adc <= 0) {
    return 125.0F;
  }
  if (raw_adc >= 1023) {
    return -40.0F;
  }

  // Conversão via Equação do Parâmetro Beta (Steinhart-Hart simplificada)
  const float raw_ratio = 1023.0F / static_cast<float>(raw_adc);
  const float adc_ratio = 1.0F / (raw_ratio - 1.0F);
  if (adc_ratio <= 0.0F) {
    return 125.0F;
  }

  const float log_ratio = logf(adc_ratio);
  const float term_beta = log_ratio / ntc_beta;
  constexpr float term_t0 = 1.0F / ntc_t0_kelvin;
  const float kelvin = 1.0F / (term_beta + term_t0);
  return kelvin - absolute_zero_celsius;
}

// Leitura do canal analógico do divisor de tensão do sensor LDR
int read_luminosity_adc() { return analogRead(pin_ldr); }

// Avalia se a leitura de luminosidade caracteriza ambiente escuro conforme o divisor adotado
bool is_dark_condition(const int raw_adc) {
  if (ldr_pullup_mode) {
    return raw_adc > light_threshold_dark_pullup;
  }
  return raw_adc < light_threshold_dark_pulldown;
}

// Classifica o nível de luminosidade em texto
const __FlashStringHelper* get_luminosity_label(const int raw_adc) {
  if (ldr_pullup_mode) {
    if (raw_adc > light_threshold_dark_pullup) {
      return F("ESCURA");
    }
    if (raw_adc < light_threshold_clear_pullup) {
      return F("CLARA");
    }
    return F("MODERADA");
  }

  if (raw_adc < light_threshold_dark_pulldown) {
    return F("ESCURA");
  }
  if (raw_adc > light_threshold_clear_pulldown) {
    return F("CLARA");
  }
  return F("MODERADA");
}

// Interpolação linear do brilho do LED vermelho via PWM na faixa de aproximação crítica (30 °C a 35 °C)
int calculate_pwm_duty(const float temp_c) {
  const float ratio = (temp_c - temp_threshold_approach) / (temp_threshold_critical - temp_threshold_approach);
  constexpr auto pwm_range = static_cast<float>(pwm_max_duty - pwm_min_duty);
  const float calculated = static_cast<float>(pwm_min_duty) + ratio * pwm_range;
  const int duty = static_cast<int>(calculated);

  if (duty < 0) {
    return 0;
  }
  if (duty > 255) {
    return 255;
  }
  return duty;
}

// Atualização das saídas digitais e PWM dos LEDs de sinalização visual
void update_visual_signaling(const float temp_c) {
  // Faixa normal (<= 25.0 °C): apenas LED verde aceso
  if (temp_c <= temp_threshold_normal) {
    digitalWrite(pin_led_green, HIGH);
    digitalWrite(pin_led_yellow, LOW);
    analogWrite(pin_led_red, 0);
    return;
  }

  // Faixa de atenção intermediária (25.0 °C a 30.0 °C): apenas LED amarelo aceso
  if (temp_c <= temp_threshold_approach) {
    digitalWrite(pin_led_green, LOW);
    digitalWrite(pin_led_yellow, HIGH);
    analogWrite(pin_led_red, 0);
    return;
  }

  // Faixa de aproximação crítica (30.0 °C a 35.0 °C): LED vermelho em PWM gradual
  if (temp_c <= temp_threshold_critical) {
    digitalWrite(pin_led_green, LOW);
    digitalWrite(pin_led_yellow, LOW);
    analogWrite(pin_led_red, calculate_pwm_duty(temp_c));
    return;
  }

  // Estado crítico (> 35.0 °C): LED vermelho em intensidade máxima (100%)
  digitalWrite(pin_led_green, LOW);
  digitalWrite(pin_led_yellow, LOW);
  analogWrite(pin_led_red, pwm_max_duty);
}

// Aciona o buzzer piezoelétrico a 1000 Hz ou silencia a saída
void control_acoustic_alarm(const bool activate) {
  if (activate) {
    tone(pin_buzzer, buzzer_frequency_hz);
    return;
  }
  noTone(pin_buzzer);
}

// Filtro de repique mecânico (debounce de 50 ms) e alternância do silenciador do alarme sonoro
void process_silence_button() {
  const unsigned long current_ms = millis();
  const int current_reading = digitalRead(pin_button);

  if (current_reading != last_button_reading) {
    last_button_change_ms = current_ms;
    last_button_reading = current_reading;
  }

  if (current_ms - last_button_change_ms <= debounce_delay_ms) {
    return;
  }

  if (current_reading == stable_button_state) {
    return;
  }

  stable_button_state = current_reading;

  // Borda de descida (botão pressionado no pino com pull-up)
  if (stable_button_state == LOW) {
    buzzer_enabled = !buzzer_enabled;
    Serial.println();
    Serial.print(F(">>> [INTERFACE DO USUARIO] Alarme Sonoro "));
    Serial.println(buzzer_enabled ? F("HABILITADO <<<") : F("DESABILITADO (SILENCIADO) <<<"));
    Serial.println();
  }
}

// Retorna o rótulo textual da faixa de temperatura para a telemetria serial
const __FlashStringHelper* get_temperature_label(const float temp_c) {
  if (temp_c <= temp_threshold_normal) {
    return F("NORMAL");
  }
  if (temp_c <= temp_threshold_approach) {
    return F("ATENCAO");
  }
  if (temp_c <= temp_threshold_critical) {
    return F("ATENCAO - APROX. CRITICA PWM");
  }
  return F("CRITICO");
}

// Transmissão periódica das informações pela porta serial
void transmit_telemetry(const float temp_c, const bool buzzer_active, const int raw_ldr) {
  Serial.print(F("[TELEMETRIA] Temp: "));
  Serial.print(temp_c, 1);
  Serial.print(F(" C ("));
  Serial.print(get_temperature_label(temp_c));

  Serial.print(F(") | LDR: "));
  Serial.print(raw_ldr);
  Serial.print(F(" ("));
  Serial.print(get_luminosity_label(raw_ldr));

  Serial.print(F(") | Alarme: "));
  Serial.print(buzzer_enabled ? F("HABILITADO") : F("SILENCIADO"));

  Serial.print(F(" | Buzzer: "));
  Serial.println(buzzer_active ? F("ATIVO!") : F("Inativo"));
}
}  // namespace

void setup() {
  Serial.begin(9600);
  Serial.println(F("=================================================="));
  Serial.println(F(" SISTEMA DE MONITORAMENTO AMBIENTAL - ARDUINO UNO"));
  Serial.println(F(" Status: Inicializado com Sucesso                "));
  Serial.println(F("=================================================="));

  pinMode(pin_button, INPUT_PULLUP);
  pinMode(pin_led_green, OUTPUT);
  pinMode(pin_led_yellow, OUTPUT);
  pinMode(pin_led_red, OUTPUT);
  pinMode(pin_buzzer, OUTPUT);

  digitalWrite(pin_led_green, LOW);
  digitalWrite(pin_led_yellow, LOW);
  analogWrite(pin_led_red, 0);
  noTone(pin_buzzer);
}

void loop() {
  const unsigned long current_ms = millis();

  process_silence_button();

  if (!telemetry_started || current_ms - last_telemetry_ms >= telemetry_interval_ms) {
    telemetry_started = true;
    last_telemetry_ms = current_ms;

    const float temp_c = read_temperature_celsius();
    const int raw_ldr = read_luminosity_adc();
    const bool is_dark = is_dark_condition(raw_ldr);

    update_visual_signaling(temp_c);

    const bool alarm_condition = (temp_c > temp_threshold_critical) || is_dark;
    const bool buzzer_active = alarm_condition && buzzer_enabled;
    control_acoustic_alarm(buzzer_active);

    transmit_telemetry(temp_c, buzzer_active, raw_ldr);
  }
}
