/**
 * @file main.cpp
 * @brief Sistema de Monitoramento Ambiental com Arduino Uno.
 * @author Aluno de Engenharia / Sistemas Embarcados
 * @date 2026-09-08
 *
 * Descrição Geral:
 * Monitoramento contínuo de temperatura (sensor NTC / TMP36) e luminosidade
 * (sensor LDR) em uma sala de equipamentos críticos utilizando Arduino Uno.
 *
 * Requisitos Atendidos:
 * - Entradas analógicas (A0 para LDR, A1 para NTC/TMP36) e digital (D7 para
 * botão).
 * - Saídas digitais/PWM (D2 LED Verde, D4 LED Amarelo, D6 LED Vermelho PWM, D8
 * Buzzer).
 * - Estruturas de decisão if / else if / else e operadores lógicos (&& e ||).
 * - Variáveis booleanas para controle de estado do alarme e disparo de
 * atuadores.
 * - Funções modulares para aquisição, classificação, atuação, interface e
 * telemetria.
 * - Desafio Técnico: Brilho gradativo do LED vermelho via PWM (analogWrite)
 * conforme a temperatura se aproxima da condição crítica (>30°C até 35°C), sem
 * sobreposição com o LED amarelo.
 * - Compatibilidade total Wokwi e Tinkercad para sensores LDR e térmicos.
 */

#include <Arduino.h>
#include <math.h>

namespace {
// ============================================================================
// 1. MAPEAMENTO DE PINOS DO HARDWARE
// ============================================================================
constexpr uint8_t pin_ldr = A0;  // Entrada analógica: Sensor de luz (LDR)
constexpr uint8_t pin_temp =
    A1;  // Entrada analógica: Sensor térmico (NTC / TMP36)
constexpr uint8_t pin_button =
    7;  // Entrada digital: Botão de toggle com pull-up interno
constexpr uint8_t pin_led_green =
    2;  // Saída digital: LED Verde (Condição Normal)
constexpr uint8_t pin_led_yellow =
    4;  // Saída digital: LED Amarelo (Estado de Atenção)
constexpr uint8_t pin_led_red =
    6;  // Saída analógica/PWM: LED Vermelho (Aproximação / Crítico)
constexpr uint8_t pin_buzzer =
    8;  // Saída digital/frequência: Buzzer piezoelétrico

// ============================================================================
// 2. CONFIGURAÇÕES DOS SENSORES E COMPATIBILIDADE SIMULADORES
// ============================================================================
// true:  Termistor NTC com Equação Beta B3950 (Padrão no Wokwi e Termistor
// Tinkercad) false: Sensor analógico linear TMP36 (Padrão nativo do Tinkercad:
// 10 mV/°C, offset 500 mV)
constexpr bool use_ntc_sensor = true;

constexpr float ntc_beta = 3950.0F;
constexpr float ntc_t0_kelvin = 298.15F;
constexpr float absolute_zero_celsius = 273.15F;

// Configuração do divisor de tensão do sensor LDR:
// true:  Modo Pull-Up (Padrão do módulo Wokwi "wokwi-photoresistor-sensor" e
// montagem
//        5V -> Resistor 10k -> A0 -> LDR -> GND no Tinkercad).
//        Escuro produz resistência alta -> Tensão sobe -> ADC alto.
// false: Modo Pull-Down (Montagem 5V -> LDR -> A0 -> Resistor 10k -> GND no
// Tinkercad).
//        Escuro produz resistência alta -> Tensão cai -> ADC baixo.
constexpr bool ldr_pullup_mode = true;

// Limiares de luminosidade para o modo Pull-Up (Wokwi / Tinkercad Pull-Up)
constexpr int light_threshold_dark_pullup =
    700;  // ADC > 700 indica ambiente Escuro
constexpr int light_threshold_clear_pullup =
    300;  // ADC < 300 indica ambiente Claro

// Limiares de luminosidade para o modo Pull-Down (Tinkercad Pull-Down
// tradicional)
constexpr int light_threshold_dark_pulldown =
    300;  // ADC < 300 indica ambiente Escuro
constexpr int light_threshold_clear_pulldown =
    700;  // ADC > 700 indica ambiente Claro

// ============================================================================
// 3. LIMIARES OPERACIONAIS DE TEMPERATURA E DESAFIO PWM
// ============================================================================
constexpr float temp_threshold_normal =
    25.0F;  // Até 25.0°C: Normal (LED Verde)
constexpr float temp_threshold_approach =
    30.0F;  // 25.0°C a 30.0°C: Atenção (LED Amarelo)
constexpr float temp_threshold_critical =
    35.0F;  // 30.0°C a 35.0°C: Desafio PWM (LED Vermelho gradual)
// Acima de 35.0°C: Crítico (LED Vermelho 100% + Buzzer)

constexpr int pwm_min_duty =
    30;  // Ciclo de trabalho mínimo para visibilidade do LED
constexpr int pwm_max_duty = 255;  // Ciclo de trabalho máximo (100%)

// ============================================================================
// 4. TEMPORIZAÇÕES E PARÂMETROS ACÚSTICOS
// ============================================================================
constexpr unsigned long telemetry_interval_ms =
    1000;  // Intervalo do Monitor Serial (1s)
constexpr unsigned long debounce_delay_ms =
    50;  // Janela de estabilização do botão
constexpr unsigned int buzzer_frequency_hz =
    1000;  // Frequência do tom de alerta (Hz)

// ============================================================================
// 5. ENUMERAÇÕES FORTEMENTE TIPADAS PARA ESTADOS OPERACIONAIS
// ============================================================================
enum class TemperatureState : uint8_t {
  normal,    // Temperatura <= 25°C
  warning,   // 25°C < Temperatura <= 35°C
  critical,  // Temperatura > 35°C
};

enum class LightLevel : uint8_t {
  dark,      // Condição Escura (gera condição de alarme)
  moderate,  // Condição Moderada
  clear,     // Condição Clara
};

// ============================================================================
// 6. DECLARAÇÃO DA CLASSE DE MONITORAMENTO AMBIENTAL
// ============================================================================
class AmbientMonitor {
 public:
  explicit AmbientMonitor() = default;

  static void init();

  void update();

  // Funções modulares de processamento criadas pelo aluno
  static float read_temperature_celsius();

  static int read_luminosity_adc();

  static TemperatureState classify_temperature(float temp_c) noexcept;

  static LightLevel classify_luminosity(int raw_adc) noexcept;

  static void update_signaling(TemperatureState temp_state, float temp_c,
                               bool activate_buzzer);

  void check_button();

  void send_telemetry(float temp_c, TemperatureState temp_state, int raw_ldr,
                      LightLevel light_lvl, bool buzzer_active) const;

  [[nodiscard]] bool is_alarm_enabled() const noexcept {
    return m_alarm_enabled;
  }

 private:
  bool m_alarm_enabled{true};
  bool m_telemetry_started{false};
  unsigned long m_last_telemetry_ms{0};
  int m_button_last_reading{HIGH};
  int m_button_stable_state{HIGH};
  unsigned long m_button_last_change_ms{0};
};

// ============================================================================
// 7. INICIALIZAÇÃO DO HARDWARE (SETUP)
// ============================================================================
void AmbientMonitor::init() {
  Serial.begin(9600);

  // Configuração dos sentidos dos pinos
  pinMode(pin_led_green, OUTPUT);
  pinMode(pin_led_yellow, OUTPUT);
  pinMode(pin_led_red, OUTPUT);
  pinMode(pin_buzzer, OUTPUT);
  pinMode(pin_button, INPUT_PULLUP);

  // Estado seguro inicial: todos os atuadores desligados
  digitalWrite(pin_led_green, LOW);
  digitalWrite(pin_led_yellow, LOW);
  analogWrite(pin_led_red, 0);
  noTone(pin_buzzer);

  Serial.println(F("=================================================="));
  Serial.println(F(" SISTEMA DE MONITORAMENTO AMBIENTAL - ARDUINO UNO"));
  Serial.println(F(" Status: Inicializado com Sucesso                "));
  Serial.println(F("=================================================="));
}

// ============================================================================
// 8. CICLO PRINCIPAL DE PROCESSAMENTO (UPDATE)
// ============================================================================
void AmbientMonitor::update() {
  // 1. Processa a leitura do botão e controle de alternância do alarme sonoro
  check_button();

  // 2. Aquisição dos valores dos sensores analógicos
  const float current_temp_c = read_temperature_celsius();
  const int current_light_adc = read_luminosity_adc();

  // 3. Classificação dos estados operacionais
  const TemperatureState temp_state = classify_temperature(current_temp_c);
  const LightLevel light_level = classify_luminosity(current_light_adc);

  // 4. Lógica booleana de disparo: aciona quando temperatura > 35°C OU
  // luminosidade escura
  const bool alarm_condition_detected =
      (current_temp_c > temp_threshold_critical) ||
      (light_level == LightLevel::dark);

  // 5. O buzzer só toca se o alarme sonoro estiver habilitado E houver condição
  // de disparo
  const bool activate_buzzer = is_alarm_enabled() && alarm_condition_detected;

  // 6. Atualização da sinalização visual e acústica
  update_signaling(temp_state, current_temp_c, activate_buzzer);

  // 7. Transmissão periódica de telemetria no Monitor Serial
  const unsigned long current_ms = millis();
  if (!m_telemetry_started ||
      current_ms - m_last_telemetry_ms >= telemetry_interval_ms) {
    m_telemetry_started = true;
    m_last_telemetry_ms = current_ms;
    send_telemetry(current_temp_c, temp_state, current_light_adc, light_level,
                   activate_buzzer);
  }
}

// ============================================================================
// 9. FUNÇÃO 1: LEITURA E CONVERSÃO DE TEMPERATURA (°C)
// ============================================================================
float AmbientMonitor::read_temperature_celsius() {
  const int raw_adc = analogRead(pin_temp);

  if (use_ntc_sensor) {
    // Salvaguardas contra saturação do ADC para evitar divisão por zero
    if (raw_adc <= 0) {
      return 125.0F;
    }
    if (raw_adc >= 1023) {
      return -40.0F;
    }

    // Equação Beta do Termistor NTC (B3950)
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

  // Conversão analógica para o sensor linear TMP36 (Tinkercad padrão)
  constexpr float adc_to_voltage = 5.0F / 1024.0F;
  const float voltage_v = static_cast<float>(raw_adc) * adc_to_voltage;
  return (voltage_v - 0.5F) * 100.0F;
}

// ============================================================================
// 10. FUNÇÃO 2: LEITURA ANALÓGICA DO SENSOR LDR
// ============================================================================
int AmbientMonitor::read_luminosity_adc() { return analogRead(pin_ldr); }

// ============================================================================
// 11. FUNÇÃO 3: CLASSIFICAÇÃO DA TEMPERATURA COM ESTRUTURAS IF/ELSE
// ============================================================================
TemperatureState AmbientMonitor::classify_temperature(
    const float temp_c) noexcept {
  auto state = TemperatureState::normal;
  if (temp_c <= temp_threshold_normal) {
    state = TemperatureState::normal;
  } else if (temp_c <= temp_threshold_critical) {
    state = TemperatureState::warning;
  } else {
    state = TemperatureState::critical;
  }
  return state;
}

// ============================================================================
// 12. FUNÇÃO 4: CLASSIFICAÇÃO DA LUMINOSIDADE COM ESTRUTURAS IF/ELSE
// ============================================================================
LightLevel AmbientMonitor::classify_luminosity(const int raw_adc) noexcept {
  auto level = LightLevel::moderate;
  if (ldr_pullup_mode) {
    // Configuração Pull-Up (Wokwi e Tinkercad Pull-Up):
    // No escuro a resistência sobe, elevando a tensão em A0 (ADC alto).
    // Na luz a resistência desce, derrubando a tensão em A0 (ADC baixo).
    if (raw_adc > light_threshold_dark_pullup) {
      level = LightLevel::dark;
    } else if (raw_adc < light_threshold_clear_pullup) {
      level = LightLevel::clear;
    } else {
      level = LightLevel::moderate;
    }
  } else {
    // Configuração Pull-Down (Tinkercad Pull-Down):
    // No escuro a tensão em A0 cai (ADC baixo).
    // Na luz a tensão em A0 sobe (ADC alto).
    if (raw_adc < light_threshold_dark_pulldown) {
      level = LightLevel::dark;
    } else if (raw_adc > light_threshold_clear_pulldown) {
      level = LightLevel::clear;
    } else {
      level = LightLevel::moderate;
    }
  }
  return level;
}

// ============================================================================
// 13. FUNÇÃO 5: ATUALIZAÇÃO DA SINALIZAÇÃO VISUAL E ACÚSTICA
// ============================================================================
void AmbientMonitor::update_signaling(const TemperatureState temp_state,
                                      const float temp_c,
                                      const bool activate_buzzer) {
  // Controle visual exclusivo dos LEDs baseado em if / else if / else
  if (temp_state == TemperatureState::normal) {
    // Condição Normal (T <= 25°C): apenas LED Verde aceso
    digitalWrite(pin_led_green, HIGH);
    digitalWrite(pin_led_yellow, LOW);
    analogWrite(pin_led_red, 0);
  } else if (temp_state == TemperatureState::warning) {
    // Desafio Técnico PWM: se a temperatura estiver na faixa de aproximação
    // crítica (>30°C até 35°C), o LED amarelo é apagado e o LED vermelho tem
    // seu brilho modulado via PWM proporcionalmente. Entre 25°C e 30°C, apenas
    // o LED amarelo fica aceso. Desta forma, os LEDs Amarelo e Vermelho NUNCA
    // ligam juntos!
    if (temp_c > temp_threshold_approach) {
      digitalWrite(pin_led_green, LOW);
      digitalWrite(pin_led_yellow,
                   LOW);  // Desliga o amarelo para evitar sobreposição

      // Interpolação linear do ciclo de trabalho (PWM de 30 a 255)
      const float ratio = (temp_c - temp_threshold_approach) /
                          (temp_threshold_critical - temp_threshold_approach);
      constexpr auto pwm_range =
          static_cast<float>(pwm_max_duty - pwm_min_duty);
      const float pwm_offset = ratio * pwm_range;
      const float calculated_pwm =
          static_cast<float>(pwm_min_duty) + pwm_offset;

      int duty_cycle = static_cast<int>(calculated_pwm);
      if (duty_cycle < 0) {
        duty_cycle = 0;
      } else if (duty_cycle > 255) {
        duty_cycle = 255;
      }

      analogWrite(pin_led_red, duty_cycle);
    } else {
      // Estado de Atenção padrão (25°C < T <= 30°C ou desafio desabilitado)
      digitalWrite(pin_led_green, LOW);
      digitalWrite(pin_led_yellow, HIGH);
      analogWrite(pin_led_red, 0);
    }
  } else {
    // Condição Crítica (T > 35°C): apenas LED Vermelho no brilho máximo
    // contínuo
    digitalWrite(pin_led_green, LOW);
    digitalWrite(pin_led_yellow, LOW);
    analogWrite(pin_led_red, pwm_max_duty);
  }

  // Controle da sirene sonora (Buzzer)
  if (activate_buzzer) {
    tone(pin_buzzer, buzzer_frequency_hz);
  } else {
    noTone(pin_buzzer);
  }
}

// ============================================================================
// 14. FUNÇÃO 6: TRATAMENTO DO BOTÃO COM DEBOUNCE NÃO-BLOQUEANTE
// ============================================================================
void AmbientMonitor::check_button() {
  const int current_reading = digitalRead(pin_button);
  const unsigned long current_ms = millis();

  // Reinicia a janela de debounce se houver variação no pino
  if (current_reading != m_button_last_reading) {
    m_button_last_change_ms = current_ms;
    m_button_last_reading = current_reading;
  }

  // Verifica se a leitura permaneceu estável pelo período de debounce
  if (current_ms - m_button_last_change_ms > debounce_delay_ms) {
    if (current_reading != m_button_stable_state) {
      m_button_stable_state = current_reading;

      // Transição de borda de descida (botão pressionado no pino com pull-up)
      if (m_button_stable_state == LOW) {
        m_alarm_enabled = !m_alarm_enabled;

        Serial.println();
        Serial.print(F(">>> [INTERFACE DO USUARIO] Alarme Sonoro "));
        Serial.println(is_alarm_enabled() ? F("HABILITADO <<<")
                                          : F("DESABILITADO (SILENCIADO) <<<"));
        Serial.println();
      }
    }
  }
}

// ============================================================================
// 15. FUNÇÃO 7: TRANSMISSÃO DE TELEMETRIA VIA MONITOR SERIAL
// ============================================================================
void AmbientMonitor::send_telemetry(const float temp_c,
                                    const TemperatureState temp_state,
                                    const int raw_ldr,
                                    const LightLevel light_lvl,
                                    const bool buzzer_active) const {
  Serial.print(F("[TELEMETRIA] Temp: "));
  Serial.print(temp_c, 1);
  Serial.print(F(" C ("));

  if (temp_state == TemperatureState::normal) {
    Serial.print(F("NORMAL"));
  } else if (temp_state == TemperatureState::warning) {
    if (temp_c > temp_threshold_approach) {
      Serial.print(F("ATENCAO - APROX. CRITICA PWM"));
    } else {
      Serial.print(F("ATENCAO"));
    }
  } else {
    Serial.print(F("CRITICO"));
  }

  Serial.print(F(") | LDR: "));
  Serial.print(raw_ldr);
  Serial.print(F(" ("));

  if (light_lvl == LightLevel::clear) {
    Serial.print(F("CLARA"));
  } else if (light_lvl == LightLevel::moderate) {
    Serial.print(F("MODERADA"));
  } else {
    Serial.print(F("ESCURA"));
  }

  Serial.print(F(") | Alarme: "));
  Serial.print(is_alarm_enabled() ? F("HABILITADO") : F("SILENCIADO"));

  Serial.print(F(" | Buzzer: "));
  Serial.println(buzzer_active ? F("ATIVO!") : F("Inativo"));
}

AmbientMonitor monitor;
}  // namespace

// ============================================================================
// PONTOS DE ENTRADA PADRÃO DO ARDUINO
// ============================================================================
void setup() { AmbientMonitor::init(); }

void loop() { monitor.update(); }
