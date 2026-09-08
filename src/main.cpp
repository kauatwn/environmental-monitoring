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

// Estados de temperatura para controle dos atuadores
enum class TemperatureState : uint8_t {
  normal,    // Temperatura <= 25 °C
  warning,   // 25 °C < Temperatura <= 35 °C
  critical,  // Temperatura > 35 °C
};

// Níveis de luminosidade ambiente
enum class LightLevel : uint8_t {
  dark,      // Escuro (gera condição de alarme)
  moderate,  // Moderado
  clear,     // Claro
};

// Leitura e debounce do botão para ligar/desligar o alarme sonoro
class DebouncedButton {
 public:
  explicit DebouncedButton(const uint8_t pin) : m_pin(pin) {}

  void init() const { pinMode(m_pin, INPUT_PULLUP); }

  // Filtra ruídos elétricos mecânicos e alterna o estado do alarme na borda de descida (botão pressionado).
  // Retorna true apenas no instante em que o operador altera o estado.
  bool update(const unsigned long current_ms) {
    const int current_reading = digitalRead(m_pin);

    // Reinicia o temporizador caso a leitura bruta sofra oscilação
    if (current_reading != m_last_reading) {
      m_last_change_ms = current_ms;
      m_last_reading = current_reading;
    }

    // Aguarda o sinal estabilizar pelo período de debounce
    if (current_ms - m_last_change_ms <= debounce_delay_ms) {
      return false;
    }

    // Se o sinal estabilizado for idêntico ao estado anterior, nenhuma ação é tomada
    if (current_reading == m_stable_state) {
      return false;
    }

    m_stable_state = current_reading;

    // Atua exclusivamente quando o botão vai para nível baixo (pressionado com resistor pull-up)
    if (m_stable_state != LOW) {
      return false;
    }

    m_toggle_state = !m_toggle_state;
    return true;
  }

  [[nodiscard]] bool is_enabled() const noexcept { return m_toggle_state; }

 private:
  uint8_t m_pin;
  bool m_toggle_state{true};
  int m_last_reading{HIGH};
  int m_stable_state{HIGH};
  unsigned long m_last_change_ms{0};
};

// Leitura e conversão de temperatura em graus Celsius
class TemperatureSensor {
 public:
  explicit TemperatureSensor(const uint8_t pin) : m_pin(pin) {}

  // Lê a entrada analógica e calcula a temperatura em °C conforme o sensor utilizado
  [[nodiscard]] float read_celsius() const {
    const int raw_adc = analogRead(m_pin);

    // Conversão para o sensor analógico linear TMP36:
    // Tensão (V) = ADC * (5.0 / 1024.0)
    // O TMP36 possui saída de 500 mV (0.5 V) a 0 °C e escala de 10 mV/°C (0.01 V/°C):
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

    // Conversão para termistor NTC via Equação do Parâmetro Beta (Steinhart-Hart simplificada):
    // Em um divisor de tensão com resistor de referência de 10 kOhm:
    //   adc_ratio = R_ntc / R_ref = 1.0 / ((1023.0 / ADC) - 1.0)
    // Pela relação exponencial de temperatura do termistor:
    //   1 / T = 1 / T0 + (1 / Beta) * ln(R_ntc / R0)
    // onde T0 = 298.15 K (25 °C), Beta = 3950 e T é a temperatura absoluta em Kelvin:
    //   Temperatura (°C) = T (Kelvin) - 273.15
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

  // Classifica a temperatura nas faixas de operação do sistema
  [[nodiscard]] static TemperatureState classify(const float temp_c) noexcept {
    if (temp_c <= temp_threshold_normal) {
      return TemperatureState::normal;
    }
    if (temp_c <= temp_threshold_critical) {
      return TemperatureState::warning;
    }
    return TemperatureState::critical;
  }

 private:
  uint8_t m_pin;
};

// Leitura e classificação da luminosidade ambiente
class LuminositySensor {
 public:
  explicit LuminositySensor(const uint8_t pin) : m_pin(pin) {}

  [[nodiscard]] int read_adc() const { return analogRead(m_pin); }

  // Determina o nível de claridade conforme a montagem do divisor de tensão
  [[nodiscard]] static LightLevel classify(const int raw_adc) noexcept {
    if (ldr_pullup_mode) {
      // Modo Pull-Up: ambiente mais escuro eleva a tensão no pino A0
      if (raw_adc > light_threshold_dark_pullup) {
        return LightLevel::dark;
      }
      if (raw_adc < light_threshold_clear_pullup) {
        return LightLevel::clear;
      }
      return LightLevel::moderate;
    }

    // Modo Pull-Down: ambiente mais escuro reduz a tensão no pino A0
    if (raw_adc < light_threshold_dark_pulldown) {
      return LightLevel::dark;
    }
    if (raw_adc > light_threshold_clear_pulldown) {
      return LightLevel::clear;
    }
    return LightLevel::moderate;
  }

 private:
  uint8_t m_pin;
};

// Pinos de conexão dos LEDs
struct LedPins {
  uint8_t green;
  uint8_t yellow;
  uint8_t red;
};

// Controle visual dos LEDs e variação gradual de brilho (PWM)
class VisualSignaling {
 public:
  explicit VisualSignaling(const LedPins pins) : m_pins(pins) {}

  void init() const {
    pinMode(m_pins.green, OUTPUT);
    pinMode(m_pins.yellow, OUTPUT);
    pinMode(m_pins.red, OUTPUT);

    digitalWrite(m_pins.green, LOW);
    digitalWrite(m_pins.yellow, LOW);
    analogWrite(m_pins.red, 0);
  }

  // Atualiza as saídas dos LEDs garantindo que apenas a indicação correta para cada estado permaneça ligada
  void update(const TemperatureState temp_state, const float temp_c) const {
    // Faixa normal (<= 25 °C): apenas LED verde ativo
    if (temp_state == TemperatureState::normal) {
      digitalWrite(m_pins.green, HIGH);
      digitalWrite(m_pins.yellow, LOW);
      analogWrite(m_pins.red, 0);
      return;
    }

    // Faixa intermediária (25 °C a 35 °C):
    // - 25 °C a 30 °C: LED amarelo ativo
    // - 30 °C a 35 °C: LED amarelo desliga e LED vermelho varia o brilho progressivamente via PWM
    if (temp_state == TemperatureState::warning) {
      if (temp_c > temp_threshold_approach) {
        digitalWrite(m_pins.green, LOW);
        digitalWrite(m_pins.yellow, LOW);
        analogWrite(m_pins.red, calculate_pwm_duty(temp_c));
        return;
      }
      digitalWrite(m_pins.green, LOW);
      digitalWrite(m_pins.yellow, HIGH);
      analogWrite(m_pins.red, 0);
      return;
    }

    // Estado crítico (> 35 °C): LED vermelho aceso em intensidade máxima
    digitalWrite(m_pins.green, LOW);
    digitalWrite(m_pins.yellow, LOW);
    analogWrite(m_pins.red, pwm_max_duty);
  }

 private:
  // Interpolação linear para cálculo do PWM entre 30.0 °C e 35.0 °C:
  //   razao = (T - 30.0) / (35.0 - 30.0)  [escala de 0.0 a 1.0]
  //   duty  = pwm_min (30) + razao * (pwm_max (255) - pwm_min (30))
  [[nodiscard]] static int calculate_pwm_duty(const float temp_c) noexcept {
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

  LedPins m_pins;
};

// Controle do buzzer piezoelétrico para emissão do alerta sonoro
class AcousticAlarm {
 public:
  explicit AcousticAlarm(const uint8_t pin) : m_pin(pin) {}

  void init() const {
    pinMode(m_pin, OUTPUT);
    noTone(m_pin);
  }

  // Aciona tom com frequência constante de 1000 Hz ou silencia o componente
  void update(const bool activate) const {
    if (activate) {
      tone(m_pin, buzzer_frequency_hz);
      return;
    }
    noTone(m_pin);
  }

 private:
  uint8_t m_pin;
};

// Formatação e envio de dados via porta serial
class TelemetryReporter {
 public:
  static void init(const unsigned long baud_rate = 9600) {
    Serial.begin(baud_rate);
    Serial.println(F("=================================================="));
    Serial.println(F(" SISTEMA DE MONITORAMENTO AMBIENTAL - ARDUINO UNO"));
    Serial.println(F(" Status: Inicializado com Sucesso                "));
    Serial.println(F("=================================================="));
  }

  // Notifica no terminal serial sempre que o operador alterar o estado do botão
  static void print_alarm_toggle(const bool alarm_enabled) {
    Serial.println();
    Serial.print(F(">>> [INTERFACE DO USUARIO] Alarme Sonoro "));
    Serial.println(alarm_enabled ? F("HABILITADO <<<") : F("DESABILITADO (SILENCIADO) <<<"));
    Serial.println();
  }

  // Envia linha periódica com o relatório das variáveis ambientais e atuadores
  static void send(const float temp_c, const TemperatureState temp_state, const int raw_ldr, const LightLevel light_lvl,
                   const bool alarm_enabled, const bool buzzer_active) {
    Serial.print(F("[TELEMETRIA] Temp: "));
    Serial.print(temp_c, 1);
    Serial.print(F(" C ("));
    Serial.print(get_temperature_label(temp_state, temp_c));

    Serial.print(F(") | LDR: "));
    Serial.print(raw_ldr);
    Serial.print(F(" ("));
    Serial.print(get_luminosity_label(light_lvl));

    Serial.print(F(") | Alarme: "));
    Serial.print(alarm_enabled ? F("HABILITADO") : F("SILENCIADO"));

    Serial.print(F(" | Buzzer: "));
    Serial.println(buzzer_active ? F("ATIVO!") : F("Inativo"));
  }

 private:
  [[nodiscard]] static const __FlashStringHelper* get_temperature_label(const TemperatureState state,
                                                                        const float temp_c) noexcept {
    if (state == TemperatureState::normal) {
      return F("NORMAL");
    }
    if (state == TemperatureState::warning) {
      if (temp_c > temp_threshold_approach) {
        return F("ATENCAO - APROX. CRITICA PWM");
      }
      return F("ATENCAO");
    }
    return F("CRITICO");
  }

  [[nodiscard]] static const __FlashStringHelper* get_luminosity_label(const LightLevel level) noexcept {
    if (level == LightLevel::clear) {
      return F("CLARA");
    }
    if (level == LightLevel::moderate) {
      return F("MODERADA");
    }
    return F("ESCURA");
  }
};

// Gerenciamento e coordenação do sistema de monitoramento
class AmbientMonitor {
 public:
  AmbientMonitor(TemperatureSensor& temp_sensor, LuminositySensor& light_sensor, DebouncedButton& button,
                 VisualSignaling& visual, AcousticAlarm& acoustic)
      : m_temp_sensor(&temp_sensor),
        m_light_sensor(&light_sensor),
        m_button(&button),
        m_visual(&visual),
        m_acoustic(&acoustic) {}

  void init() const {
    TelemetryReporter::init();
    m_button->init();
    m_visual->init();
    m_acoustic->init();
  }

  // Ciclo principal de atualização e lógica de decisão do sistema
  void update() {
    const unsigned long current_ms = millis();

    // 1. Processa a leitura do botão e notifica no terminal se o operador acionou o silenciador
    if (m_button->update(current_ms)) {
      TelemetryReporter::print_alarm_toggle(m_button->is_enabled());
    }

    // 2. Aquisição dos valores medidos pelos sensores
    const float current_temp_c = m_temp_sensor->read_celsius();
    const int current_light_adc = m_light_sensor->read_adc();

    const auto temp_state = TemperatureSensor::classify(current_temp_c);
    const auto light_level = LuminositySensor::classify(current_light_adc);

    // 3. Regra de disparo do alarme: temperatura > 35 °C OU ambiente escuro
    const bool alarm_condition_detected = current_temp_c > temp_threshold_critical || light_level == LightLevel::dark;

    // 4. O alarme acústico só dispara se o alarme sonoro estiver habilitado pelo usuário
    const bool activate_buzzer = m_button->is_enabled() && alarm_condition_detected;

    // 5. Atualização dos estados dos atuadores
    m_visual->update(temp_state, current_temp_c);
    m_acoustic->update(activate_buzzer);

    // 6. Transmissão periódica das informações para a porta serial (a cada 1000 ms)
    if (!m_telemetry_started || current_ms - m_last_telemetry_ms >= telemetry_interval_ms) {
      m_telemetry_started = true;
      m_last_telemetry_ms = current_ms;
      TelemetryReporter::send(current_temp_c, temp_state, current_light_adc, light_level, m_button->is_enabled(),
                              activate_buzzer);
    }
  }

 private:
  TemperatureSensor* m_temp_sensor{nullptr};
  LuminositySensor* m_light_sensor{nullptr};
  DebouncedButton* m_button{nullptr};
  VisualSignaling* m_visual{nullptr};
  AcousticAlarm* m_acoustic{nullptr};

  bool m_telemetry_started{false};
  unsigned long m_last_telemetry_ms{0};
};

// Instanciação estática dos módulos de sensoriamento e controle
TemperatureSensor temp_sensor(pin_temp);
LuminositySensor light_sensor(pin_ldr);
DebouncedButton system_button(pin_button);
VisualSignaling visual_signaling({
    .green = pin_led_green,
    .yellow = pin_led_yellow,
    .red = pin_led_red,
});
AcousticAlarm acoustic_alarm(pin_buzzer);

AmbientMonitor monitor(temp_sensor, light_sensor, system_button, visual_signaling, acoustic_alarm);
}  // namespace

void setup() { monitor.init(); }

void loop() { monitor.update(); }
