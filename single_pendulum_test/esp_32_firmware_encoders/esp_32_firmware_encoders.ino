// Quadrature encoder reader for the inverted pendulum — ESP32-C6, hardware PCNT (x4).
// Wiring: A → GPIO18, B → GPIO19, 3.3 kΩ pull-ups to 3V3, encoder GND common with ESP32 GND.

#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include <math.h>

class Encoder {
public:
  Encoder(int pinA, int pinB, int cpr)
    : pinA_(pinA), pinB_(pinB),
      countsPerRev_(cpr * QUADRATURE_FACTOR),
      degPerCount_(DEG_PER_REV / (cpr * QUADRATURE_FACTOR)),
      radPerCount_(TWO_PI_F / (cpr * QUADRATURE_FACTOR)) {
    // Alpha-beta con doppio polo reale in z = r = e^(-2π·f·T) (smorzamento critico):
    // z² − (2−α−β)z + (1−α) = (z−r)²  ->  α = 1 − r²,  β = (1 − r)²
    const float r = expf(-TWO_PI_F * FILTER_BW_HZ * SAMPLE_PERIOD_S);
    alpha_ = 1.0f - r * r;
    beta_ = (1.0f - r) * (1.0f - r);
  }

  bool begin() {
    pcnt_unit_config_t unitCfg = {};
    unitCfg.low_limit = -PCNT_LIMIT;
    unitCfg.high_limit = PCNT_LIMIT;
    unitCfg.flags.accum_count = 1;  // estende il contatore HW a 16 bit a 32 bit
    if (pcnt_new_unit(&unitCfg, &unit_) != ESP_OK) return false;

    pcnt_glitch_filter_config_t filterCfg = { .max_glitch_ns = GLITCH_FILTER_NS };
    if (pcnt_unit_set_glitch_filter(unit_, &filterCfg) != ESP_OK) return false;

    pcnt_channel_handle_t chA, chB;
    if (!addChannel(pinA_, pinB_, &chA) || !addChannel(pinB_, pinA_, &chB)) return false;

    // Decodifica quadratura x4 standard
    pcnt_channel_set_edge_action(chA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // Watch point sui limiti: necessari perché accum_count gestisca gli overflow
    if (pcnt_unit_add_watch_point(unit_, PCNT_LIMIT) != ESP_OK) return false;
    if (pcnt_unit_add_watch_point(unit_, -PCNT_LIMIT) != ESP_OK) return false;

    if (pcnt_unit_enable(unit_) != ESP_OK) return false;
    if (pcnt_unit_clear_count(unit_) != ESP_OK) return false;
    if (pcnt_unit_start(unit_) != ESP_OK) return false;

    const int64_t nowUs = esp_timer_get_time();
    lastCount_ = windowCount_ = rawCount();
    lastEdgeUs_ = windowEdgeUs_ = prevPollUs_ = lastSampleUs_ = nowUs;
    return true;
  }

  // Da chiamare a OGNI giro di loop (il più spesso possibile).
  // 1) Ad ogni chiamata: rileva i fronti, ne registra l'istante e controlla le inversioni.
  // 2) Ogni SAMPLE_PERIOD_US (1 kHz): M/T con finestra minima + filtro alpha-beta.
  void update() {
    const int64_t nowUs = esp_timer_get_time();
    const int32_t count = rawCount();

    // --- 1) Timestamp dei fronti + rilevamento inversioni ---
    if (count != lastCount_) {
      // Il fronte è avvenuto tra la lettura precedente e questa: uso il punto medio
      const int64_t edgeUs = (prevPollUs_ + nowUs) / 2;
      const int8_t dir = (count > lastCount_) ? 1 : -1;

      if (stalled_) {                 // ripartenza: la finestra si apre su questo fronte
        windowCount_ = count;
        windowEdgeUs_ = edgeUs;
        stalled_ = false;
      } else if (lastDir_ != 0 && dir != lastDir_) {
        // Inversione di moto: la finestra riparte da qui, la velocità passa per lo zero
        windowCount_ = count;
        windowEdgeUs_ = edgeUs;
        vRaw_ = 0.0f;
      }
      lastDir_ = dir;
      lastCount_ = count;
      lastEdgeUs_ = edgeUs;
    }
    prevPollUs_ = nowUs;

    // --- 2) Stima a frequenza fissa ---
    if (nowUs - lastSampleUs_ < SAMPLE_PERIOD_US) return;
    lastSampleUs_ += SAMPLE_PERIOD_US;
    if (nowUs - lastSampleUs_ >= SAMPLE_PERIOD_US) lastSampleUs_ = nowUs;  // riallinea dopo un blocco

    counts_ = count - zeroCount_;   // posizione campionata

    // M/T con finestra minima: Δcount e Δt misurati tra fronti.
    // La finestra si chiude con almeno MIN_WINDOW_COUNTS conteggi (4 = ciclo di quadratura
    // completo, annulla l'errore di fase A/B) oppure dopo MIN_WINDOW_US con almeno 1 conteggio.
    const int32_t dCount = lastCount_ - windowCount_;
    const int64_t dTimeUs = lastEdgeUs_ - windowEdgeUs_;
    const bool enoughCounts = abs(dCount) >= MIN_WINDOW_COUNTS;
    const bool enoughTime = (dCount != 0) && (dTimeUs >= MIN_WINDOW_US);
    if (dTimeUs > 0 && (enoughCounts || enoughTime)) {
      vRaw_ = dCount * radPerCount_ * (US_PER_S / static_cast<float>(dTimeUs));
      windowCount_ = lastCount_;
      windowEdgeUs_ = lastEdgeUs_;
    }

    // Senza fronti da 'idle' µs vale |ω| <= 1 count / idle -> la misura decade con continuità
    const int64_t idleUs = nowUs - lastEdgeUs_;
    if (idleUs >= STALL_TIMEOUT_US) {
      vRaw_ = velocity_ = accel_ = 0.0f;
      lastDir_ = 0;
      stalled_ = true;
      return;
    }
    if (idleUs > 0) {
      const float vMax = radPerCount_ * (US_PER_S / static_cast<float>(idleUs));
      if (fabsf(vRaw_) > vMax) vRaw_ = copysignf(vMax, vRaw_);
    }

    // Filtro alpha-beta (stati: velocità, accelerazione), misura = velocità M/T.
    // Predizione ad accelerazione costante -> niente ritardo a regime anche su rampe di velocità.
    const float vPred = velocity_ + accel_ * SAMPLE_PERIOD_S;
    const float residual = vRaw_ - vPred;
    velocity_ = vPred + alpha_ * residual;
    accel_ += (beta_ / SAMPLE_PERIOD_S) * residual;
  }

  // Angolo avvolto nel range [0, 360)
  float angleDeg() const {
    int32_t c = counts_ % countsPerRev_;
    if (c < 0) c += countsPerRev_;
    return c * degPerCount_;
  }

  float velocityRadS() const { return velocity_; }     // filtrata (da usare nel controllo)
  float velocityRawRadS() const { return vRaw_; }      // M/T grezza
  float accelRadS2() const { return accel_; }          // accelerazione stimata

  void setZero() {
    zeroCount_ = rawCount();
    counts_ = 0;
  }

  // Stampa leggibile (Serial Monitor)
  void print() const {
    Serial.printf("θ: %.2f° θ̇: %.3f rad/s\n", angleDeg(), velocity_);
  }

  // Stampa per il Serial Plotter (formato label:valore): grezza vs filtrata
  void printPlotter() const {
    Serial.printf("vel_raw:%.3f,vel_filt:%.3f\n", vRaw_, velocity_);
  }

private:
  static constexpr int QUADRATURE_FACTOR = 4;
  static constexpr float DEG_PER_REV = 360.0f;
  static constexpr float TWO_PI_F = 6.28318530718f;
  static constexpr float US_PER_S = 1e6f;
  static constexpr int PCNT_LIMIT = 30000;            // dentro il range HW a 16 bit
  static constexpr uint32_t GLITCH_FILTER_NS = 1000;  // < 2.5 µs di fase a 100 kHz
  static constexpr int64_t SAMPLE_PERIOD_US = 1000;   // stima a 1 kHz
  static constexpr float SAMPLE_PERIOD_S = 0.001f;
  static constexpr int32_t MIN_WINDOW_COUNTS = 4;     // ciclo di quadratura completo
  static constexpr int64_t MIN_WINDOW_US = 5000;      // oppure 5 ms
  static constexpr int64_t STALL_TIMEOUT_US = 200000; // sotto ~0.003 rad/s -> fermo
  static constexpr float FILTER_BW_HZ = 20.0f;        // banda alpha-beta: ↑ più reattivo, ↓ più smooth

  bool addChannel(int edgePin, int levelPin, pcnt_channel_handle_t* channel) {
    pcnt_chan_config_t cfg = {};
    cfg.edge_gpio_num = edgePin;
    cfg.level_gpio_num = levelPin;
    return pcnt_new_channel(unit_, &cfg, channel) == ESP_OK;
  }

  int32_t rawCount() const {
    int value = 0;
    pcnt_unit_get_count(unit_, &value);
    return value;
  }

  const int pinA_;
  const int pinB_;
  const int32_t countsPerRev_;
  const float degPerCount_;
  const float radPerCount_;
  float alpha_ = 0.0f;
  float beta_ = 0.0f;
  pcnt_unit_handle_t unit_ = nullptr;

  int32_t zeroCount_ = 0;
  int32_t counts_ = 0;          // posizione campionata a 1 kHz
  float vRaw_ = 0.0f;           // velocità M/T grezza
  float velocity_ = 0.0f;       // velocità filtrata alpha-beta
  float accel_ = 0.0f;          // accelerazione stimata alpha-beta

  int32_t lastCount_ = 0;       // conteggio all'ultimo fronte
  int64_t lastEdgeUs_ = 0;      // istante dell'ultimo fronte
  int32_t windowCount_ = 0;     // conteggio al fronte di inizio finestra
  int64_t windowEdgeUs_ = 0;    // istante del fronte di inizio finestra
  int64_t prevPollUs_ = 0;
  int64_t lastSampleUs_ = 0;
  int8_t lastDir_ = 0;          // verso dell'ultimo fronte (+1 / -1, 0 = sconosciuto)
  bool stalled_ = true;
};

constexpr int PIN_A = 18;
constexpr int PIN_B = 19;
constexpr int ENCODER_CPR = 2500;
constexpr uint32_t PRINT_PERIOD_MS = 10;  // 100 Hz

Encoder encoder(PIN_A, PIN_B, ENCODER_CPR);
bool plotterMode = false;  // false = Serial Monitor, true = Serial Plotter

void setup() {
  Serial.begin(921600);
  if (!encoder.begin()) {
    Serial.println("Encoder init failed");
    while (true) delay(1000);
  }
  Serial.println("Encoder ready. 'Z' = zero, 'M' = cambia modalita' (monitor/plotter).");
}

void loop() {
  encoder.update();  // ad ogni giro: timestamp fronti + stima a 1 kHz

  while (Serial.available()) {
    const char c = Serial.read();
    if (c == 'Z' || c == 'z') encoder.setZero();
    if (c == 'M' || c == 'm') plotterMode = !plotterMode;
  }

  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs += PRINT_PERIOD_MS;
    if (plotterMode) encoder.printPlotter();
    else             encoder.print();
  }
}