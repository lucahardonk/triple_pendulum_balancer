// Board used esp32 c6 dev
// Tre encoder in quadratura — ESP32-C6, hardware PCNT x4.
//
// Collegamenti provvisori:
//   bottom: A -> GPIO18, B -> GPIO19 (collegamento originale)
//   middle: A -> GPIO20, B -> GPIO21
//   top:    A -> GPIO22, B -> GPIO23
//
// Verificare la disponibilita' dei GPIO sulla propria scheda.
// Per ogni ingresso: pull-up esterno da 3.3 kohm a 3V3.
// Massa comune tra encoder ed ESP32.
//
// Formato seriale, una riga per campione:
// bottom_rad,bottom_rad_s,middle_rad,middle_rad_s,top_rad,top_rad_s
//
// Nessuna etichetta, intestazione o modalita' plotter.
// Comando 'Z': azzera contemporaneamente la posizione dei tre encoder.

#include <Arduino.h>
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include <math.h>
#include <stdio.h>

class Encoder {
public:
  Encoder(int pinA, int pinB, int cpr)
    : pinA_(pinA),
      pinB_(pinB),
      countsPerRev_(cpr * QUADRATURE_FACTOR),
      radPerCount_(TWO_PI_F / (cpr * QUADRATURE_FACTOR)) {
    // Filtro alpha-beta con doppio polo reale:
    // alpha = 1 - r^2, beta = (1 - r)^2.
    const float r =
      expf(-TWO_PI_F * FILTER_BW_HZ * SAMPLE_PERIOD_S);

    alpha_ = 1.0f - r * r;
    beta_ = (1.0f - r) * (1.0f - r);
  }

  bool begin() {
    pcnt_unit_config_t unitCfg = {};
    unitCfg.low_limit = -PCNT_LIMIT;
    unitCfg.high_limit = PCNT_LIMIT;
    unitCfg.flags.accum_count = 1;

    if (pcnt_new_unit(&unitCfg, &unit_) != ESP_OK) return false;

    pcnt_glitch_filter_config_t filterCfg = {};
    filterCfg.max_glitch_ns = GLITCH_FILTER_NS;

    if (pcnt_unit_set_glitch_filter(unit_, &filterCfg) != ESP_OK) {
      return false;
    }

    pcnt_channel_handle_t chA = nullptr;
    pcnt_channel_handle_t chB = nullptr;

    if (!addChannel(pinA_, pinB_, &chA)) return false;
    if (!addChannel(pinB_, pinA_, &chB)) return false;

    // Decodifica in quadratura x4.
    if (pcnt_channel_set_edge_action(
          chA,
          PCNT_CHANNEL_EDGE_ACTION_DECREASE,
          PCNT_CHANNEL_EDGE_ACTION_INCREASE) != ESP_OK) {
      return false;
    }

    if (pcnt_channel_set_level_action(
          chA,
          PCNT_CHANNEL_LEVEL_ACTION_KEEP,
          PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK) {
      return false;
    }

    if (pcnt_channel_set_edge_action(
          chB,
          PCNT_CHANNEL_EDGE_ACTION_INCREASE,
          PCNT_CHANNEL_EDGE_ACTION_DECREASE) != ESP_OK) {
      return false;
    }

    if (pcnt_channel_set_level_action(
          chB,
          PCNT_CHANNEL_LEVEL_ACTION_KEEP,
          PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK) {
      return false;
    }

    // Necessari per accumulare i conteggi oltre i limiti hardware.
    if (pcnt_unit_add_watch_point(unit_, PCNT_LIMIT) != ESP_OK) {
      return false;
    }

    if (pcnt_unit_add_watch_point(unit_, -PCNT_LIMIT) != ESP_OK) {
      return false;
    }

    if (pcnt_unit_enable(unit_) != ESP_OK) return false;
    if (pcnt_unit_clear_count(unit_) != ESP_OK) return false;
    if (pcnt_unit_start(unit_) != ESP_OK) return false;

    const int64_t nowUs = esp_timer_get_time();

    lastCount_ = windowCount_ = rawCount();
    lastEdgeUs_ = windowEdgeUs_ = nowUs;
    prevPollUs_ = lastSampleUs_ = nowUs;

    return true;
  }

  // Chiamare a ogni giro di loop, il piu' spesso possibile.
  void update() {
    const int64_t nowUs = esp_timer_get_time();
    const int32_t count = rawCount();

    // 1. Rileva variazioni del contatore.
    // Il timestamp e' stimato dal polling, non catturato dall'hardware.
    if (count != lastCount_) {
      const int64_t edgeUs = (prevPollUs_ + nowUs) / 2;
      const int8_t dir = (count > lastCount_) ? 1 : -1;

      if (stalled_) {
        windowCount_ = count;
        windowEdgeUs_ = edgeUs;
        stalled_ = false;
      } else if (lastDir_ != 0 && dir != lastDir_) {
        // Inversione: riapre la finestra di misura.
        windowCount_ = count;
        windowEdgeUs_ = edgeUs;
        vRaw_ = 0.0f;
      }

      lastDir_ = dir;
      lastCount_ = count;
      lastEdgeUs_ = edgeUs;
    }

    prevPollUs_ = nowUs;

    // 2. Aggiorna posizione e stima della velocita' a circa 1 kHz.
    if (nowUs - lastSampleUs_ < SAMPLE_PERIOD_US) return;

    lastSampleUs_ += SAMPLE_PERIOD_US;

    // Non recuperare in raffica i campioni persi dopo un blocco.
    if (nowUs - lastSampleUs_ >= SAMPLE_PERIOD_US) {
      lastSampleUs_ = nowUs;
    }

    counts_ = count - zeroCount_;

    // Velocita' M/T: variazione di conteggio / tempo tra fronti osservati.
    const int32_t dCount = lastCount_ - windowCount_;
    const int64_t dTimeUs = lastEdgeUs_ - windowEdgeUs_;

    const bool enoughCounts = abs(dCount) >= MIN_WINDOW_COUNTS;
    const bool enoughTime =
      (dCount != 0) && (dTimeUs >= MIN_WINDOW_US);

    if (dTimeUs > 0 && (enoughCounts || enoughTime)) {
      vRaw_ = dCount * radPerCount_
            * (US_PER_S / static_cast<float>(dTimeUs));

      windowCount_ = lastCount_;
      windowEdgeUs_ = lastEdgeUs_;
    }

    // In assenza di fronti, limita progressivamente la velocita'.
    const int64_t idleUs = nowUs - lastEdgeUs_;

    if (idleUs >= STALL_TIMEOUT_US) {
      vRaw_ = 0.0f;
      velocity_ = 0.0f;
      accel_ = 0.0f;
      lastDir_ = 0;
      stalled_ = true;
      return;
    }

    if (idleUs > 0) {
      const float vMax =
        radPerCount_ * (US_PER_S / static_cast<float>(idleUs));

      if (fabsf(vRaw_) > vMax) {
        vRaw_ = copysignf(vMax, vRaw_);
      }
    }

    // Filtro alpha-beta sulla velocita'.
    // Alpha-beta con doppio polo reale in z = r = e^(-2π·f·T) (smorzamento critico):
    // z² − (2−α−β)z + (1−α) = (z−r)²  ->  α = 1 − r²,  β = (1 − r)² 
    // L'accelerazione resta uno stato interno, non viene esposta.
    const float vPred = velocity_ + accel_ * SAMPLE_PERIOD_S;
    const float residual = vRaw_ - vPred;

    velocity_ = vPred + alpha_ * residual;
    accel_ += (beta_ / SAMPLE_PERIOD_S) * residual;
  }

  // Angolo relativo allo zero, avvolto in [0, 2*pi).
  float angleRad() const {
    int32_t c = counts_ % countsPerRev_;
    if (c < 0) c += countsPerRev_;
    return c * radPerCount_;
  }

  // Velocita' filtrata in rad/s.
  float velocityRadS() const {
    return velocity_;
  }

  // Cambia solo il riferimento di posizione, non il filtro di velocita'.
  void setZero() {
    zeroCount_ = rawCount();
    counts_ = 0;
  }

private:
  static constexpr int QUADRATURE_FACTOR = 4;
  static constexpr float TWO_PI_F = 6.28318530718f;
  static constexpr float US_PER_S = 1e6f;

  static constexpr int PCNT_LIMIT = 30000;
  static constexpr uint32_t GLITCH_FILTER_NS = 1000;

  static constexpr int64_t SAMPLE_PERIOD_US = 1000;
  static constexpr float SAMPLE_PERIOD_S = 0.001f;

  static constexpr int32_t MIN_WINDOW_COUNTS = 4;
  static constexpr int64_t MIN_WINDOW_US = 5000;
  static constexpr int64_t STALL_TIMEOUT_US = 200000;
  static constexpr float FILTER_BW_HZ = 20.0f;

  bool addChannel(
      int edgePin,
      int levelPin,
      pcnt_channel_handle_t* channel) {
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
  const float radPerCount_;

  pcnt_unit_handle_t unit_ = nullptr;

  float alpha_ = 0.0f;
  float beta_ = 0.0f;

  int32_t zeroCount_ = 0;
  int32_t counts_ = 0;

  float vRaw_ = 0.0f;
  float velocity_ = 0.0f;
  float accel_ = 0.0f;

  int32_t lastCount_ = 0;
  int32_t windowCount_ = 0;

  int64_t lastEdgeUs_ = 0;
  int64_t windowEdgeUs_ = 0;
  int64_t prevPollUs_ = 0;
  int64_t lastSampleUs_ = 0;

  int8_t lastDir_ = 0;
  bool stalled_ = true;
};

// -----------------------------------------------------------------------------
// Configurazione hardware
// -----------------------------------------------------------------------------

constexpr int ENCODER_CPR = 2500;

constexpr int BOTTOM_PIN_A = 18;
constexpr int BOTTOM_PIN_B = 19;

constexpr int MIDDLE_PIN_A = 20;
constexpr int MIDDLE_PIN_B = 21;

constexpr int TOP_PIN_A = 22;
constexpr int TOP_PIN_B = 23;

Encoder bottom(BOTTOM_PIN_A, BOTTOM_PIN_B, ENCODER_CPR);
Encoder middle(MIDDLE_PIN_A, MIDDLE_PIN_B, ENCODER_CPR);
Encoder top(TOP_PIN_A, TOP_PIN_B, ENCODER_CPR);

// -----------------------------------------------------------------------------
// Trasmissione seriale
// -----------------------------------------------------------------------------

constexpr uint32_t SERIAL_BAUD = 921600;
constexpr int64_t TX_PERIOD_US = 1000;  // Frequenza richiesta: 1 kHz.

// Un solo produttore scrive sulla seriale: questa funzione.
// Se manca spazio, il frame viene scartato e non conservato per dopo.
void sendSample() {
  char frame[128];

  const int length = snprintf(
    frame,
    sizeof(frame),
    "%.4f,%.3f,%.4f,%.3f,%.4f,%.3f\n",
    bottom.angleRad(), bottom.velocityRadS(),
    middle.angleRad(), middle.velocityRadS(),
    top.angleRad(), top.velocityRadS()
  );

  if (length <= 0 || length >= static_cast<int>(sizeof(frame))) {
    return;
  }

  if (Serial.availableForWrite() < length) {
    return;
  }

  Serial.write(
    reinterpret_cast<const uint8_t*>(frame),
    static_cast<size_t>(length)
  );
}

// -----------------------------------------------------------------------------
// Setup e loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Nessuna stampa sul canale dati.
  // Se un encoder non viene inizializzato, non trasmettere misure incomplete.
  if (!bottom.begin() || !middle.begin() || !top.begin()) {
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  // 1. Aggiorna continuamente i tre encoder.
  bottom.update();
  middle.update();
  top.update();

  // 2. Legge al massimo un comando per giro, senza attese.
  if (Serial.available() > 0) {
    const char command = Serial.read();

    if (command == 'Z' || command == 'z') {
      bottom.setZero();
      middle.setZero();
      top.setZero();
    }
  }

  // 3. Invia al massimo un frame ogni millisecondo.
  // Nessuna raffica di recupero se un invio e' in ritardo.
  static int64_t lastTxUs = 0;
  const int64_t nowUs = esp_timer_get_time();

  if (nowUs - lastTxUs >= TX_PERIOD_US) {
    lastTxUs = nowUs;
    sendSample();
  }
}