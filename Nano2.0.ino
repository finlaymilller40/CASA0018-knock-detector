

#define EIDSP_QUANTIZE_FILTERBANK 0
#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW 4

#include <PDM.h>
#include <Knock_detector_project_3.0_inferencing.h>
#include <ArduinoBLE.h>

#define CONFIDENCE_THRESHOLD 0.85
#define KNOCK_COOLDOWN_MS 5000

BLEService knockService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic knockChar(
  "19B10001-E8F2-537E-4F6C-D104768A1214",
  BLERead | BLENotify | BLEWrite, 32
);

void ledRed()  { digitalWrite(LEDR,LOW);  digitalWrite(LEDG,HIGH); digitalWrite(LEDB,HIGH); }
void ledBlue() { digitalWrite(LEDR,HIGH); digitalWrite(LEDG,HIGH); digitalWrite(LEDB,LOW);  }

enum DeviceState { STANDBY, KNOCK_ALERT };
DeviceState currentState = STANDBY;

static unsigned long lastKnockTime = 0;

typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

static inference_t inference;
static bool record_ready = false;
static signed short *sampleBuffer;
static bool debug_nn = false;
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);

void resetToStandby() {
    currentState = STANDBY;
    ledBlue();
    knockChar.writeValue("Listening...");
    Serial.println("Returned to standby");
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    ledBlue();

    if (!BLE.begin()) {
        Serial.println("BLE failed!");
        while (1);
    }
    BLE.setLocalName("KnockDetector");
    BLE.setAdvertisedService(knockService);
    knockService.addCharacteristic(knockChar);
    BLE.addService(knockService);
    knockChar.writeValue("Listening...");
    BLE.advertise();
    Serial.println("BLE ready");

    run_classifier_init();
    if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
        Serial.println("Microphone init failed!");
        return;
    }

    Serial.println("Knock detector ready!");
    ledBlue();
}

void loop() {
    BLE.poll();

    if (knockChar.written()) {
        String received = String(knockChar.value().c_str());
        if (received == "ack") {
            Serial.println("ACK received via BLE");
            resetToStandby();
        }
    }

    bool m = microphone_inference_record();
    if (!m) return;

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) return;

    if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)) {

        float knockConf = 0;

        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (strcmp(result.classification[ix].label, "General Knock") == 0) {
                knockConf = result.classification[ix].value;
            }
        }

        unsigned long now = millis();

        if (knockConf >= CONFIDENCE_THRESHOLD && currentState == STANDBY) {
            Serial.println("KNOCK");
            currentState = KNOCK_ALERT;
            ledRed();
            knockChar.writeValue("Someone at the door!");
            lastKnockTime = now;
        }

        print_results = 0;
    }
}

static void pdm_data_ready_inference_callback(void) {
    int bytesAvailable = PDM.available();
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);
    if (record_ready == true) {
        for (int i = 0; i < bytesRead >> 1; i++) {
            inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];
            if (inference.buf_count >= inference.n_samples) {
                inference.buf_select ^= 1;
                inference.buf_count = 0;
                inference.buf_ready = 1;
            }
        }
    }
}

static bool microphone_inference_start(uint32_t n_samples) {
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));
    if (inference.buffers[0] == NULL) return false;
    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));
    if (inference.buffers[1] == NULL) { free(inference.buffers[0]); return false; }
    sampleBuffer = (signed short *)malloc((n_samples >> 1) * sizeof(signed short));
    if (sampleBuffer == NULL) {
        free(inference.buffers[0]);
        free(inference.buffers[1]);
        return false;
    }
    inference.buf_select = 0;
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;
    PDM.onReceive(&pdm_data_ready_inference_callback);
    PDM.setBufferSize((n_samples >> 1) * sizeof(int16_t));
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        Serial.println("PDM failed!");
    }
    PDM.setGain(45);
    record_ready = true;
    return true;
}

static bool microphone_inference_record(void) {
    bool ret = true;
    if (inference.buf_ready == 1) {
        ret = false;
    }
    while (inference.buf_ready == 0) { delay(1); }
    inference.buf_ready = 0;
    return ret;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(
        &inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    return 0;
}

static void microphone_inference_end(void) {
    PDM.end();
    free(inference.buffers[0]);
    free(inference.buffers[1]);
    free(sampleBuffer);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif