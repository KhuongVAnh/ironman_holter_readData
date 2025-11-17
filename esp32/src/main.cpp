// #include <WiFi.h>
// #include <HTTPClient.h>
// #include <Arduino.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>

// // --- Khai báo cho cảm biến MPU 6050 ---
// Adafruit_MPU6050 mpu;

// const char *ssid = "Vankkk";    // WiFi SSID
// const char *password = "vanhvinh"; // WiFi password
// const char *serverUrl = "http://172.20.10.2:3000/api/telemetry";

// const int ecgPin = 34;     // OUT pin of AD8232 -> GPIO34 (ADC input)
// const int loMinusPin = 14; // LO- pin of AD8232
// const int loPlusPin = 27;  // LO+ pin of AD8232
// const int sdnPin = 25;

// const int SAMPLE_RATE = 250;                    // Hz (ECG)
// const int DURATION = 5;                         // seconds mỗi batch
// const int NUM_SAMPLES = SAMPLE_RATE * DURATION; // số mẫu ECG trong 5s
// const int BUFFER_SIZE = NUM_SAMPLES;

// // MPU lấy mẫu 50Hz
// const int MPU_SAMPLE_RATE = 50;                       // Hz (MPU)
// const int MPU_NUM_SAMPLES = MPU_SAMPLE_RATE * DURATION; // số mẫu MPU trong 5s

// float ecgData[BUFFER_SIZE]; // buffer rolling
// // Buffers for 5 seconds of accelerometer and gyroscope data
// float accelX[MPU_NUM_SAMPLES];
// float accelY[MPU_NUM_SAMPLES];
// float accelZ[MPU_NUM_SAMPLES];
// float gyroX[MPU_NUM_SAMPLES];
// float gyroY[MPU_NUM_SAMPLES];
// float gyroZ[MPU_NUM_SAMPLES];

// int indexSample = 0;

// // Đọc MPU6050 và lưu vào buffer tại vị trí index
// void readMpuSensorToBuffer(int index)
// {
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   accelX[index] = a.acceleration.x;
//   accelY[index] = a.acceleration.y;
//   accelZ[index] = a.acceleration.z;
//   gyroX[index] = g.gyro.x;
//   gyroY[index] = g.gyro.y;
//   gyroZ[index] = g.gyro.z;
// }

// // --- Bộ lọc Notch 50Hz ---
// float x1_ = 0, x2_ = 0; // input buffer
// float y1_ = 0, y2_ = 0; // output buffer
// const float b0 = 0.9723;
// const float b1 = -1.8478;
// const float b2 = 0.9723;
// const float a1 = -1.8478;
// const float a2 = 0.9446;

// // --- Hàm lọc ---
// float notchFilter(float x)
// {
//   float y = b0 * x + b1 * x1_ + b2 * x2_ - a1 * y1_ - a2 * y2_;
//   x2_ = x1_;
//   x1_ = x;
//   y2_ = y1_;
//   y1_ = y;
//   return y;
// }

// void setup()
// {
//   Serial.begin(9600);
//   analogReadResolution(12); // ESP32 ADC 12-bit (0-4095)

//   pinMode(loMinusPin, INPUT);
//   pinMode(loPlusPin, INPUT);
//   pinMode(ecgPin, INPUT);

//   pinMode(sdnPin, OUTPUT);
//   digitalWrite(sdnPin, LOW); // LOW = bật module, HIGH = shutdown

//   Serial.println("ECG Monitor Started...");

//   // --- Khởi tạo cho MPU 6050 ---
//   if (!mpu.begin())
//   {
//     Serial.println("Không tìm thấy cảm biến MPU6050! Kiểm tra lại kết nối.");
//     while (1)
//     {
//       delay(10);
//     }
//   }
//   Serial.println("Đã tìm thấy MPU6050!");

//   // Cài đặt các thông số cho MPU6050
//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

//   // init buffers
//   for (int i = 0; i < BUFFER_SIZE; i++)
//   {
//     ecgData[i] = 0;
//   }
//   for (int i = 0; i < MPU_NUM_SAMPLES; i++)
//   {
//     accelX[i] = 0;
//     accelY[i] = 0;
//     accelZ[i] = 0;
//     gyroX[i] = 0;
//     gyroY[i] = 0;
//     gyroZ[i] = 0;
//   }

//   // WiFi connect
//   WiFi.begin(ssid, password);
//   Serial.print("Đang kết nối WiFi");
//   while (WiFi.status() != WL_CONNECTED)
//   {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("\nWiFi connected!");
// }

// unsigned long lastSample = 0;
// unsigned long lastMpuSample = 0;
// int mpuIndex = 0;
// void loop()
// {
//   if (micros() - lastSample >= 4000)
//   { // 4000 µs = 4 ms
//     lastSample = micros();
//     if (indexSample < NUM_SAMPLES)
//     {
//       // đọc ADC
//       int ecgValue = analogRead(ecgPin);
//       float V = ((float)ecgValue / 4095.0) * 3.3;
//       V = (V - 1.65); // dịch offset về 0

//       ecgData[indexSample] = notchFilter(notchFilter(V));
//       // đọc MPU 50Hz (mỗi 20ms) và lưu vào buffer riêng
//       if ((micros() - lastMpuSample) >= 20000 && mpuIndex < MPU_NUM_SAMPLES)
//       {
//         lastMpuSample = micros();
//         readMpuSensorToBuffer(mpuIndex);
//         mpuIndex++;
//       }
//       indexSample++;
//     }
//     else
//     {
//       // đủ 5s thì in và gửi
//       Serial.println("=== ECG Data (5s rolling) ===");
//       String result = "";

//       for (int i = 0; i < BUFFER_SIZE; i++)
//       {
//         result += String(ecgData[i], 3);
//         if (i < BUFFER_SIZE - 1)
//           result += ",";
//       }

//       indexSample = 0;
//       // reset MPU chỉ số cho batch mới
//       int mpuCount = mpuIndex; // số mẫu thực tế lấy được trong 5s
//       mpuIndex = 0;
//       lastMpuSample = micros();

//       // gửi lên server
//       if (WiFi.status() == WL_CONNECTED)
//       {
//         HTTPClient http;
//         http.begin(serverUrl);
//         http.addHeader("Content-Type", "application/json");

//         // Dự trữ bộ nhớ để tránh phân mảnh và cắt chuỗi
//         String ecg; ecg.reserve(BUFFER_SIZE * 8);
//         String json; json.reserve(BUFFER_SIZE * 10 + MPU_NUM_SAMPLES * 90 + 128);

//         ecg = "[" + result + "]";

//         // Xây JSON trực tiếp để giảm bản sao tạm
//         json = "{\"ecg_signal\":";
//         json += ecg;
//         json += ",\"accel\":[";
//         for (int i = 0; i < mpuCount; i++)
//         {
//           json += "{\"x\":" + String(accelX[i], 3) + ",\"y\":" + String(accelY[i], 3) + ",\"z\":" + String(accelZ[i], 3) + "}";
//           if (i < mpuCount - 1) json += ",";
//         }
//         json += "],\"gyro\":[";
//         for (int i = 0; i < mpuCount; i++)
//         {
//           json += "{\"x\":" + String(gyroX[i], 3) + ",\"y\":" + String(gyroY[i], 3) + ",\"z\":" + String(gyroZ[i], 3) + "}";
//           if (i < mpuCount - 1) json += ",";
//         }
//         json += "]}";

//         int httpCode = http.POST(json);
//         if (httpCode > 0)
//         {
//           Serial.print("Response code: ");
//           Serial.println(httpCode);
//           // String payload = http.getString();
//           // Serial.println("Server response: " + payload);
//         }
//         else
//         {
//           Serial.print("Error sending POST: ");
//           Serial.println(httpCode);
//         }
//         http.end();
//       }
//     }
//   }
// }

#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ArduinoJson.h>

// ---------- CẤU HÌNH ----------
const char *ssid = "Vankkk";
const char *password = "vanhhh";
const char *serverUrl = "http://172.20.10.2:3000/api/telemetry";

#define ECG_PIN 34
#define LO_MINUS_PIN 14
#define LO_PLUS_PIN 27
#define SDN_PIN 25

#define SAMPLE_RATE 250 // ECG Hz
#define DURATION 5      // seconds per batch
#define NUM_SAMPLES (SAMPLE_RATE * DURATION)

int MPU_SAMPLE_RATE = 100; // có thể chỉnh linh hoạt
#define MAX_MPU_RATE 250  // tránh tràn bộ nhớ

Adafruit_MPU6050 mpu;

// ECG buffer
float ecgBufA[NUM_SAMPLES];
float ecgBufB[NUM_SAMPLES];

// MPU buffers
float accelX_A[MAX_MPU_RATE * DURATION];
float accelY_A[MAX_MPU_RATE * DURATION];
float accelZ_A[MAX_MPU_RATE * DURATION];
float gyroX_A[MAX_MPU_RATE * DURATION];
float gyroY_A[MAX_MPU_RATE * DURATION];
float gyroZ_A[MAX_MPU_RATE * DURATION];

float accelX_B[MAX_MPU_RATE * DURATION];
float accelY_B[MAX_MPU_RATE * DURATION];
float accelZ_B[MAX_MPU_RATE * DURATION];
float gyroX_B[MAX_MPU_RATE * DURATION];
float gyroY_B[MAX_MPU_RATE * DURATION];
float gyroZ_B[MAX_MPU_RATE * DURATION];

// Cờ double buffer
volatile bool bufferReadyA = false;
volatile bool useBufferA = true;

SemaphoreHandle_t dataMutex;

// ---------- BỘ LỌC NOTCH 50Hz ----------
float x1_ = 0, x2_ = 0;
float y1_ = 0, y2_ = 0;
const float b0 = 0.9723, b1 = -1.8478, b2 = 0.9723;
const float a1 = -1.8478, a2 = 0.9446;

float notchFilter(float x)
{
  float y = b0 * x + b1 * x1_ + b2 * x2_ - a1 * y1_ - a2 * y2_;
  x2_ = x1_;
  x1_ = x;
  y2_ = y1_;
  y1_ = y;
  return y;
}

// ---------- TASK 1: Sensor reading ----------
void SensorTask(void *param)
{
  unsigned long lastECG = micros();
  unsigned long lastMPU = micros();
  int ecgIdx = 0, mpuIdx = 0;

  while (true)
  {
    // 250Hz ECG
    if ((int32_t)(micros() - lastECG) >= 1000000 / SAMPLE_RATE)
    {
      lastECG = micros();
      int ecgValue = analogRead(ECG_PIN);
      float V = ((float)ecgValue / 4095.0f) * 3.3f - 1.65f;
      V = notchFilter(notchFilter(V));
      V = int(V * 1000) / 1000;
      xSemaphoreTake(dataMutex, portMAX_DELAY); // giữ khóa

      if (useBufferA)
        ecgBufA[ecgIdx] = V;
      else
        ecgBufB[ecgIdx] = V;
      xSemaphoreGive(dataMutex); // nhả khóa
      ecgIdx++;
    }

    // MPU
    if ((int32_t)(micros() - lastMPU) >= (1000000 / MPU_SAMPLE_RATE))
    {
      lastMPU = micros();
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      if (useBufferA)
      {
        accelX_A[mpuIdx] = a.acceleration.x;
        accelY_A[mpuIdx] = a.acceleration.y;
        accelZ_A[mpuIdx] = a.acceleration.z;
        gyroX_A[mpuIdx] = g.gyro.x;
        gyroY_A[mpuIdx] = g.gyro.y;
        gyroZ_A[mpuIdx] = g.gyro.z;
      }
      else
      {
        accelX_B[mpuIdx] = a.acceleration.x;
        accelY_B[mpuIdx] = a.acceleration.y;
        accelZ_B[mpuIdx] = a.acceleration.z;
        gyroX_B[mpuIdx] = g.gyro.x;
        gyroY_B[mpuIdx] = g.gyro.y;
        gyroZ_B[mpuIdx] = g.gyro.z;
      }
      xSemaphoreGive(dataMutex);
      mpuIdx++;
    }

    // Sau 5s thì đổi buffer
    if (ecgIdx >= NUM_SAMPLES)
    {
      ecgIdx = 0;
      mpuIdx = 0;
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      bufferReadyA = useBufferA;
      useBufferA = !useBufferA;
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(1); // nhả CPU
  }
}

// ---------- TASK 2: Sender HTTP ----------
void SenderTask(void *param)
{

  while (true)
  {
    unsigned long startTime = millis(); // thời điểm bắt đầu chu kỳ

    bool sendA;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    sendA = bufferReadyA;
    bufferReadyA = false;
    xSemaphoreGive(dataMutex);

    // chọn buffer gửi
    float *ecg = sendA ? ecgBufA : ecgBufB;
    float *ax = sendA ? accelX_A : accelX_B;
    float *ay = sendA ? accelY_A : accelY_B;
    float *az = sendA ? accelZ_A : accelZ_B;
    float *gx = sendA ? gyroX_A : gyroX_B;
    float *gy = sendA ? gyroY_A : gyroY_B;
    float *gz = sendA ? gyroZ_A : gyroZ_B;

    int nMPU = MPU_SAMPLE_RATE * DURATION;

    JsonDocument doc;

    JsonArray ecgArr = doc["ecg_signal"].to<JsonArray>();
    for (int i = 0; i < NUM_SAMPLES; i++)
      ecgArr.add(roundf(ecg[i] * 1000) / 1000);;

    JsonObject accel = doc["accel"].to<JsonObject>();
    JsonArray axArr = accel["x"].to<JsonArray>();
    JsonArray ayArr = accel["y"].to<JsonArray>();
    JsonArray azArr = accel["z"].to<JsonArray>();
    for (int i = 0; i < nMPU; i++)
    {
      axArr.add(roundf(ax[i] * 1000) / 1000);
      ayArr.add(roundf(ay[i] * 1000) / 1000);
      azArr.add(roundf(az[i] * 1000) / 1000);
    }

    JsonObject gyro = doc["gyro"].to<JsonObject>();
    JsonArray gxArr = gyro["x"].to<JsonArray>();
    JsonArray gyArr = gyro["y"].to<JsonArray>();
    JsonArray gzArr = gyro["z"].to<JsonArray>();
    for (int i = 0; i < nMPU; i++)
    {
      gxArr.add(roundf(gx[i] * 1000) / 1000);
      gyArr.add(roundf(gy[i] * 1000) / 1000);
      gzArr.add(roundf(gz[i] * 1000) / 1000);
    }

    // === Thêm tần số lấy mẫu ===
    doc["sampling_rate"] = JsonObject();              // Tạo nhóm riêng
    doc["sampling_rate"]["ecg_hz"] = SAMPLE_RATE;     // ECG Hz
    doc["sampling_rate"]["mpu_hz"] = MPU_SAMPLE_RATE; // MPU Hz
    doc["sampling_rate"]["duration"] = DURATION;      // Thêm cả thời lượng batch (5s)

    String payload;
    serializeJson(doc, payload);

    // --- Gửi HTTP ---
    if (WiFi.status() == WL_CONNECTED)
    {
      HTTPClient http;
      http.begin(serverUrl);
      http.addHeader("Content-Type", "application/json");
      int code = http.POST(payload);
      Serial.printf("📡 POST %d bytes -> %d\n", payload.length(), code);
      http.end();
    }
    else
    {
      Serial.println("⚠️ WiFi lost, skipping send");
    }

    // Giải phóng RAM JSON
    doc.clear();

    // --- Tính thời gian gửi xong ---
    unsigned long elapsed = millis() - startTime;
    unsigned long period = DURATION * 1000; // 5000ms

    // Nếu gửi mất <5 s → chờ phần còn lại, nếu >5 s thì bỏ qua delay
    if (elapsed < period)
      vTaskDelay(pdMS_TO_TICKS(period - elapsed));
    else
      Serial.printf("⚠️ Send took %lu ms (> period)\n", elapsed);
  }
}

void setup()
{
  Serial.begin(9600);
  analogReadResolution(12);

  pinMode(SDN_PIN, OUTPUT);
  digitalWrite(SDN_PIN, LOW);

  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected");

  if (!mpu.begin())
  {
    Serial.println("❌ Không tìm thấy MPU6050!");
    while (1)
      delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  dataMutex = xSemaphoreCreateMutex();

  // Tạo 2 task song song
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(SenderTask, "SenderTask", 8192, NULL, 1, NULL, 0);
}

void loop()
{
  vTaskDelay(1000);
}
