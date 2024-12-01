#include <Wire.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Parametry konfiguracyjne
#define MPU_ADDR 0x68 // Adres MPU-6500
#define MOTION_THRESHOLD 3000 // Próg detekcji ruchu
#define GPS_WAIT_TIME 10000 // Czas oczekiwania na dane GPS (w milisekundach)
#define CALIBRATION_SAMPLES 100 // Liczba próbek do kalibracji MPU-6500

// Inicjalizacja obiektu GPS
TinyGPSPlus gps;
HardwareSerial serialGPS(1);  // Użycie UART1 dla GPS

// Inicjalizacja SIM800L
HardwareSerial sim800(2);  // Użycie UART2 dla SIM800L
const int SIM800_RX = 4;  // Pin RX dla SIM800L
const int SIM800_TX = 5;  // Pin TX dla SIM800L
const String phoneNumber = "+48733046468";  // Numer telefonu
const String message = "Hello from ESP32!";  // Treść wiadomości SMS

// Zmienne do kalibracji
int16_t axOffset = 0, ayOffset = 0, azOffset = 0;
int16_t gxOffset = 0, gyOffset = 0, gzOffset = 0;

// Zmienne do przechowywania danych z MPU
int16_t ax, ay, az, gx, gy, gz;

unsigned long previousMillis = 0;
const long interval = 1000; // Interwał 1 sekundy

// Deklaracja funkcji kalibracji i wysyłania SMS
void calibrateMPU();
void sendSMS(String number, String msg);
String readSIM800Response() {
  String response = "";
  while (sim800.available()) {
    response += (char)sim800.read();
  }
  return response;
}

void setup() {
  Serial.begin(115200); // Inicjalizacja monitora szeregowego
  Wire.begin(); // Inicjalizacja I2C

  // Inicjalizacja MPU-6500
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Rejestr PWR_MGMT_1
  Wire.write(0);    // Wysłanie wartości 0 w celu wybudzenia MPU-6500
  Wire.endTransmission(true);

  // Kalibracja MPU-6500
  calibrateMPU();
  
  // Inicjalizacja GPS
  serialGPS.begin(9600, SERIAL_8N1, 16, 17);  // RX = 16, TX = 17
  Serial.println("Czekam na stabilizację...");
  delay(2000); // Poczekaj chwilę, aby odczyty się ustabilizowały

  // Inicjalizacja SIM800L
  sim800.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);
  delay(1000);
  sim800.println("AT+CREG?"); // Sprawdzenie rejestracji w sieci
  delay(1000);
  Serial.println("Odpowiedź AT+CREG?: " + readSIM800Response());
  sim800.println("AT+CGREG?"); // Sprawdzenie rejestracji w sieci GPRS
  delay(1000);
  Serial.println("Odpowiedź AT+CGREG?: " + readSIM800Response());
  sim800.println("AT");  // Sprawdzenie komunikacji
  delay(1000);
  sim800.println("AT+CSCA?");  // Sprawdzenie numeru centrum SMS
  delay(1000);
  sim800.println("AT+CSMP=17,167,0,0");  // Ustawienia formatu wiadomości SMS
  delay(1000);
}

void loop() {
  unsigned long currentMillis = millis();

  // Odczyt danych z MPU-6500
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Rejestr ACCEL_XOUT_H (początek danych)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14); // Odczyt 14 bajtów danych

  if (Wire.available() >= 14) {
    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();

    // Korekcja danych na podstawie kalibracji
    ax -= axOffset;
    ay -= ayOffset;
    az -= azOffset;
    gx -= gxOffset;
    gy -= gyOffset;
    gz -= gzOffset;

    // Wykrywanie ruchu
    int motion = abs(ax) + abs(ay) + abs(az);
    if (motion > MOTION_THRESHOLD) {
      Serial.println("Wykryto ruch!");

      // Czekaj, aż GPS dostarczy dane
      unsigned long gpsStartTime = millis();
      bool gpsDataReceived = false;
      while (millis() - gpsStartTime < GPS_WAIT_TIME) {  // Oczekiwanie na dane GPS
        while (serialGPS.available() > 0) {
          char c = serialGPS.read();
          gps.encode(c);
          if (gps.location.isValid()) {
            Serial.print("Szerokość: ");
            Serial.println(gps.location.lat(), 6);
            Serial.print("Długość: ");
            Serial.println(gps.location.lng(), 6);
            gpsDataReceived = true;
            break;  // Przerywa wewnętrzną pętlę
          }
        }
        if (gpsDataReceived) {
          break;  // Przerywa zewnętrzną pętlę
        }
      }

      if (!gpsDataReceived) {
        Serial.println("Brak danych GPS. Wysyłam SMS...");
        sendSMS(phoneNumber, message);

        // Ponowne sprawdzenie statusu rejestracji w sieci
        sim800.println("AT+CREG?");
        delay(1000);
        Serial.println("Odpowiedź AT+CREG?: " + readSIM800Response());

        sim800.println("AT+CGREG?");
        delay(1000);
        Serial.println("Odpowiedź AT+CGREG?: " + readSIM800Response());
      }

      // Zakończ komunikację GPS przed kolejną próbą
      serialGPS.end();
      delay(100);  // Krótkie opóźnienie przed kolejnym testem
    }
  }

  // Interwał 1 sekundy
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Inne czynności cykliczne, jeśli są potrzebne
  }
}

// Funkcja kalibracji MPU-6500
void calibrateMPU() {
  Serial.println("Kalibracja MPU-6500...");

  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  // Zbieranie danych kalibracyjnych
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Rejestr ACCEL_XOUT_H (początek danych)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14); // Odczyt 14 bajtów danych

    if (Wire.available() >= 14) {
      axSum += (Wire.read() << 8 | Wire.read());
      aySum += (Wire.read() << 8 | Wire.read());
      azSum += (Wire.read() << 8 | Wire.read());
      gxSum += (Wire.read() << 8 | Wire.read());
      gySum += (Wire.read() << 8 | Wire.read());
      gzSum += (Wire.read() << 8 | Wire.read());
    }
    delay(10);  // Krótkie opóźnienie między próbkami
  }

  // Obliczanie średnich wartości
  axOffset = axSum / CALIBRATION_SAMPLES;
  ayOffset = aySum / CALIBRATION_SAMPLES;
  azOffset = azSum / CALIBRATION_SAMPLES;
  gxOffset = gxSum / CALIBRATION_SAMPLES;
  gyOffset = gySum / CALIBRATION_SAMPLES;
  gzOffset = gzSum / CALIBRATION_SAMPLES;

  Serial.print("Kalibracja zakończona. Offsety: ");
  Serial.print("ax: "); Serial.print(axOffset);
  Serial.print(", ay: "); Serial.print(ayOffset);
  Serial.print(", az: "); Serial.println(azOffset);
}

// Funkcja wysyłania SMS
void sendSMS(String number, String msg) {
  Serial.println("Wysyłanie SMS do: " + number);
  Serial.println("Treść wiadomości: " + msg);
  
  sim800.println("AT+CMGF=1");  // Ustawienie trybu tekstowego
  delay(1000);
  
  sim800.println("AT+CMGS=\"" + number + "\"");  // Ustawienie numeru odbiorcy
  delay(1000);

  sim800.print(msg);  // Treść wiadomości
  delay(1000);

  sim800.write(26);  // Zakończenie wiadomości (CTRL+Z)
  delay(3000);  // Czekamy na wysłanie wiadomości

  Serial.println("Wiadomość wysłana.");
}

