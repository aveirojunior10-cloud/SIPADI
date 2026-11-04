
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <RTClib.h>
#include <esp_sleep.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <driver/rtc_io.h>

// =====================================================
// 1. KONFIGURASI PIN
// =====================================================

static const uint8_t TRIG_PIN              = 12;
static const uint8_t ECHO_PIN              = 13;
static const uint8_t PIR_PIN               = 27;
static const uint8_t LED_HIJAU             = 2;
static const uint8_t LED_KUNING            = 4;
static const uint8_t LED_MERAH             = 5;
static const uint8_t BUZZER_PIN            = 25;
static const uint8_t LED_STROBE_PIN        = 18;
static const uint8_t TOMBOL_SUDAH_DISIRAM  = 33;

static const uint8_t I2C_SDA               = 21;
static const uint8_t I2C_SCL               = 22;
static const uint8_t SIM800_RX             = 16;
static const uint8_t SIM800_TX             = 17;

// =====================================================
// 2. KONFIGURASI PENGGUNA
// =====================================================

// Nomor petani dari firmware asli. Ubah jika nomor tujuan berbeda.
static const char NOMOR_PETANI[] = "+6287710189118";

// =====================================================
// 3. KONFIGURASI SENSOR AIR
// =====================================================

static const float JARAK_SENSOR_KE_TANAH_CM = 60.0f;
static const float KOREKSI_TINGGI_AIR_CM    = 0.0f;

static const float JSN_MIN_BACA_CM          = 20.0f;
static const float JSN_MAX_BACA_CM          = 450.0f;
static const uint8_t JUMLAH_SAMPEL_AIR      = 5;
static const uint32_t TIMEOUT_ECHO_US       = 35000UL;
static const uint32_t JEDA_SAMPEL_AIR_MS    = 100UL;

// Ambang klasifikasi dipertahankan dari firmware awal.
static const float BATAS_AIR_CUKUP_CM       = 20.0f;
static const float BATAS_AIR_BERLEBIH_CM    = 25.0f;

// =====================================================
// 4. KONFIGURASI DETEKSI HAMA
// =====================================================

static const uint8_t JUMLAH_SAMPEL_VALIDASI_PIR = 3;
static const uint8_t BATAS_VALIDASI_PIR         = 2;
static const uint32_t JEDA_VALIDASI_PIR_MS      = 700UL;

static const uint32_t DURASI_AKTUATOR_HAMA_MS   = 10000UL;
static const uint32_t COOLDOWN_AKTUATOR_DETIK   = 60UL;
static const float BATAS_SUHU_AKTUATOR_C        = 45.0f;

static const uint32_t JENDELA_HAMA_DETIK        = 3600UL;
static const uint8_t AMBANG_NOTIFIKASI_HAMA     = 3; // Notifikasi saat > 3 deteksi.
static const uint32_t PIR_LOW_STABIL_MS         = 1000UL;

// =====================================================
// 5. KONFIGURASI BUZZER DAN SMS
// =====================================================

static const uint32_t DURASI_BUZZER_AIR_MS      = 5000UL;
static const uint32_t COOLDOWN_SMS_DETIK        = 2UL * 60UL * 60UL;

static const uint8_t JAM_SMS_HARIAN             = 6;
static const uint8_t MENIT_SMS_HARIAN           = 0;

static const uint32_t TIMEOUT_REGISTRASI_GSM_MS = 30000UL;
static const int16_t ZONA_WAKTU_MENIT           = 7 * 60; // WIB UTC+7.

static const uint32_t INTERVAL_POLL_SMS_MS      = 15000UL;
static const uint32_t DURASI_MODE_AKTIF_MS      = 300000UL;

// =====================================================
// 6. KONFIGURASI TOMBOL DAN DEEP SLEEP
// =====================================================

static const uint32_t DEBOUNCE_TOMBOL_MS        = 80UL;
static const uint64_t FAKTOR_DETIK_KE_US        = 1000000ULL;
static const uint64_t DURASI_DEEP_SLEEP_DETIK   = 15ULL * 60ULL;

// =====================================================
// 7. KONFIGURASI EEPROM AT24C32
// =====================================================

static const uint8_t EEPROM_I2C_ADDRESS         = 0x57;
static const uint16_t EEPROM_UKURAN_BYTE        = 4096;
static const uint16_t EEPROM_ALAMAT_MAGIC       = 0;
static const uint16_t EEPROM_ALAMAT_INDEX        = 4;
static const uint16_t EEPROM_ALAMAT_TOTAL        = 6;
static const uint16_t EEPROM_ALAMAT_DATA         = 16;
static const uint8_t EEPROM_UKURAN_RECORD        = 8;
static const uint8_t EEPROM_UKURAN_HALAMAN       = 32;

static const uint16_t EEPROM_KAPASITAS_RECORD =
    (EEPROM_UKURAN_BYTE - EEPROM_ALAMAT_DATA) / EEPROM_UKURAN_RECORD;

static const uint8_t EEPROM_MAGIC[4] = {'S', 'I', 'P', 'A'};
static const uint8_t EEPROM_MARKER_RECORD = 0xA5;

// =====================================================
// 8. OBJEK DAN STATUS GLOBAL
// =====================================================

RTC_DS3231 rtc;
HardwareSerial sim800(2);

enum StatusAir : uint8_t {
  SENSOR_ERROR = 0,
  BERLEBIH     = 1,
  CUKUP        = 2,
  KURANG       = 3
};

enum JenisPeringatanAir : uint8_t {
  TIDAK_ADA_PERINGATAN = 0,
  PERINGATAN_BERLEBIH  = 1,
  PERINGATAN_KURANG    = 2
};

static bool rtcTersedia = false;
static bool eepromTersedia = false;
static bool modemSiap = false;

static DateTime waktuSekarang(2024, 1, 1, 0, 0, 0);
static DateTime waktuKompilasi(__DATE__, __TIME__);
static uint32_t fallbackEpoch = 0;

static float jarakAirTerakhir = NAN;
static float tinggiAirTerakhir = NAN;
static float suhuRTCterakhir = NAN;
static StatusAir statusAirTerakhir = SENSOR_ERROR;

static bool autoSmsSudahDiprosesPadaWake = false;
static bool tombolDibangunkan = false;
static bool pirDibangunkan = false;

// Variabel tombol.
static bool tombolBacaanTerakhir = HIGH;
static bool tombolStatusStabil = HIGH;
static uint32_t tombolWaktuBerubah = 0;

// Variabel polling modem.
static uint32_t waktuPollSMSterakhir = 0;

// Variabel LED sensor error.
static bool ledErrorState = false;
static uint32_t ledErrorWaktuTerakhir = 0;

// Variabel buzzer air.
static bool buzzerAirAktif = false;
static uint32_t buzzerAirBerakhirMs = 0;

// Variabel aktuator hama non-blocking.
static bool aktuatorHamaAktif = false;
static bool polaAktuatorON = false;
static uint32_t aktuatorBerakhirMs = 0;
static uint32_t aktuatorToggleBerikutnyaMs = 0;

// Variabel re-arm PIR.
static uint32_t pirMulaiLOWms = 0;
static uint8_t jumlahValidasiPIRTerakhir = 0;

// RTC memory: dipertahankan selama deep sleep.
RTC_DATA_ATTR uint32_t rtcStateMagic = 0;
RTC_DATA_ATTR bool fiturHamaAktif = true;

RTC_DATA_ATTR int jumlahDeteksiHamaHariIni = 0;
RTC_DATA_ATTR int jumlahDeteksiHamaKemarin = 0;
RTC_DATA_ATTR int jumlahDeteksiHamaDalamJendela = 0;
RTC_DATA_ATTR int jumlahPengairan = 0;

RTC_DATA_ATTR int tanggalCounterHama = -1;
RTC_DATA_ATTR int bulanCounterHama = -1;
RTC_DATA_ATTR int tahunCounterHama = -1;

RTC_DATA_ATTR uint32_t waktuAwalJendelaHama = 0;
RTC_DATA_ATTR uint32_t waktuAktuatorHamaTerakhir = 0;
RTC_DATA_ATTR uint32_t waktuSmsBerlebihTerakhir = 0;
RTC_DATA_ATTR uint32_t waktuSmsKurangTerakhir = 0;
RTC_DATA_ATTR uint32_t waktuSmsHamaTerakhir = 0;

RTC_DATA_ATTR uint8_t peringatanAirTertunda = TIDAK_ADA_PERINGATAN;
RTC_DATA_ATTR bool notifikasiHamaTertunda = false;

RTC_DATA_ATTR int tanggalSmsHarianTerakhir = -1;
RTC_DATA_ATTR int bulanSmsHarianTerakhir = -1;
RTC_DATA_ATTR int tahunSmsHarianTerakhir = -1;

RTC_DATA_ATTR int tanggalSinkronRTCTerakhir = -1;
RTC_DATA_ATTR int bulanSinkronRTCTerakhir = -1;
RTC_DATA_ATTR int tahunSinkronRTCTerakhir = -1;

RTC_DATA_ATTR int statusAirSebelumnya = -1;
RTC_DATA_ATTR bool pirEventLatched = false;
RTC_DATA_ATTR bool tombolWakeSudahDiproses = false;

// =====================================================
// 9. DEKLARASI FUNGSI
// =====================================================

void initPin();
void initRTC();
void initSIM800L();
void inisialisasiStateRTC();
void tampilkanPenyebabBangun(esp_sleep_wakeup_cause_t sebab, uint64_t statusExt1);

DateTime ambilWaktuSekarang();
String formatJamMenit(const DateTime &waktu);
String formatTanggal(const DateTime &waktu);
String formatTanggalJam(const DateTime &waktu);
String namaStatus(StatusAir status);
String rekomendasi(StatusAir status);

bool sinkronRTCdariGSM();
bool tanggalValid(int tahun, int bulan, int tanggal);
uint8_t jumlahHariDalamBulan(int tahun, int bulan);

String kirimATBacaRespon(const String &command, uint32_t timeoutMs);
bool kirimSMS(const String &nomor, const String &pesan);
bool tungguPromptSMS(uint32_t timeoutMs);
void kosongkanBufferModem();
void pollSMSBelumDibaca();
void hapusSMS(int indexSMS);
void prosesSMSMasuk(const String &nomorPengirim, String isiPesan);
String ambilFieldDalamPetik(const String &baris, uint8_t nomorField);
int ambilIndexSMS(const String &header);
String normalisasiNomor(const String &nomor);
bool nomorDiizinkan(const String &nomor);

float bacaJarakSekali();
float bacaJarakMedian();
float hitungTinggiAir(float jarakSensor);
StatusAir tentukanStatusAir(float jarakSensor, float tinggiAir);
void ukurDanPerbaruiAir();
void perbaruiIndikatorAir();
void serviceIndikatorAir();
void matikanSemuaLED();
void evaluasiStatusAir(StatusAir status);
void serviceBuzzerAir();

void serviceTombol();
void catatPengairan(const String &nomorBalasan, bool kirimKonfirmasi);
void kedipHijau(uint8_t jumlahKedip);

void servicePIR();
bool validasiPIRBerulang();
void catatDeteksiHama();
void mulaiAktuatorHama();
void serviceAktuatorHama();
void matikanAktuatorHama();
void matikanSemuaAktuator();

void prosesNotifikasiOtomatis();
bool bolehKirimSMS(uint32_t waktuTerakhir, uint32_t epochSekarang);
bool laporanHarianSudahDikirim();
bool laporanHarianSudahJatuhTempo();
void perbaruiCounterHamaHarian(const DateTime &waktu);
void perbaruiJendelaHama(uint32_t epochSekarang);

String buatSMSBerlebih();
String buatSMSKurang();
String buatSMSHama();
String buatSMSStatusHarian();
String buatSMSStatusManual();
String buatSMSPengairanTercatat();

bool initEEPROMIrigasi();
bool eepromBacaBlok(uint16_t alamat, uint8_t *data, uint8_t panjang);
bool eepromTulisBlok(uint16_t alamat, const uint8_t *data, uint8_t panjang);
bool eepromTungguSiap(uint32_t timeoutMs = 20);
uint16_t eepromBacaUInt16(uint16_t alamat);
bool eepromTulisMetadata(uint16_t indexBerikutnya, uint16_t totalLog);
bool eepromPulihkanMetadata(uint16_t &indexBerikutnya, uint16_t &totalLog);
bool simpanLogPengairanEEPROM(const DateTime &waktu);

void jalankanModeAktif();
void masukDeepSleep();

// =====================================================
// 10. SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  esp_sleep_wakeup_cause_t sebabBangun = esp_sleep_get_wakeup_cause();
  uint64_t statusExt1 = 0;

  if (sebabBangun == ESP_SLEEP_WAKEUP_EXT1) {
    statusExt1 = esp_sleep_get_ext1_wakeup_status();
  }

  tampilkanPenyebabBangun(sebabBangun, statusExt1);
  inisialisasiStateRTC();

  // GPIO RTC dikembalikan ke mode GPIO normal setelah bangun.
  rtc_gpio_deinit((gpio_num_t)TOMBOL_SUDAH_DISIRAM);
  rtc_gpio_deinit((gpio_num_t)PIR_PIN);

  initPin();
  randomSeed((uint32_t)esp_random());

  initRTC();
  initEEPROMIrigasi();
  initSIM800L();

  waktuSekarang = ambilWaktuSekarang();

  if (rtcTersedia) {
    sinkronRTCdariGSM();
    waktuSekarang = ambilWaktuSekarang();
  }

  perbaruiCounterHamaHarian(waktuSekarang);
  perbaruiJendelaHama(waktuSekarang.unixtime());

  // Jika ESP32 bangun karena tombol, catat satu kali saja.
  if (sebabBangun == ESP_SLEEP_WAKEUP_EXT1 &&
      (statusExt1 & (1ULL << TOMBOL_SUDAH_DISIRAM))) {
    tombolDibangunkan = true;
    if (!tombolWakeSudahDiproses) {
      tombolWakeSudahDiproses = true;
      catatPengairan(String(NOMOR_PETANI), true);
    }
    tombolBacaanTerakhir = LOW;
    tombolStatusStabil = LOW;
    tombolWaktuBerubah = millis();
  }

  // Jika bangun karena PIR, latch diaktifkan sebelum validasi.
  if (sebabBangun == ESP_SLEEP_WAKEUP_EXT0) {
    pirDibangunkan = true;
    if (fiturHamaAktif) {
      pirEventLatched = true;
      if (validasiPIRBerulang()) {
        catatDeteksiHama();
      }
    }
  }

  Serial.println();
  Serial.println("==============================================");
  Serial.println("SIPADI FIRMWARE V2.0");
  Serial.println("Monitoring Air | Deteksi Hama | SMS | EEPROM");
  Serial.println("==============================================");

  // Baca SMS yang tertahan di memori SIM setelah boot/wake.
  pollSMSBelumDibaca();

  // Satu pengukuran air pada setiap siklus bangun.
  ukurDanPerbaruiAir();

  // Periksa PIR pada saat boot, termasuk setelah wake timer.
  servicePIR();

  // Prioritas notifikasi otomatis: air > laporan harian > hama.
  prosesNotifikasiOtomatis();

  Serial.println("Memulai mode aktif 5 menit.");
  jalankanModeAktif();

  masukDeepSleep();
}

void loop() {
  // Seluruh siklus kerja dijalankan dari setup().
  // ESP32 masuk deep sleep setelah mode aktif selesai.
  delay(1000);
}

// =====================================================
// 11. INISIALISASI STATE RTC
// =====================================================

void inisialisasiStateRTC() {
  const uint32_t MAGIC_STATE = 0x53495032UL; // "SIP2"

  if (rtcStateMagic == MAGIC_STATE) {
    return;
  }

  Serial.println("State RTC baru/kosong. Menginisialisasi nilai awal.");

  fiturHamaAktif = true;

  jumlahDeteksiHamaHariIni = 0;
  jumlahDeteksiHamaKemarin = 0;
  jumlahDeteksiHamaDalamJendela = 0;
  jumlahPengairan = 0;

  tanggalCounterHama = -1;
  bulanCounterHama = -1;
  tahunCounterHama = -1;

  waktuAwalJendelaHama = 0;
  waktuAktuatorHamaTerakhir = 0;
  waktuSmsBerlebihTerakhir = 0;
  waktuSmsKurangTerakhir = 0;
  waktuSmsHamaTerakhir = 0;

  peringatanAirTertunda = TIDAK_ADA_PERINGATAN;
  notifikasiHamaTertunda = false;

  tanggalSmsHarianTerakhir = -1;
  bulanSmsHarianTerakhir = -1;
  tahunSmsHarianTerakhir = -1;

  tanggalSinkronRTCTerakhir = -1;
  bulanSinkronRTCTerakhir = -1;
  tahunSinkronRTCTerakhir = -1;

  statusAirSebelumnya = -1;
  pirEventLatched = false;
  tombolWakeSudahDiproses = false;

  fallbackEpoch = waktuKompilasi.unixtime();
  rtcStateMagic = MAGIC_STATE;
}

// =====================================================
// 12. INISIALISASI PIN
// =====================================================

void initPin() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);

  pinMode(LED_HIJAU, OUTPUT);
  pinMode(LED_KUNING, OUTPUT);
  pinMode(LED_MERAH, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_STROBE_PIN, OUTPUT);

  pinMode(TOMBOL_SUDAH_DISIRAM, INPUT_PULLUP);

  digitalWrite(TRIG_PIN, LOW);
  matikanSemuaLED();
  matikanSemuaAktuator();

  tombolBacaanTerakhir = digitalRead(TOMBOL_SUDAH_DISIRAM);
  tombolStatusStabil = tombolBacaanTerakhir;
  tombolWaktuBerubah = millis();

  Serial.print("Status tombol awal: ");
  Serial.println(tombolBacaanTerakhir == LOW ? "DITEKAN" : "TIDAK DITEKAN");
}

// =====================================================
// 13. INISIALISASI RTC DAN BUS I2C
// =====================================================

void initRTC() {
  // Pemulihan sederhana jika jalur SDA tertahan LOW.
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(5);

  if (digitalRead(I2C_SDA) == LOW) {
    Serial.println("SDA LOW. Mencoba pemulihan bus I2C.");
    pinMode(I2C_SCL, OUTPUT);
    for (uint8_t i = 0; i < 9; i++) {
      digitalWrite(I2C_SCL, HIGH);
      delayMicroseconds(10);
      digitalWrite(I2C_SCL, LOW);
      delayMicroseconds(10);
    }
    pinMode(I2C_SCL, INPUT_PULLUP);
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  rtcTersedia = rtc.begin(&Wire);

  if (!rtcTersedia) {
    Serial.println("PERINGATAN: RTC DS3231 tidak terdeteksi.");
    Serial.println("Fitur berbasis waktu menggunakan fallback sementara.");
    return;
  }

  if (rtc.lostPower()) {
    Serial.println("RTC kehilangan daya. Mengatur waktu dari waktu kompilasi.");
    rtc.adjust(waktuKompilasi);
  }

  suhuRTCterakhir = rtc.getTemperature();
  Serial.print("RTC aktif: ");
  Serial.println(formatTanggalJam(rtc.now()));
  Serial.print("Suhu RTC: ");
  Serial.print(suhuRTCterakhir, 1);
  Serial.println(" C");
}

DateTime ambilWaktuSekarang() {
  if (rtcTersedia) {
    return rtc.now();
  }

  if (fallbackEpoch == 0) {
    fallbackEpoch = waktuKompilasi.unixtime();
  }

  return DateTime(fallbackEpoch + (millis() / 1000UL));
}

// =====================================================
// 14. INISIALISASI SIM800L
// =====================================================

void initSIM800L() {
  sim800.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);
  delay(1500);

  Serial.println("Inisialisasi SIM800L...");

  modemSiap = false;

  for (uint8_t i = 0; i < 3; i++) {
    String respon = kirimATBacaRespon("AT", 1500);
    if (respon.indexOf("OK") >= 0) {
      modemSiap = true;
      break;
    }
    delay(500);
  }

  if (!modemSiap) {
    Serial.println("PERINGATAN: SIM800L tidak merespons AT.");
    return;
  }

  kirimATBacaRespon("ATE0", 1000);
  kirimATBacaRespon("AT+CMGF=1", 1000);
  kirimATBacaRespon("AT+CSCS=\"GSM\"", 1000);
  kirimATBacaRespon("AT+CPMS=\"SM\",\"SM\",\"SM\"", 1500);
  kirimATBacaRespon("AT+CNMI=2,1,0,0,0", 1000);

  // Memungkinkan modem menerima pembaruan waktu operator.
  kirimATBacaRespon("AT+CLTS=1", 1000);
  kirimATBacaRespon("AT+CTZU=1", 1000);
  kirimATBacaRespon("AT&W", 1500);

  Serial.println("SIM800L merespons dan konfigurasi dasar selesai.");
}

// =====================================================
// 15. FUNGSI AT SIM800L
// =====================================================

void kosongkanBufferModem() {
  while (sim800.available()) {
    sim800.read();
  }
}

String kirimATBacaRespon(const String &command, uint32_t timeoutMs) {
  kosongkanBufferModem();
  sim800.println(command);

  String respon;
  respon.reserve(300);

  uint32_t mulai = millis();

  while (millis() - mulai < timeoutMs) {
    while (sim800.available()) {
      char c = (char)sim800.read();
      respon += c;
    }

    if (respon.indexOf("\nOK") >= 0 ||
        respon.indexOf("\rOK") >= 0 ||
        respon.endsWith("OK") ||
        respon.indexOf("ERROR") >= 0 ||
        respon.indexOf("+CME ERROR") >= 0 ||
        respon.indexOf("+CMS ERROR") >= 0) {
      break;
    }

    delay(2);
  }

  Serial.print("[SIM800 AT] ");
  Serial.println(command);
  if (respon.length() > 0) {
    Serial.println(respon);
  } else {
    Serial.println("(tidak ada respons)");
  }

  return respon;
}

bool tungguPromptSMS(uint32_t timeoutMs) {
  uint32_t mulai = millis();
  String respon;

  while (millis() - mulai < timeoutMs) {
    while (sim800.available()) {
      char c = (char)sim800.read();
      respon += c;

      if (c == '>') {
        return true;
      }
      if (respon.indexOf("ERROR") >= 0) {
        return false;
      }
    }
    delay(2);
  }

  return false;
}

// =====================================================
// 16. PENGIRIMAN SMS DENGAN KONFIRMASI MODEM
// =====================================================

bool kirimSMS(const String &nomor, const String &pesan) {
  if (!modemSiap) {
    Serial.println("SMS batal: modem belum siap.");
    return false;
  }

  String tujuan = normalisasiNomor(nomor);
  if (tujuan.length() < 8) {
    Serial.println("SMS batal: nomor tujuan tidak valid.");
    return false;
  }

  Serial.println();
  Serial.print("Mengirim SMS ke: ");
  Serial.println(tujuan);
  Serial.println(pesan);

  String mode = kirimATBacaRespon("AT+CMGF=1", 1500);
  if (mode.indexOf("OK") < 0) {
    Serial.println("Gagal mengatur mode SMS teks.");
    return false;
  }

  kosongkanBufferModem();
  sim800.print("AT+CMGS=\"");
  sim800.print(tujuan);
  sim800.println("\"");

  if (!tungguPromptSMS(7000)) {
    Serial.println("Gagal memperoleh prompt SMS dari SIM800L.");
    return false;
  }

  sim800.print(pesan);
  sim800.write((uint8_t)26); // CTRL+Z untuk mengirim.

  String respon;
  respon.reserve(250);
  uint32_t mulai = millis();

  while (millis() - mulai < 25000UL) {
    while (sim800.available()) {
      respon += (char)sim800.read();
    }

    if (respon.indexOf("+CMGS:") >= 0 &&
        (respon.indexOf("\nOK") >= 0 || respon.endsWith("OK"))) {
      Serial.println("SMS terkonfirmasi terkirim.");
      return true;
    }

    if (respon.indexOf("+CMS ERROR") >= 0 ||
        respon.indexOf("ERROR") >= 0) {
      Serial.println("SIM800L melaporkan kegagalan pengiriman SMS.");
      Serial.println(respon);
      return false;
    }

    delay(5);
  }

  Serial.println("Timeout menunggu konfirmasi SMS.");
  Serial.println(respon);
  return false;
}

// =====================================================
// 17. PARSER SMS MASUK
// =====================================================

String ambilFieldDalamPetik(const String &baris, uint8_t nomorField) {
  int posisi = 0;

  for (uint8_t field = 0; field <= nomorField; field++) {
    int awal = baris.indexOf('"', posisi);
    if (awal < 0) {
      return "";
    }

    int akhir = baris.indexOf('"', awal + 1);
    if (akhir < 0) {
      return "";
    }

    if (field == nomorField) {
      return baris.substring(awal + 1, akhir);
    }

    posisi = akhir + 1;
  }

  return "";
}

int ambilIndexSMS(const String &header) {
  int titikDua = header.indexOf(':');
  int koma = header.indexOf(',', titikDua + 1);

  if (titikDua < 0 || koma < 0) {
    return -1;
  }

  String nilai = header.substring(titikDua + 1, koma);
  nilai.trim();
  return nilai.toInt();
}

String normalisasiNomor(const String &nomor) {
  String digit;

  for (uint16_t i = 0; i < nomor.length(); i++) {
    char c = nomor.charAt(i);
    if (c >= '0' && c <= '9') {
      digit += c;
    }
  }

  if (digit.startsWith("0")) {
    digit = "62" + digit.substring(1);
  }

  if (digit.startsWith("62")) {
    return "+" + digit;
  }

  if (digit.length() > 0) {
    return "+" + digit;
  }

  return "";
}

bool nomorDiizinkan(const String &nomor) {
  return normalisasiNomor(nomor) == normalisasiNomor(String(NOMOR_PETANI));
}

void hapusSMS(int indexSMS) {
  if (indexSMS < 0 || !modemSiap) {
    return;
  }

  String command = "AT+CMGD=" + String(indexSMS);
  String respon = kirimATBacaRespon(command, 2000);

  if (respon.indexOf("OK") >= 0) {
    Serial.print("SMS index ");
    Serial.print(indexSMS);
    Serial.println(" dihapus dari memori SIM.");
  } else {
    Serial.print("Gagal menghapus SMS index ");
    Serial.println(indexSMS);
  }
}

void pollSMSBelumDibaca() {
  if (!modemSiap) {
    return;
  }

  Serial.println("Memeriksa SMS belum dibaca...");
  String respon = kirimATBacaRespon("AT+CMGL=\"REC UNREAD\"", 8000);

  int posisi = 0;
  bool adaPesan = false;

  int indexSaatIni = -1;
  String nomorSaatIni = "";
  String isiSaatIni = "";
  bool sedangBacaIsi = false;

  while (posisi < respon.length()) {
    int akhirBaris = respon.indexOf('\n', posisi);
    if (akhirBaris < 0) {
      akhirBaris = respon.length();
    }

    String baris = respon.substring(posisi, akhirBaris);
    posisi = akhirBaris + 1;
    baris.replace("\r", "");
    baris.trim();

    if (baris.length() == 0) {
      continue;
    }

    if (baris.startsWith("+CMGL:")) {
      // Selesaikan pesan sebelumnya sebelum membaca header baru.
      if (indexSaatIni >= 0 && isiSaatIni.length() > 0) {
        prosesSMSMasuk(nomorSaatIni, isiSaatIni);
        hapusSMS(indexSaatIni);
        adaPesan = true;
      }

      indexSaatIni = ambilIndexSMS(baris);
      nomorSaatIni = ambilFieldDalamPetik(baris, 1);
      isiSaatIni = "";
      sedangBacaIsi = true;
      continue;
    }

    if (baris == "OK" || baris == "ERROR") {
      continue;
    }

    if (sedangBacaIsi && indexSaatIni >= 0) {
      if (isiSaatIni.length() > 0) {
        isiSaatIni += " ";
      }
      isiSaatIni += baris;
    }
  }

  // Selesaikan pesan terakhir.
  if (indexSaatIni >= 0 && isiSaatIni.length() > 0) {
    prosesSMSMasuk(nomorSaatIni, isiSaatIni);
    hapusSMS(indexSaatIni);
    adaPesan = true;
  }

  Serial.println(adaPesan ? "Pemrosesan SMS selesai." : "Tidak ada SMS baru.");
}

void prosesSMSMasuk(const String &nomorPengirim, String isiPesan) {
  String nomor = normalisasiNomor(nomorPengirim);
  isiPesan.trim();
  isiPesan.toUpperCase();

  Serial.println();
  Serial.println("SMS masuk:");
  Serial.print("Nomor: ");
  Serial.println(nomor);
  Serial.print("Isi  : ");
  Serial.println(isiPesan);

  // Semua perintah kontrol hanya diterima dari nomor petani terdaftar.
  if (!nomorDiizinkan(nomor)) {
    Serial.println("SMS diabaikan: nomor pengirim tidak berwenang.");
    return;
  }

  waktuSekarang = ambilWaktuSekarang();

  if (isiPesan == "STATUS") {
    ukurDanPerbaruiAir();
    String balasan = buatSMSStatusManual();
    kirimSMS(nomor, balasan);
  }
  else if (isiPesan == "HAMA ON") {
    fiturHamaAktif = true;
    pirEventLatched = false;
    pirMulaiLOWms = 0;

    String balasan = "FITUR DETEKSI HAMA AKTIF\n\n";
    balasan += "Sistem akan memantau aktivitas gerakan melalui sensor PIR.\n";
    balasan += "Notifikasi dikirim jika deteksi berulang memenuhi ambang.";
    kirimSMS(nomor, balasan);
  }
  else if (isiPesan == "HAMA OFF") {
    fiturHamaAktif = false;
    notifikasiHamaTertunda = false;
    jumlahDeteksiHamaDalamJendela = 0;
    pirEventLatched = false;
    pirMulaiLOWms = 0;
    matikanAktuatorHama();

    String balasan = "FITUR DETEKSI HAMA DINONAKTIFKAN.\n\n";
    balasan += "Monitoring air dan notifikasi SMS tetap berjalan.";
    kirimSMS(nomor, balasan);
  }
  else if (isiPesan == "SUDAH DISIRAM" || isiPesan == "RESET AIR") {
    catatPengairan(nomor, true);
  }
  else {
    String balasan = "PERINTAH TIDAK DIKENALI.\n\n";
    balasan += "Gunakan:\n";
    balasan += "STATUS\n";
    balasan += "HAMA ON\n";
    balasan += "HAMA OFF\n";
    balasan += "SUDAH DISIRAM\n";
    balasan += "RESET AIR";
    kirimSMS(nomor, balasan);
  }
}

// =====================================================
// 18. SINKRONISASI RTC DARI JARINGAN GSM
// =====================================================

uint8_t jumlahHariDalamBulan(int tahun, int bulan) {
  static const uint8_t hari[] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
  };

  if (bulan < 1 || bulan > 12) {
    return 0;
  }

  if (bulan == 2) {
    bool kabisat = ((tahun % 4 == 0 && tahun % 100 != 0) || (tahun % 400 == 0));
    return kabisat ? 29 : 28;
  }

  return hari[bulan - 1];
}

bool tanggalValid(int tahun, int bulan, int tanggal) {
  if (tahun < 2024 || tahun > 2099 || bulan < 1 || bulan > 12) {
    return false;
  }

  uint8_t maksimum = jumlahHariDalamBulan(tahun, bulan);
  return tanggal >= 1 && tanggal <= maksimum;
}

bool sinkronRTCdariGSM() {
  if (!rtcTersedia || !modemSiap) {
    Serial.println("Sinkronisasi RTC dilewati: RTC/modem tidak tersedia.");
    return false;
  }

  DateTime sekarang = rtc.now();

  bool sudahSinkronHariIni =
      sekarang.day() == tanggalSinkronRTCTerakhir &&
      sekarang.month() == bulanSinkronRTCTerakhir &&
      sekarang.year() == tahunSinkronRTCTerakhir;

  if (sudahSinkronHariIni) {
    Serial.println("RTC sudah disinkronkan pada tanggal ini.");
    return true;
  }

  Serial.println("Menunggu registrasi jaringan GSM untuk sinkronisasi RTC...");

  bool jaringanSiap = false;
  uint32_t mulai = millis();

  while (millis() - mulai < TIMEOUT_REGISTRASI_GSM_MS) {
    String respon = kirimATBacaRespon("AT+CREG?", 1500);
    int p = respon.indexOf("+CREG:");

    if (p >= 0) {
      int koma = respon.indexOf(',', p);
      int status = -1;

      if (koma >= 0) {
        int akhir = respon.indexOf('\n', koma);
        String bagian = respon.substring(koma + 1, akhir < 0 ? respon.length() : akhir);
        bagian.trim();
        status = bagian.toInt();
      }

      if (status == 1 || status == 5) {
        jaringanSiap = true;
        break;
      }
    }

    delay(1000);
  }

  if (!jaringanSiap) {
    Serial.println("Jaringan GSM belum terdaftar. RTC memakai waktu sebelumnya.");
    return false;
  }

  for (uint8_t percobaan = 0; percobaan < 5; percobaan++) {
    String respon = kirimATBacaRespon("AT+CCLK?", 2000);

    int posisi = respon.indexOf("+CCLK:");
    int petikAwal = respon.indexOf('"', posisi);
    int petikAkhir = respon.indexOf('"', petikAwal + 1);

    if (posisi >= 0 && petikAwal >= 0 && petikAkhir > petikAwal) {
      String nilai = respon.substring(petikAwal + 1, petikAkhir);

      if (nilai.length() >= 17) {
        int tahun = 2000 + nilai.substring(0, 2).toInt();
        int bulan = nilai.substring(3, 5).toInt();
        int tanggal = nilai.substring(6, 8).toInt();
        int jam = nilai.substring(9, 11).toInt();
        int menit = nilai.substring(12, 14).toInt();
        int detik = nilai.substring(15, 17).toInt();

        bool waktuValid =
            tanggalValid(tahun, bulan, tanggal) &&
            jam >= 0 && jam <= 23 &&
            menit >= 0 && menit <= 59 &&
            detik >= 0 && detik <= 59;

        if (waktuValid) {
          int offsetJaringanMenit = ZONA_WAKTU_MENIT;

          // Format offset SIM800L menggunakan satuan 15 menit.
          if (nilai.length() >= 20 &&
              (nilai.charAt(17) == '+' || nilai.charAt(17) == '-')) {
            int offset15Menit = nilai.substring(18).toInt();
            offsetJaringanMenit = offset15Menit * 15;

            if (nilai.charAt(17) == '-') {
              offsetJaringanMenit = -offsetJaringanMenit;
            }
          }

          DateTime waktuJaringan(tahun, bulan, tanggal, jam, menit, detik);

          int64_t epochUTC =
              (int64_t)waktuJaringan.unixtime() -
              ((int64_t)offsetJaringanMenit * 60LL);

          int64_t epochTarget =
              epochUTC + ((int64_t)ZONA_WAKTU_MENIT * 60LL);

          if (epochTarget > 0 && epochTarget <= 0xFFFFFFFFLL) {
            DateTime hasil((uint32_t)epochTarget);
            rtc.adjust(hasil);

            tanggalSinkronRTCTerakhir = hasil.day();
            bulanSinkronRTCTerakhir = hasil.month();
            tahunSinkronRTCTerakhir = hasil.year();

            Serial.print("RTC berhasil disinkronkan: ");
            Serial.println(formatTanggalJam(hasil));
            return true;
          }
        }
      }
    }

    delay(1500);
  }

  Serial.println("Gagal membaca waktu valid dari SIM800L.");
  return false;
}

// =====================================================
// 19. SENSOR AIR: PEMBACAAN DAN FILTER MEDIAN
// =====================================================

float bacaJarakSekali() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long durasi = pulseIn(ECHO_PIN, HIGH, TIMEOUT_ECHO_US);

  if (durasi == 0) {
    return NAN;
  }

  float jarak = (float)durasi * 0.0343f / 2.0f;

  // Blind zone JSN-SR04T pada perangkat ini berada sekitar 20-25 cm.
  // Gunakan 25 cm sebagai batas minimum konservatif agar data zona mati
  // tidak dipakai untuk menghitung tinggi air.
  if (jarak < JSN_MIN_BACA_CM || jarak > JSN_MAX_BACA_CM) {
    return NAN;
  }

  return jarak;
}

float bacaJarakMedian() {
  float sampel[JUMLAH_SAMPEL_AIR];
  uint8_t jumlahValid = 0;

  for (uint8_t i = 0; i < JUMLAH_SAMPEL_AIR; i++) {
    float hasil = bacaJarakSekali();

    if (!isnan(hasil)) {
      sampel[jumlahValid++] = hasil;
    }

    if (i < JUMLAH_SAMPEL_AIR - 1) {
      delay(JEDA_SAMPEL_AIR_MS);
    }
  }

  if (jumlahValid == 0) {
    return NAN;
  }

  // Sorting sederhana untuk memperoleh median.
  for (uint8_t i = 0; i < jumlahValid - 1; i++) {
    for (uint8_t j = i + 1; j < jumlahValid; j++) {
      if (sampel[j] < sampel[i]) {
        float temp = sampel[i];
        sampel[i] = sampel[j];
        sampel[j] = temp;
      }
    }
  }

  return sampel[jumlahValid / 2];
}

float hitungTinggiAir(float jarakSensor) {
  if (isnan(jarakSensor)) {
    return NAN;
  }

  float tinggi =
      JARAK_SENSOR_KE_TANAH_CM -
      jarakSensor +
      KOREKSI_TINGGI_AIR_CM;

  if (tinggi < 0.0f) {
    tinggi = 0.0f;
  }

  return tinggi;
}

StatusAir tentukanStatusAir(float jarakSensor, float tinggiAir) {
  if (isnan(jarakSensor) || isnan(tinggiAir)) {
    return SENSOR_ERROR;
  }

  if (tinggiAir >= BATAS_AIR_BERLEBIH_CM) {
    return BERLEBIH;
  }

  if (tinggiAir >= BATAS_AIR_CUKUP_CM) {
    return CUKUP;
  }

  return KURANG;
}

// =====================================================
// 20. PEMBARUAN KONDISI AIR DAN INDIKATOR
// =====================================================

void ukurDanPerbaruiAir() {
  waktuSekarang = ambilWaktuSekarang();

  jarakAirTerakhir = bacaJarakMedian();
  tinggiAirTerakhir = hitungTinggiAir(jarakAirTerakhir);
  statusAirTerakhir = tentukanStatusAir(jarakAirTerakhir, tinggiAirTerakhir);
  suhuRTCterakhir = rtcTersedia ? rtc.getTemperature() : NAN;

  evaluasiStatusAir(statusAirTerakhir);
  perbaruiIndikatorAir();

  Serial.println();
  Serial.println("------------- MONITORING AIR -------------");
  Serial.print("Waktu       : ");
  Serial.println(formatTanggalJam(waktuSekarang));

  if (isnan(jarakAirTerakhir)) {
    Serial.println("Jarak       : SENSOR ERROR");
    Serial.println("Tinggi air  : Tidak tersedia");
  } else {
    Serial.print("Jarak       : ");
    Serial.print(jarakAirTerakhir, 1);
    Serial.println(" cm");

    Serial.print("Tinggi air  : ");
    Serial.print(tinggiAirTerakhir, 1);
    Serial.println(" cm");
  }

  Serial.print("Status      : ");
  Serial.println(namaStatus(statusAirTerakhir));
  Serial.print("Rekomendasi : ");
  Serial.println(rekomendasi(statusAirTerakhir));
  Serial.println("------------------------------------------");
}

void evaluasiStatusAir(StatusAir status) {
  uint32_t epoch = ambilWaktuSekarang().unixtime();

  if (status == SENSOR_ERROR) {
    return;
  }

  if (status == CUKUP) {
    // Kondisi aman mereset cooldown dan antrean peringatan air.
    waktuSmsBerlebihTerakhir = 0;
    waktuSmsKurangTerakhir = 0;
    peringatanAirTertunda = TIDAK_ADA_PERINGATAN;
  }
  else if (status == BERLEBIH) {
    peringatanAirTertunda =
        bolehKirimSMS(waktuSmsBerlebihTerakhir, epoch)
        ? PERINGATAN_BERLEBIH
        : TIDAK_ADA_PERINGATAN;
  }
  else if (status == KURANG) {
    peringatanAirTertunda =
        bolehKirimSMS(waktuSmsKurangTerakhir, epoch)
        ? PERINGATAN_KURANG
        : TIDAK_ADA_PERINGATAN;

    if (statusAirSebelumnya != KURANG) {
      buzzerAirAktif = true;
      buzzerAirBerakhirMs = millis() + DURASI_BUZZER_AIR_MS;
      Serial.println("Buzzer peringatan air kurang aktif selama 5 detik.");
    }
  }

  statusAirSebelumnya = (int)status;
}

void perbaruiIndikatorAir() {
  ledErrorState = false;
  ledErrorWaktuTerakhir = millis();

  if (statusAirTerakhir == CUKUP) {
    digitalWrite(LED_HIJAU, HIGH);
    digitalWrite(LED_KUNING, LOW);
    digitalWrite(LED_MERAH, LOW);
  }
  else if (statusAirTerakhir == BERLEBIH) {
    digitalWrite(LED_HIJAU, LOW);
    digitalWrite(LED_KUNING, HIGH);
    digitalWrite(LED_MERAH, LOW);
  }
  else if (statusAirTerakhir == KURANG) {
    digitalWrite(LED_HIJAU, LOW);
    digitalWrite(LED_KUNING, LOW);
    digitalWrite(LED_MERAH, HIGH);
  }
  else {
    ledErrorState = true;
    ledErrorWaktuTerakhir = millis();
    digitalWrite(LED_HIJAU, LOW);
    digitalWrite(LED_KUNING, LOW);
    digitalWrite(LED_MERAH, LOW);
  }
}

void serviceIndikatorAir() {
  if (!ledErrorState) {
    return;
  }

  if (millis() - ledErrorWaktuTerakhir >= 200UL) {
    ledErrorWaktuTerakhir = millis();
    bool nyala = digitalRead(LED_HIJAU) == LOW;

    digitalWrite(LED_HIJAU, nyala ? HIGH : LOW);
    digitalWrite(LED_KUNING, nyala ? HIGH : LOW);
    digitalWrite(LED_MERAH, nyala ? HIGH : LOW);
  }
}

void matikanSemuaLED() {
  digitalWrite(LED_HIJAU, LOW);
  digitalWrite(LED_KUNING, LOW);
  digitalWrite(LED_MERAH, LOW);
}

String namaStatus(StatusAir status) {
  switch (status) {
    case BERLEBIH: return "BERLEBIH";
    case CUKUP: return "CUKUP";
    case KURANG: return "KURANG";
    default: return "SENSOR ERROR";
  }
}

String rekomendasi(StatusAir status) {
  switch (status) {
    case BERLEBIH:
      return "Air berlebih. Kurangi atau hentikan pengairan sesuai kebutuhan tanaman.";
    case CUKUP:
      return "Air cukup. Pertahankan kondisi pengairan.";
    case KURANG:
      return "Air kurang. Lakukan pengairan sesuai kebutuhan tanaman.";
    default:
      return "Periksa posisi sensor, kabel, dan rentang pembacaan sensor.";
  }
}

// =====================================================
// 21. BUZZER AIR
// =====================================================

void serviceBuzzerAir() {
  if (buzzerAirAktif && (int32_t)(millis() - buzzerAirBerakhirMs) >= 0) {
    buzzerAirAktif = false;
  }

  // Buzzer air diprioritaskan; aktuator hama tetap mengendalikan strobo.
  bool buzzerHama = aktuatorHamaAktif && polaAktuatorON;
  digitalWrite(BUZZER_PIN, (buzzerAirAktif || buzzerHama) ? HIGH : LOW);
}

// =====================================================
// 22. PUSH BUTTON DAN PENCATATAN PENGAIRAN
// =====================================================

void serviceTombol() {
  bool bacaan = digitalRead(TOMBOL_SUDAH_DISIRAM);

  if (bacaan != tombolBacaanTerakhir) {
    tombolBacaanTerakhir = bacaan;
    tombolWaktuBerubah = millis();
  }

  if (millis() - tombolWaktuBerubah < DEBOUNCE_TOMBOL_MS) {
    return;
  }

  if (bacaan != tombolStatusStabil) {
    tombolStatusStabil = bacaan;

    if (tombolStatusStabil == LOW) {
      if (!tombolWakeSudahDiproses) {
        tombolWakeSudahDiproses = true;
        Serial.println("Push button SUDAH DISIRAM ditekan.");
        catatPengairan(String(NOMOR_PETANI), true);
      }
    } else {
      // Tombol sudah dilepas; izinkan event berikutnya.
      tombolWakeSudahDiproses = false;
    }
  }
}

void catatPengairan(const String &nomorBalasan, bool kirimKonfirmasi) {
  waktuSekarang = ambilWaktuSekarang();

  jumlahPengairan++;

  // Reset cooldown dan antrean peringatan air sesuai fungsi awal.
  waktuSmsBerlebihTerakhir = 0;
  waktuSmsKurangTerakhir = 0;
  peringatanAirTertunda = TIDAK_ADA_PERINGATAN;

  bool tersimpan = simpanLogPengairanEEPROM(waktuSekarang);

  Serial.println();
  Serial.println("========== PENGAIRAN TERCATAT ==========");
  Serial.print("Waktu: ");
  Serial.println(formatTanggalJam(waktuSekarang));
  Serial.print("Counter pengairan: ");
  Serial.println(jumlahPengairan);
  Serial.print("EEPROM: ");
  Serial.println(tersimpan ? "BERHASIL" : "GAGAL/TIDAK TERSEDIA");
  Serial.println("Cooldown peringatan air direset.");
  Serial.println("========================================");

  kedipHijau(3);

  if (kirimKonfirmasi) {
    String pesan = buatSMSPengairanTercatat();
    if (!kirimSMS(nomorBalasan, pesan)) {
      Serial.println("Konfirmasi pengairan gagal dikirim.");
    }
  }
}

void kedipHijau(uint8_t jumlahKedip) {
  // Feedback singkat untuk pencatatan pengairan.
  matikanSemuaLED();

  for (uint8_t i = 0; i < jumlahKedip; i++) {
    digitalWrite(LED_HIJAU, HIGH);
    delay(180);
    digitalWrite(LED_HIJAU, LOW);
    delay(180);
  }

  perbaruiIndikatorAir();
}

// =====================================================
// 23. DETEKSI PIR DAN EVENT LATCH
// =====================================================

bool validasiPIRBerulang() {
  uint8_t jumlahHigh = 0;

  for (uint8_t i = 0; i < JUMLAH_SAMPEL_VALIDASI_PIR; i++) {
    if (digitalRead(PIR_PIN) == HIGH) {
      jumlahHigh++;
    }

    if (i < JUMLAH_SAMPEL_VALIDASI_PIR - 1) {
      delay(JEDA_VALIDASI_PIR_MS);
    }
  }

  jumlahValidasiPIRTerakhir = jumlahHigh;

  Serial.print("Validasi PIR: ");
  Serial.print(jumlahHigh);
  Serial.print("/");
  Serial.println(JUMLAH_SAMPEL_VALIDASI_PIR);

  return jumlahHigh >= BATAS_VALIDASI_PIR;
}

void servicePIR() {
  if (!fiturHamaAktif) {
    matikanAktuatorHama();
    return;
  }

  bool pirHigh = digitalRead(PIR_PIN) == HIGH;

  if (!pirHigh) {
    if (pirEventLatched) {
      if (pirMulaiLOWms == 0) {
        pirMulaiLOWms = millis();
      }

      if (millis() - pirMulaiLOWms >= PIR_LOW_STABIL_MS) {
        pirEventLatched = false;
        pirMulaiLOWms = 0;
        Serial.println("PIR LOW stabil. Deteksi siap diaktifkan kembali.");
      }
    } else {
      pirMulaiLOWms = 0;
    }

    return;
  }

  pirMulaiLOWms = 0;

  // Satu sinyal HIGH hanya boleh menghasilkan satu event sampai PIR LOW stabil.
  if (pirEventLatched) {
    return;
  }

  pirEventLatched = true;

  Serial.println("PIR HIGH: memulai validasi gerakan.");
  if (validasiPIRBerulang()) {
    catatDeteksiHama();
  } else {
    Serial.println("Gerakan tidak memenuhi validasi; event tetap dilatch sampai PIR LOW.");
  }
}

void catatDeteksiHama() {
  waktuSekarang = ambilWaktuSekarang();
  uint32_t epoch = waktuSekarang.unixtime();

  perbaruiJendelaHama(epoch);

  jumlahDeteksiHamaHariIni++;
  jumlahDeteksiHamaDalamJendela++;

  Serial.println("AKTIVITAS GERAKAN TERVALIDASI.");
  Serial.print("Deteksi hari ini: ");
  Serial.println(jumlahDeteksiHamaHariIni);
  Serial.print("Deteksi dalam jendela: ");
  Serial.println(jumlahDeteksiHamaDalamJendela);

  bool cooldownSelesai =
      waktuAktuatorHamaTerakhir == 0 ||
      epoch < waktuAktuatorHamaTerakhir ||
      (epoch - waktuAktuatorHamaTerakhir >= COOLDOWN_AKTUATOR_DETIK);

  if (cooldownSelesai) {
    suhuRTCterakhir = rtcTersedia ? rtc.getTemperature() : NAN;

    // Jika suhu tidak dapat dibaca, aktuator ditahan dalam kondisi aman.
    if (isnan(suhuRTCterakhir)) {
      Serial.println("Aktuator tidak aktif: suhu RTC tidak tersedia.");
    }
    else if (suhuRTCterakhir >= BATAS_SUHU_AKTUATOR_C) {
      Serial.println("Aktuator tidak aktif: suhu RTC mencapai/melebihi 45 C.");
    }
    else {
      mulaiAktuatorHama();
      waktuAktuatorHamaTerakhir = epoch;
    }
  } else {
    Serial.println("Aktuator hama masih dalam cooldown 60 detik.");
  }

  if (jumlahDeteksiHamaDalamJendela > AMBANG_NOTIFIKASI_HAMA) {
    notifikasiHamaTertunda = true;
    Serial.println("Ambang notifikasi hama tercapai; SMS masuk antrean.");
  }
}

// =====================================================
// 24. AKTUATOR HAMA NON-BLOCKING
// =====================================================

void mulaiAktuatorHama() {
  if (aktuatorHamaAktif) {
    return;
  }

  aktuatorHamaAktif = true;
  polaAktuatorON = true;
  aktuatorBerakhirMs = millis() + DURASI_AKTUATOR_HAMA_MS;
  aktuatorToggleBerikutnyaMs = millis() + (uint32_t)random(150, 700);

  digitalWrite(LED_STROBE_PIN, HIGH);
  serviceBuzzerAir();

  Serial.println("Aktuator pengusir hama aktif maksimal 10 detik.");
}

void serviceAktuatorHama() {
  if (!aktuatorHamaAktif) {
    serviceBuzzerAir();
    return;
  }

  uint32_t sekarang = millis();

  if ((int32_t)(sekarang - aktuatorBerakhirMs) >= 0) {
    matikanAktuatorHama();
    return;
  }

  if ((int32_t)(sekarang - aktuatorToggleBerikutnyaMs) >= 0) {
    polaAktuatorON = !polaAktuatorON;

    digitalWrite(LED_STROBE_PIN, polaAktuatorON ? HIGH : LOW);

    uint32_t jeda = polaAktuatorON
        ? (uint32_t)random(150, 700)
        : (uint32_t)random(120, 500);

    aktuatorToggleBerikutnyaMs = sekarang + jeda;
  }

  serviceBuzzerAir();
}

void matikanAktuatorHama() {
  aktuatorHamaAktif = false;
  polaAktuatorON = false;
  digitalWrite(LED_STROBE_PIN, LOW);
  serviceBuzzerAir();
}

void matikanSemuaAktuator() {
  aktuatorHamaAktif = false;
  polaAktuatorON = false;
  buzzerAirAktif = false;

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_STROBE_PIN, LOW);
}

// =====================================================
// 25. MANAJEMEN JENDELA DAN COUNTER HAMA
// =====================================================

void perbaruiJendelaHama(uint32_t epochSekarang) {
  if (waktuAwalJendelaHama == 0 ||
      epochSekarang < waktuAwalJendelaHama ||
      (epochSekarang - waktuAwalJendelaHama >= JENDELA_HAMA_DETIK)) {

    if (!notifikasiHamaTertunda) {
      jumlahDeteksiHamaDalamJendela = 0;
      waktuAwalJendelaHama = epochSekarang;
    }
  }
}

void perbaruiCounterHamaHarian(const DateTime &waktu) {
  if (tanggalCounterHama < 0 ||
      bulanCounterHama < 0 ||
      tahunCounterHama < 0) {
    tanggalCounterHama = waktu.day();
    bulanCounterHama = waktu.month();
    tahunCounterHama = waktu.year();
    return;
  }

  bool tanggalBerubah =
      tanggalCounterHama != waktu.day() ||
      bulanCounterHama != waktu.month() ||
      tahunCounterHama != waktu.year();

  if (!tanggalBerubah) {
    return;
  }

  jumlahDeteksiHamaKemarin = jumlahDeteksiHamaHariIni;
  jumlahDeteksiHamaHariIni = 0;

  tanggalCounterHama = waktu.day();
  bulanCounterHama = waktu.month();
  tahunCounterHama = waktu.year();

  Serial.print("Rekap deteksi hama hari sebelumnya: ");
  Serial.println(jumlahDeteksiHamaKemarin);
}

// =====================================================
// 26. COOLDOWN DAN NOTIFIKASI OTOMATIS
// =====================================================

bool bolehKirimSMS(uint32_t waktuTerakhir, uint32_t epochSekarang) {
  if (waktuTerakhir == 0) {
    return true;
  }

  if (epochSekarang < waktuTerakhir) {
    // RTC mundur/sinkronisasi waktu: izinkan pengiriman berikutnya.
    return true;
  }

  return (epochSekarang - waktuTerakhir) >= COOLDOWN_SMS_DETIK;
}

bool laporanHarianSudahDikirim() {
  return tanggalSmsHarianTerakhir == waktuSekarang.day() &&
         bulanSmsHarianTerakhir == waktuSekarang.month() &&
         tahunSmsHarianTerakhir == waktuSekarang.year();
}

bool laporanHarianSudahJatuhTempo() {
  if (waktuSekarang.hour() > JAM_SMS_HARIAN) {
    return !laporanHarianSudahDikirim();
  }

  if (waktuSekarang.hour() == JAM_SMS_HARIAN &&
      waktuSekarang.minute() >= MENIT_SMS_HARIAN) {
    return !laporanHarianSudahDikirim();
  }

  return false;
}

void prosesNotifikasiOtomatis() {
  if (autoSmsSudahDiprosesPadaWake || !modemSiap) {
    return;
  }

  waktuSekarang = ambilWaktuSekarang();
  uint32_t epoch = waktuSekarang.unixtime();

  // PRIORITAS 1: PERINGATAN AIR.
  if (peringatanAirTertunda != TIDAK_ADA_PERINGATAN) {
    // Jangan mengirim alert lama dengan angka yang tidak valid ketika sensor error.
    if (statusAirTerakhir == SENSOR_ERROR) {
      Serial.println("Peringatan air menunggu pembacaan sensor yang valid.");
      return;
    }

    String pesan;
    bool cooldownValid = false;
    uint32_t *waktuTerakhir = nullptr;

    if (peringatanAirTertunda == PERINGATAN_BERLEBIH) {
      waktuTerakhir = (uint32_t *)&waktuSmsBerlebihTerakhir;
      cooldownValid = bolehKirimSMS(waktuSmsBerlebihTerakhir, epoch);
      pesan = buatSMSBerlebih();
    } else if (peringatanAirTertunda == PERINGATAN_KURANG) {
      waktuTerakhir = (uint32_t *)&waktuSmsKurangTerakhir;
      cooldownValid = bolehKirimSMS(waktuSmsKurangTerakhir, epoch);
      pesan = buatSMSKurang();
    }

    if (cooldownValid && waktuTerakhir != nullptr) {
      autoSmsSudahDiprosesPadaWake = true;

      if (kirimSMS(String(NOMOR_PETANI), pesan)) {
        *waktuTerakhir = epoch;
        peringatanAirTertunda = TIDAK_ADA_PERINGATAN;
      } else {
        Serial.println("Peringatan air tetap tertunda untuk percobaan berikutnya.");
      }

      return;
    }
  }

  // PRIORITAS 2: LAPORAN HARIAN.
  if (laporanHarianSudahJatuhTempo()) {
    autoSmsSudahDiprosesPadaWake = true;

    if (kirimSMS(String(NOMOR_PETANI), buatSMSStatusHarian())) {
      tanggalSmsHarianTerakhir = waktuSekarang.day();
      bulanSmsHarianTerakhir = waktuSekarang.month();
      tahunSmsHarianTerakhir = waktuSekarang.year();
      Serial.println("Laporan harian berhasil dikirim.");
    } else {
      Serial.println("Laporan harian belum terkirim; akan dicoba pada wake berikutnya.");
    }

    return;
  }

  // PRIORITAS 3: NOTIFIKASI HAMA.
  if (fiturHamaAktif && notifikasiHamaTertunda) {
    if (!bolehKirimSMS(waktuSmsHamaTerakhir, epoch)) {
      Serial.println("Notifikasi hama menunggu cooldown SMS.");
      return;
    }

    autoSmsSudahDiprosesPadaWake = true;

    if (kirimSMS(String(NOMOR_PETANI), buatSMSHama())) {
      waktuSmsHamaTerakhir = epoch;
      jumlahDeteksiHamaDalamJendela = 0;
      waktuAwalJendelaHama = epoch;
      notifikasiHamaTertunda = false;
      Serial.println("Notifikasi aktivitas hama berhasil dikirim.");
    } else {
      Serial.println("Notifikasi hama tetap tertunda untuk wake berikutnya.");
    }
  }
}

// =====================================================
// 27. FORMAT DAN TEMPLATE SMS
// =====================================================

String formatJamMenit(const DateTime &waktu) {
  char buffer[6];
  snprintf(buffer, sizeof(buffer), "%02d:%02d",
           (int)waktu.hour(), (int)waktu.minute());
  return String(buffer);
}

String formatTanggal(const DateTime &waktu) {
  char buffer[12];
  snprintf(buffer, sizeof(buffer), "%02d/%02d/%04d",
           (int)waktu.day(), (int)waktu.month(), (int)waktu.year());
  return String(buffer);
}

String formatTanggalJam(const DateTime &waktu) {
  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%02d/%02d/%04d %02d:%02d:%02d",
           (int)waktu.day(), (int)waktu.month(), (int)waktu.year(),
           (int)waktu.hour(), (int)waktu.minute(), (int)waktu.second());
  return String(buffer);
}

String buatSMSBerlebih() {
  String pesan = "STATUS AIR BERLEBIH\n\n";
  pesan += "Tinggi air: ";
  if (isnan(tinggiAirTerakhir)) {
    pesan += "ERROR";
  } else {
    pesan += String(tinggiAirTerakhir, 1);
    pesan += " cm";
  }
  pesan += "\nWaktu: ";
  pesan += formatTanggalJam(ambilWaktuSekarang());
  pesan += "\n\nPeriksa kondisi lahan dan kurangi pengairan bila diperlukan.";
  return pesan;
}

String buatSMSKurang() {
  String pesan = "STATUS AIR KURANG\n\n";
  pesan += "Tinggi air: ";
  if (isnan(tinggiAirTerakhir)) {
    pesan += "ERROR";
  } else {
    pesan += String(tinggiAirTerakhir, 1);
    pesan += " cm";
  }
  pesan += "\nWaktu: ";
  pesan += formatTanggalJam(ambilWaktuSekarang());
  pesan += "\n\nLakukan pengairan sesuai kebutuhan tanaman.";
  return pesan;
}

String buatSMSHama() {
  String pesan = "PERINGATAN AKTIVITAS GERAKAN\n\n";
  pesan += "Sensor PIR mendeteksi gerakan berulang.\n";
  pesan += "Jumlah deteksi: ";
  pesan += String(jumlahDeteksiHamaDalamJendela);
  pesan += " kali dalam jendela 1 jam.\n";
  pesan += "Waktu: ";
  pesan += formatTanggalJam(ambilWaktuSekarang());
  pesan += "\n\nPeriksa kondisi sawah. PIR mendeteksi gerakan dan tidak mengidentifikasi jenis hewan.";
  return pesan;
}

String buatSMSStatusHarian() {
  waktuSekarang = ambilWaktuSekarang();

  String pesan = "STATUS HARIAN SIPADI\n\n";

  if (isnan(tinggiAirTerakhir)) {
    pesan += "Tinggi air: ERROR\n";
    pesan += "Status air: SENSOR ERROR\n";
  } else {
    pesan += "Tinggi air: ";
    pesan += String(tinggiAirTerakhir, 1);
    pesan += " cm\n";
    pesan += "Status air: ";
    pesan += namaStatus(statusAirTerakhir);
    pesan += "\n";
  }

  pesan += "Fitur hama: ";
  pesan += fiturHamaAktif ? "AKTIF\n" : "NONAKTIF\n";

  pesan += "Deteksi hama kemarin: ";
  pesan += String(jumlahDeteksiHamaKemarin);
  pesan += " kali\n";

  pesan += "Pengairan tercatat: ";
  pesan += String(jumlahPengairan);
  pesan += " kali\n";

  pesan += "Suhu RTC: ";
  pesan += isnan(suhuRTCterakhir) ? "TIDAK TERSEDIA\n" : String(suhuRTCterakhir, 1) + " C\n";

  pesan += "Waktu: ";
  pesan += formatTanggalJam(waktuSekarang);
  return pesan;
}

String buatSMSStatusManual() {
  waktuSekarang = ambilWaktuSekarang();

  String pesan = "STATUS SIPADI\n\n";

  if (isnan(tinggiAirTerakhir)) {
    pesan += "Tinggi air: ERROR\n";
    pesan += "Status air: SENSOR ERROR\n";
  } else {
    pesan += "Tinggi air: ";
    pesan += String(tinggiAirTerakhir, 1);
    pesan += " cm\n";
    pesan += "Status air: ";
    pesan += namaStatus(statusAirTerakhir);
    pesan += "\n";
  }

  pesan += "Fitur hama: ";
  pesan += fiturHamaAktif ? "AKTIF\n" : "NONAKTIF\n";

  pesan += "Deteksi hama hari ini: ";
  pesan += String(jumlahDeteksiHamaHariIni);
  pesan += " kali\n";

  pesan += "Pengairan tercatat: ";
  pesan += String(jumlahPengairan);
  pesan += " kali\n";

  pesan += "Suhu RTC: ";
  pesan += isnan(suhuRTCterakhir) ? "TIDAK TERSEDIA\n" : String(suhuRTCterakhir, 1) + " C\n";

  pesan += "Waktu: ";
  pesan += formatTanggalJam(waktuSekarang);
  pesan += "\n\n";
  pesan += rekomendasi(statusAirTerakhir);

  return pesan;
}

String buatSMSPengairanTercatat() {
  String pesan = "PENGAIRAN TERCATAT\n\n";
  pesan += "Aktivitas pengairan telah dicatat.\n";
  pesan += "Waktu: ";
  pesan += formatTanggalJam(ambilWaktuSekarang());
  pesan += "\n";
  pesan += "Total pengairan: ";
  pesan += String(jumlahPengairan);
  pesan += "\nCooldown peringatan air direset.";
  return pesan;
}

// =====================================================
// 28. EEPROM AT24C32
// =====================================================

bool eepromTungguSiap(uint32_t timeoutMs) {
  uint32_t mulai = millis();

  while (millis() - mulai < timeoutMs) {
    Wire.beginTransmission(EEPROM_I2C_ADDRESS);
    if (Wire.endTransmission() == 0) {
      return true;
    }
    delay(1);
  }

  return false;
}

bool eepromBacaBlok(uint16_t alamat, uint8_t *data, uint8_t panjang) {
  if (!eepromTersedia || panjang == 0 ||
      (uint32_t)alamat + panjang > EEPROM_UKURAN_BYTE) {
    return false;
  }

  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((uint8_t)(alamat >> 8));
  Wire.write((uint8_t)(alamat & 0xFF));

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t diterima = Wire.requestFrom((int)EEPROM_I2C_ADDRESS, (int)panjang);
  if (diterima != panjang) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < panjang; i++) {
    if (!Wire.available()) {
      return false;
    }
    data[i] = (uint8_t)Wire.read();
  }

  return true;
}

bool eepromTulisBlok(uint16_t alamat, const uint8_t *data, uint8_t panjang) {
  if (!eepromTersedia || panjang == 0 ||
      (uint32_t)alamat + panjang > EEPROM_UKURAN_BYTE) {
    return false;
  }

  uint8_t sudahDitulis = 0;

  while (sudahDitulis < panjang) {
    uint16_t alamatSekarang = alamat + sudahDitulis;
    uint8_t sisaHalaman = EEPROM_UKURAN_HALAMAN -
                          (alamatSekarang % EEPROM_UKURAN_HALAMAN);
    uint8_t sisaData = panjang - sudahDitulis;
    uint8_t jumlah = min((uint8_t)30, min(sisaHalaman, sisaData));

    if (!eepromTungguSiap()) {
      return false;
    }

    Wire.beginTransmission(EEPROM_I2C_ADDRESS);
    Wire.write((uint8_t)(alamatSekarang >> 8));
    Wire.write((uint8_t)(alamatSekarang & 0xFF));

    for (uint8_t i = 0; i < jumlah; i++) {
      Wire.write(data[sudahDitulis + i]);
    }

    if (Wire.endTransmission() != 0) {
      return false;
    }

    if (!eepromTungguSiap(30)) {
      return false;
    }

    sudahDitulis += jumlah;
  }

  return true;
}

uint16_t eepromBacaUInt16(uint16_t alamat) {
  uint8_t data[2] = {0, 0};

  if (!eepromBacaBlok(alamat, data, 2)) {
    return 0;
  }

  return ((uint16_t)data[0] << 8) | data[1];
}

bool eepromTulisMetadata(uint16_t indexBerikutnya, uint16_t totalLog) {
  uint8_t metadata[4] = {
    (uint8_t)(indexBerikutnya >> 8),
    (uint8_t)(indexBerikutnya & 0xFF),
    (uint8_t)(totalLog >> 8),
    (uint8_t)(totalLog & 0xFF)
  };

  return eepromTulisBlok(EEPROM_ALAMAT_INDEX, metadata, 4);
}

bool eepromPulihkanMetadata(uint16_t &indexBerikutnya, uint16_t &totalLog) {
  bool ditemukan = false;
  uint32_t epochTerbaru = 0;
  uint16_t indexTerbaru = 0;
  uint16_t jumlahValid = 0;

  for (uint16_t i = 0; i < EEPROM_KAPASITAS_RECORD; i++) {
    uint16_t alamat = EEPROM_ALAMAT_DATA + i * EEPROM_UKURAN_RECORD;
    uint8_t record[EEPROM_UKURAN_RECORD];

    if (!eepromBacaBlok(alamat, record, EEPROM_UKURAN_RECORD)) {
      continue;
    }

    if (record[0] != EEPROM_MARKER_RECORD) {
      continue;
    }

    int tahun = ((int)record[1] << 8) | record[2];
    int bulan = record[3];
    int tanggal = record[4];
    int jam = record[5];
    int menit = record[6];
    int detik = record[7];

    if (!tanggalValid(tahun, bulan, tanggal) ||
        jam > 23 || menit > 59 || detik > 59) {
      continue;
    }

    DateTime waktuRecord(tahun, bulan, tanggal, jam, menit, detik);
    uint32_t epoch = waktuRecord.unixtime();
    jumlahValid++;

    if (!ditemukan || epoch > epochTerbaru) {
      ditemukan = true;
      epochTerbaru = epoch;
      indexTerbaru = i;
    }
  }

  if (!ditemukan) {
    indexBerikutnya = 0;
    totalLog = 0;
  } else {
    indexBerikutnya = (indexTerbaru + 1) % EEPROM_KAPASITAS_RECORD;
    totalLog = jumlahValid;
  }

  Serial.println("Metadata EEPROM dipulihkan melalui pemeriksaan record.");
  return eepromTulisMetadata(indexBerikutnya, totalLog);
}

bool initEEPROMIrigasi() {
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  if (Wire.endTransmission() != 0) {
    eepromTersedia = false;
    Serial.println("EEPROM AT24C32 alamat 0x57 tidak terdeteksi.");
    return false;
  }

  eepromTersedia = true;

  uint8_t magicTerbaca[4] = {0, 0, 0, 0};
  bool magicValid = eepromBacaBlok(EEPROM_ALAMAT_MAGIC, magicTerbaca, 4);

  if (magicValid) {
    for (uint8_t i = 0; i < 4; i++) {
      if (magicTerbaca[i] != EEPROM_MAGIC[i]) {
        magicValid = false;
        break;
      }
    }
  }

  if (!magicValid) {
    Serial.println("Magic EEPROM tidak valid. Menyiapkan header baru.");

    if (!eepromTulisBlok(EEPROM_ALAMAT_MAGIC, EEPROM_MAGIC, 4)) {
      eepromTersedia = false;
      return false;
    }

    uint16_t index = 0;
    uint16_t total = 0;
    if (!eepromPulihkanMetadata(index, total)) {
      eepromTersedia = false;
      return false;
    }
  }

  uint16_t indexBerikutnya = eepromBacaUInt16(EEPROM_ALAMAT_INDEX);
  uint16_t totalLog = eepromBacaUInt16(EEPROM_ALAMAT_TOTAL);

  if (indexBerikutnya >= EEPROM_KAPASITAS_RECORD) {
    Serial.println("Metadata pointer EEPROM tidak valid; melakukan pemulihan.");
    if (!eepromPulihkanMetadata(indexBerikutnya, totalLog)) {
      eepromTersedia = false;
      return false;
    }
  }

  jumlahPengairan = totalLog;

  Serial.print("EEPROM siap. Record tersimpan: ");
  Serial.println(totalLog);
  Serial.print("Index record berikutnya: ");
  Serial.println(indexBerikutnya);

  return true;
}

bool simpanLogPengairanEEPROM(const DateTime &waktu) {
  if (!eepromTersedia) {
    return false;
  }

  uint16_t indexBerikutnya = eepromBacaUInt16(EEPROM_ALAMAT_INDEX);
  uint16_t totalLog = eepromBacaUInt16(EEPROM_ALAMAT_TOTAL);

  if (indexBerikutnya >= EEPROM_KAPASITAS_RECORD) {
    return false;
  }

  uint16_t alamatRecord =
      EEPROM_ALAMAT_DATA + indexBerikutnya * EEPROM_UKURAN_RECORD;

  uint16_t tahun = waktu.year();

  uint8_t record[EEPROM_UKURAN_RECORD] = {
    EEPROM_MARKER_RECORD,
    (uint8_t)(tahun >> 8),
    (uint8_t)(tahun & 0xFF),
    (uint8_t)waktu.month(),
    (uint8_t)waktu.day(),
    (uint8_t)waktu.hour(),
    (uint8_t)waktu.minute(),
    (uint8_t)waktu.second()
  };

  if (!eepromTulisBlok(alamatRecord, record, EEPROM_UKURAN_RECORD)) {
    return false;
  }

  indexBerikutnya = (indexBerikutnya + 1) % EEPROM_KAPASITAS_RECORD;

  // jumlahPengairan sudah dinaikkan sebelum fungsi ini dipanggil.
  // Gunakan nilai terbesar agar metadata EEPROM tidak menurunkan counter RTC.
  if (jumlahPengairan > (int)totalLog) {
    totalLog = (jumlahPengairan > 65535) ? 65535 : (uint16_t)jumlahPengairan;
  } else if (totalLog < 65535) {
    totalLog++;
  }

  if (!eepromTulisMetadata(indexBerikutnya, totalLog)) {
    Serial.println("Record tersimpan, tetapi metadata EEPROM gagal diperbarui.");
    return false;
  }

  jumlahPengairan = totalLog;
  return true;
}

// =====================================================
// 29. MODE AKTIF 5 MENIT
// =====================================================

void jalankanModeAktif() {
  uint32_t mulai = millis();
  waktuPollSMSterakhir = millis();

  while (millis() - mulai < DURASI_MODE_AKTIF_MS) {
    serviceTombol();
    servicePIR();
    serviceAktuatorHama();
    serviceBuzzerAir();
    serviceIndikatorAir();

    if (millis() - waktuPollSMSterakhir >= INTERVAL_POLL_SMS_MS) {
      waktuPollSMSterakhir = millis();
      pollSMSBelumDibaca();

      // Maksimal satu notifikasi otomatis per satu siklus wake.
      prosesNotifikasiOtomatis();
    }

    delay(20);
  }

  Serial.println("Mode aktif 5 menit selesai.");
}

// =====================================================
// 30. PENYEBAB WAKE-UP
// =====================================================

void tampilkanPenyebabBangun(esp_sleep_wakeup_cause_t sebab, uint64_t statusExt1) {
  Serial.print("Penyebab ESP32 bangun: ");

  switch (sebab) {
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Timer deep sleep");
      break;

    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("PIR GPIO 27");
      break;

    case ESP_SLEEP_WAKEUP_EXT1:
      if (statusExt1 & (1ULL << TOMBOL_SUDAH_DISIRAM)) {
        Serial.println("Push button GPIO 33");
      } else {
        Serial.println("Sumber EXT1 lainnya");
      }
      break;

    case ESP_SLEEP_WAKEUP_UNDEFINED:
      Serial.println("Boot normal/reset manual");
      break;

    default:
      Serial.println("Sumber wake-up lainnya");
      break;
  }
}

// =====================================================
// 31. PERSIAPAN DAN DEEP SLEEP
// =====================================================

void masukDeepSleep() {
  Serial.println();
  Serial.println("Menyiapkan SIPADI untuk deep sleep...");

  matikanSemuaAktuator();
  matikanSemuaLED();

  // Jika RTC tidak tersedia, pertahankan estimasi waktu antar-wake.
  if (!rtcTersedia) {
    DateTime sekarang = ambilWaktuSekarang();
    fallbackEpoch = sekarang.unixtime() + DURASI_DEEP_SLEEP_DETIK;
  }

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  // Wake-up timer setiap 15 menit.
  esp_sleep_enable_timer_wakeup(
      DURASI_DEEP_SLEEP_DETIK * FAKTOR_DETIK_KE_US);

  // Tombol GPIO 33 aktif LOW. Hindari wake loop jika tombol masih ditahan.
  if (digitalRead(TOMBOL_SUDAH_DISIRAM) == HIGH &&
      !tombolWakeSudahDiproses) {
    rtc_gpio_pullup_en((gpio_num_t)TOMBOL_SUDAH_DISIRAM);
    rtc_gpio_pulldown_dis((gpio_num_t)TOMBOL_SUDAH_DISIRAM);
    esp_sleep_enable_ext1_wakeup(
        1ULL << TOMBOL_SUDAH_DISIRAM,
        ESP_EXT1_WAKEUP_ALL_LOW);
  } else {
    Serial.println("Wake-up tombol dinonaktifkan sementara karena tombol masih LOW/ditahan.");
  }

  // PIR hanya menjadi sumber wake-up jika fitur aktif dan event sudah re-arm.
  if (fiturHamaAktif && !pirEventLatched) {
    rtc_gpio_pulldown_en((gpio_num_t)PIR_PIN);
    rtc_gpio_pullup_dis((gpio_num_t)PIR_PIN);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIR_PIN, 1);
  } else {
    Serial.println("Wake-up PIR ditahan sampai sensor kembali LOW stabil.");
  }

  // Mempertahankan RTC peripheral untuk pull-up/pull-down RTC GPIO.
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  Serial.print("Deep sleep selama ");
  Serial.print(DURASI_DEEP_SLEEP_DETIK);
  Serial.println(" detik.");
  Serial.println("SIPADI masuk deep sleep sekarang.");
  Serial.flush();

  delay(100);
  esp_deep_sleep_start();
}
