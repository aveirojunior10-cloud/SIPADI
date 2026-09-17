#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "RTClib.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#define TRIG_PIN 12
#define ECHO_PIN 13
#define PIR_PIN 27
#define LED_HIJAU 2
#define LED_KUNING 4
#define LED_MERAH 5
#define BUZZER_PIN 25
#define LED_STROBE_1 18
#define TOMBOL_SUDAH_DISIRAM 33
#define I2C_SDA 21
#define I2C_SCL 22
#define SIM800_RX 16
#define SIM800_TX 17

RTC_DS3231 rtc;
HardwareSerial sim800(2);
String NOMOR_PETANI = "+6287710189118";
// =====================================================
// KONFIGURASI SENSOR AIR
// =====================================================
/*
  Ukur langsung dari permukaan depan kepala JSN-SR04T
  sampai permukaan tanah sawah yang menjadi titik 0 cm.

  Nilai 60.0 cm digunakan berdasarkan rencana pemasangan.
  Ganti nilai ini sesuai hasil pengukuran aktual setelah
  sensor terpasang, misalnya 55.0 jika jaraknya 55 cm.
*/
const float JARAK_SENSOR_KE_TANAH_CM = 55.0;

/*
  Koreksi hasil kalibrasi. Biarkan 0.0 jika hasil sensor
  sudah sama dengan pengukuran menggunakan penggaris.
  Nilai positif menambah hasil tinggi air, sedangkan
  nilai negatif menguranginya.
*/
const float KOREKSI_TINGGI_AIR_CM = 0.0;

const float JSN_MIN_BACA_CM = 20.0;
const float JSN_MAX_BACA_CM = 450.0;
const int JUMLAH_SAMPEL = 5;
const unsigned long TIMEOUT_ECHO = 35000;
// =====================================================
// KONFIGURASI HAMA
// =====================================================
const unsigned long DURASI_AKTUATOR_HAMA = 10000;
const int JUMLAH_SAMPEL_VALIDASI_PIR = 3;
const int BATAS_VALIDASI_PIR = 2;
const unsigned long JEDA_SAMPEL_VALIDASI_PIR_MS = 700;
const unsigned long COOLDOWN_AKTUATOR_HAMA_DETIK = 60;
const float BATAS_SUHU_TINGGI_C = 45.0;

// Buzzer kondisi air kritis menggunakan pin buzzer yang sama.
const unsigned long DURASI_BUZZER_AIR_KRITIS_MS = 5000;
// =====================================================
// KONFIGURASI SMS
// =====================================================
// Cooldown per level peringatan sesuai dokumen: 2 jam.
const unsigned long COOLDOWN_SMS_DETIK = 2UL * 60UL * 60UL;
const unsigned long JENDELA_HAMA_DETIK = 3600;
// =====================================================
// SMS HARIAN
// =====================================================
const int JAM_SMS_HARIAN = 6;
const int MENIT_SMS_HARIAN_AWAL = 0;
const int MENIT_SMS_HARIAN_AKHIR = 10;
// =====================================================
// SINKRONISASI RTC DARI JARINGAN GSM
// =====================================================
// WIB = UTC+7. Ubah menjadi 8 * 60 untuk WITA atau 9 * 60 untuk WIT.
const int ZONA_WAKTU_MENIT = 7 * 60;
const unsigned long TIMEOUT_REGISTRASI_GSM_MS = 30000;
// =====================================================
// PUSH BUTTON
// =====================================================
const unsigned long DEBOUNCE_TOMBOL_MS = 80;
// =====================================================
// EEPROM AT24C32 PADA MODUL DS3231
// =====================================================
const uint8_t EEPROM_I2C_ADDRESS = 0x57;
const uint16_t EEPROM_UKURAN_BYTE = 4096;
const uint16_t EEPROM_ALAMAT_MAGIC = 0;
const uint16_t EEPROM_ALAMAT_INDEX_BERIKUTNYA = 4;
const uint16_t EEPROM_ALAMAT_TOTAL_LOG = 6;
const uint16_t EEPROM_ALAMAT_DATA = 16;
const uint8_t EEPROM_UKURAN_RECORD = 8;
const uint16_t EEPROM_KAPASITAS_RECORD =
    (EEPROM_UKURAN_BYTE - EEPROM_ALAMAT_DATA) / EEPROM_UKURAN_RECORD;
const uint8_t EEPROM_MAGIC[4] = {'S', 'I', 'P', 'A'};
// =====================================================
// DEEP SLEEP
// =====================================================
const uint64_t FAKTOR_DETIK_KE_MIKRODETIK = 1000000ULL;
const uint64_t DURASI_DEEP_SLEEP_DETIK = 15 * 60;
const unsigned long DURASI_MODE_AKTIF_SMS_MS = 300000;
// =====================================================
// RTC MEMORY
// =====================================================
RTC_DATA_ATTR bool fiturHamaAktif = true;
RTC_DATA_ATTR int jumlahDeteksiHama = 0;
RTC_DATA_ATTR int jumlahDeteksiHamaDalamJendela = 0;
RTC_DATA_ATTR int jumlahDeteksiPIRValidasi = 0;
RTC_DATA_ATTR int jumlahDeteksiHamaKemarin = 0;
RTC_DATA_ATTR int tanggalCounterHama = -1;
RTC_DATA_ATTR int bulanCounterHama = -1;
RTC_DATA_ATTR int tahunCounterHama = -1;
RTC_DATA_ATTR int jumlahPengairan = 0;
RTC_DATA_ATTR unsigned long waktuAwalJendelaHama = 0;
RTC_DATA_ATTR unsigned long waktuAktuatorHamaTerakhir = 0;
RTC_DATA_ATTR unsigned long waktuSmsPeringatanTerakhir = 0;
RTC_DATA_ATTR unsigned long waktuSmsKurangTerakhir = 0;
RTC_DATA_ATTR unsigned long waktuSmsHamaTerakhir = 0;
// TRUE jika ambang deteksi hama sudah tercapai, tetapi SMS hama
// masih menunggu karena SMS ketinggian air sedang diprioritaskan.
RTC_DATA_ATTR bool notifikasiHamaTertunda = false;
RTC_DATA_ATTR int tanggalSmsHarianTerakhir = -1;
RTC_DATA_ATTR int bulanSmsHarianTerakhir = -1;
RTC_DATA_ATTR int tahunSmsHarianTerakhir = -1;

// Penanda agar sinkronisasi RTC hanya dilakukan sekali per hari.
// Variabel RTC_DATA_ATTR tetap tersimpan selama deep sleep.
RTC_DATA_ATTR int tanggalSinkronRTCTerakhir = -1;
RTC_DATA_ATTR int bulanSinkronRTCTerakhir = -1;
RTC_DATA_ATTR int tahunSinkronRTCTerakhir = -1;
// =====================================================
// VARIABEL PUSH BUTTON
// =====================================================
bool statusTombolTerakhir = HIGH;
bool statusTombolStabil = HIGH;
unsigned long waktuDebounceTombol = 0;
/*
  TRUE jika ESP32 bangun dari deep sleep
  karena tombol GPIO 33.
*/
bool bangunDariTombol = false;
// =====================================================
// VARIABEL SIM800L
// =====================================================
String bufferSIM = "";
bool menungguIsiSMS = false;
String nomorPengirimTerakhir = "";
// =====================================================
// VARIABEL DATA
// =====================================================
DateTime waktuSekarang;
float dataJarakTerakhir = 0;
float dataTinggiAirTerakhir = 0;
bool eepromIrigasiTersedia = false;
// =====================================================
// STATUS AIR
// =====================================================
enum StatusAir { SENSOR_ERROR, BERLEBIH, CUKUP, KURANG };
StatusAir statusAirTerakhir = SENSOR_ERROR;

// =====================================================
// DEKLARASI FUNGSI
// Wajib ada jika file dipakai sebagai .cpp atau PlatformIO.
// Arduino IDE pada file .ino biasanya membuat ini otomatis,
// tetapi pada .cpp tidak.
// =====================================================
void initPin();
void initRTC();
void initSIM800L();
String kirimATBacaRespon(String command, unsigned long timeout);
bool sinkronRTC();
void kirimCommandSIM(String command, unsigned long timeout);
bool kirimSMS(String nomor, String pesan);
void cekSMSMasuk();
void prosesBarisSIM(String baris);
String ambilNomorDariHeaderSMS(String header);
int ambilIndexDariCMGL(String header);
int ambilIndexDariCMTI(String baris);
void bacaSMSDariIndex(int indexSMS);
void bacaSMSBelumDibacaDariMemori();
void hapusSMS(int indexSMS);
void modeAktifTerimaSMS();
void prosesPerintahSMS(String nomorPengirim, String pesan);
float bacaJarakSekali();
float bacaJarakMedian();
float hitungTinggiAir(float jarakSensor);
StatusAir tentukanStatusAir(float jarakSensor, float tinggiAir);
String namaStatus(StatusAir status);
String rekomendasi(StatusAir status);
void updateLEDStatus(StatusAir status);
void matikanSemuaLED();
bool prosesSMSAir(StatusAir status, float tinggiAir);
bool bolehKirimSMS(unsigned long waktuSmsTerakhir, unsigned long epochSekarang);
void resetCooldownSMSJikaAirAman(StatusAir status);
void aktifkanBuzzerAirKritis(StatusAir status);
bool prosesSMSHarian(float tinggiAir, StatusAir status);
void perbaruiCounterHamaHarian(DateTime waktu);
void cekTombolSudahDisiram();
void prosesTombolSudahDisiram();
void kedipHijau(int jumlahKedip);
bool validasiPIRBerulang();
bool prosesDeteksiHama();
bool prosesSMSHama();
void prosesJendelaHama();
void aktifkanAktuatorHama();
void matikanAktuatorHama();
String buatSMSBerlebih(float tinggiAir);
String buatSMSKurang(float tinggiAir);
String buatSMSHama();
String buatSMSStatusHarian(float tinggiAir, StatusAir status);
String buatSMSPengairanTercatat();
String buatSMSStatusManual();
String formatJamMenit(DateTime waktu);
String formatTanggal(DateTime waktu);
String formatTanggalJam(DateTime waktu);
bool initEEPROMIrigasi();
bool eepromBacaBlok(uint16_t alamat, uint8_t *data, uint8_t panjang);
bool eepromTulisBlok(uint16_t alamat, const uint8_t *data, uint8_t panjang);
uint16_t eepromBacaUInt16(uint16_t alamat);
bool eepromTulisMetadata(uint16_t indexBerikutnya, uint16_t totalLog);
bool simpanLogPengairanEEPROM(DateTime waktu);
void tampilkanHasil(float jarakSensor, float tinggiAir, StatusAir status, bool hamaTerdeteksi);
void tampilkanPenyebabBangun();
void masukDeepSleep();

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  /*
    Cek dulu penyebab ESP32 bangun.
  */
  tampilkanPenyebabBangun();
  /*
    GPIO 33 digunakan sebagai RTC GPIO
    ketika deep sleep.
    Setelah bangun, kembalikan ke GPIO digital.
  */
  rtc_gpio_deinit((gpio_num_t)TOMBOL_SUDAH_DISIRAM);
  initPin();
  randomSeed(micros());
  initRTC();
  initEEPROMIrigasi();
  initSIM800L();

  // Ambil waktu lokal dari jaringan operator dan simpan ke DS3231.
  // Jika gagal, RTC tetap menggunakan waktu sebelumnya.
  sinkronRTC();

  waktuSekarang = rtc.now();
  perbaruiCounterHamaHarian(waktuSekarang);
  if (waktuAwalJendelaHama == 0) {
    waktuAwalJendelaHama = waktuSekarang.unixtime();
  }
  /*
    Jika bangun karena push button,
    langsung catat pengairan.
  */
  if (bangunDariTombol) {
    Serial.println();
    Serial.println("ESP32 BANGUN KARENA PUSH BUTTON");
    prosesTombolSudahDisiram();
    /*
      Sinkronisasi tombol supaya satu tekan
      tidak dihitung dua kali.
    */
    statusTombolTerakhir = digitalRead(TOMBOL_SUDAH_DISIRAM);
    statusTombolStabil = statusTombolTerakhir;
    waktuDebounceTombol = millis();
  }
  bacaSMSBelumDibacaDariMemori();
  Serial.println();
  Serial.println("=================================");
  Serial.println("SIPADI");
  Serial.println("Monitoring Air, Hama, RTC, SMS, Deep Sleep");
  Serial.println("=================================");
  Serial.print("Nomor petani: ");
  Serial.println(NOMOR_PETANI);
  Serial.println("Perintah SMS: STATUS, HAMA ON, HAMA OFF, SUDAH DISIRAM");
  Serial.println("RTC: sinkron otomatis dari jaringan GSM sekali per hari");
  Serial.println("=================================");
}
// =====================================================
// LOOP
// =====================================================
void loop() {
  waktuSekarang = rtc.now();
  perbaruiCounterHamaHarian(waktuSekarang);
  cekSMSMasuk();
  float jarakSensor = bacaJarakMedian();
  float tinggiAir = hitungTinggiAir(jarakSensor);
  StatusAir status = tentukanStatusAir(jarakSensor, tinggiAir);
  dataJarakTerakhir = jarakSensor;
  dataTinggiAirTerakhir = tinggiAir;
  statusAirTerakhir = status;
  updateLEDStatus(status);
  resetCooldownSMSJikaAirAman(status);
  aktifkanBuzzerAirKritis(status);
  /*
    Cek push button saat loop utama.
  */
  cekTombolSudahDisiram();
  /*
    PRIORITAS SMS OTOMATIS:
    1. Peringatan ketinggian air
    2. Status harian yang memuat ketinggian air
    3. Peringatan hama

    Maksimal satu jenis SMS otomatis diproses pada satu siklus bangun.
    Jika SMS air diprioritaskan, SMS hama disimpan sebagai tertunda dan
    dicoba kembali pada siklus berikutnya.
  */
  bool smsOtomatisSudahDiprioritaskan = prosesSMSAir(status, tinggiAir);

  bool hamaTerdeteksi = prosesDeteksiHama();
  prosesJendelaHama();

  if (!smsOtomatisSudahDiprioritaskan) {
    smsOtomatisSudahDiprioritaskan = prosesSMSHarian(tinggiAir, status);
  }

  if (!smsOtomatisSudahDiprioritaskan) {
    prosesSMSHama();
  }
  tampilkanHasil(jarakSensor, tinggiAir, status, hamaTerdeteksi);
  cekSMSMasuk();
  /*
    Selama mode aktif ini tombol juga
    terus diperiksa.
  */
  modeAktifTerimaSMS();
  masukDeepSleep();
}
// =====================================================
// INISIALISASI PIN
// =====================================================
void initPin() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_HIJAU, OUTPUT);
  pinMode(LED_KUNING, OUTPUT);
  pinMode(LED_MERAH, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_STROBE_1, OUTPUT);
  /*
    PUSH BUTTON
    Wiring:
    GPIO 33
       |
    PUSH BUTTON
       |
      GND
    Tidak ditekan = HIGH
    Ditekan       = LOW
  */
  pinMode(TOMBOL_SUDAH_DISIRAM, INPUT_PULLUP);
  /*
    Sinkronisasi status tombol saat boot.
  */
  statusTombolTerakhir = digitalRead(TOMBOL_SUDAH_DISIRAM);
  statusTombolStabil = statusTombolTerakhir;
  digitalWrite(TRIG_PIN, LOW);
  matikanSemuaLED();
  matikanAktuatorHama();
  Serial.print("Status awal tombol: ");
  if (digitalRead(TOMBOL_SUDAH_DISIRAM) == LOW) {
    Serial.println("DITEKAN");
  } else {
    Serial.println("TIDAK DITEKAN");
  }
}
// =====================================================
// RTC
// =====================================================
void initRTC() {
  // --- 1. I2C BUS RECOVERY (Mengatasi SDA stuck LOW akibat Deep Sleep) ---
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(10);

  // Jika SDA tertahan LOW, berikan clock SCL 9 kali untuk melepaskan kuncian
  if (digitalRead(I2C_SDA) == LOW) {
    Serial.println("Jalur I2C tersangkut (SDA LOW), mencoba melepaskan kuncian bus...");
    pinMode(I2C_SCL, OUTPUT);
    for (int i = 0; i < 9; i++) {
      digitalWrite(I2C_SCL, HIGH);
      delayMicroseconds(10);
      digitalWrite(I2C_SCL, LOW);
      delayMicroseconds(10);
    }
    pinMode(I2C_SCL, INPUT_PULLUP);
  }

  // --- 2. INISIALISASI WIRE & RTC ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // Set kecepatan I2C standar (100kHz)

  // Masukkan pointer &Wire secara eksplisit
  if (!rtc.begin(&Wire)) {
    Serial.println("RTC DS3231 tidak terdeteksi.");
    Serial.println("Cek kabel SDA, SCL, VCC, dan GND.");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC kehilangan daya.");
    Serial.println("Waktu RTC diatur sesuai waktu compile program.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  DateTime now = rtc.now();
  Serial.print("Waktu RTC aktif: ");
  Serial.println(formatTanggalJam(now));
}
// =====================================================
// SIM800L
// =====================================================
void initSIM800L() {
  sim800.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);
  delay(3000);
  Serial.println("Inisialisasi SIM800L...");
  kirimCommandSIM("AT", 1000);
  kirimCommandSIM("ATE0", 1000);
  kirimCommandSIM("AT+CMGF=1", 1000);
  kirimCommandSIM("AT+CSCS=\"GSM\"", 1000);
  kirimCommandSIM("AT+CPMS=\"SM\",\"SM\",\"SM\"", 1000);
  kirimCommandSIM("AT+CNMI=2,1,0,0,0", 1000);

  // Aktifkan pembaruan waktu dan zona waktu dari jaringan operator.
  kirimCommandSIM("AT+CLTS=1", 1000);
  kirimCommandSIM("AT+CTZU=1", 1000);
  kirimCommandSIM("AT&W", 1000);

  Serial.println("SIM800L siap digunakan.");
}

// =====================================================
// KIRIM AT DAN AMBIL RESPONS
// =====================================================
String kirimATBacaRespon(String command, unsigned long timeout) {
  // Bersihkan data lama agar tidak tercampur dengan respons baru.
  while (sim800.available()) {
    sim800.read();
  }

  sim800.println(command);

  String respon = "";
  unsigned long mulai = millis();

  while (millis() - mulai < timeout) {
    while (sim800.available()) {
      respon += char(sim800.read());
    }
    delay(10);
  }

  Serial.print("AT: ");
  Serial.println(command);
  Serial.println(respon);

  return respon;
}

// =====================================================
// SINKRONISASI RTC DARI WAKTU JARINGAN SIM800L
// =====================================================
bool sinkronRTC() {
  DateTime rtcSebelum = rtc.now();

  // setup() dijalankan lagi setelah deep sleep. Cukup sinkron sekali per hari.
  bool sudahSinkronHariIni =
      rtcSebelum.day() == tanggalSinkronRTCTerakhir &&
      rtcSebelum.month() == bulanSinkronRTCTerakhir &&
      rtcSebelum.year() == tahunSinkronRTCTerakhir;

  if (sudahSinkronHariIni) {
    Serial.println("RTC sudah disinkronkan hari ini.");
    return true;
  }

  Serial.println();
  Serial.println("Mencoba sinkronisasi RTC dari jaringan GSM...");

  // Tunggu sampai SIM800L terdaftar di jaringan.
  bool jaringanSiap = false;
  unsigned long mulaiRegistrasi = millis();

  while (millis() - mulaiRegistrasi < TIMEOUT_REGISTRASI_GSM_MS) {
    String responRegistrasi = kirimATBacaRespon("AT+CREG?", 1000);

    int posisiCREG = responRegistrasi.indexOf("+CREG:");
    if (posisiCREG >= 0) {
      int posisiKoma = responRegistrasi.indexOf(',', posisiCREG);
      int statusRegistrasi = -1;

      if (posisiKoma >= 0) {
        statusRegistrasi =
            responRegistrasi.substring(posisiKoma + 1).toInt();
      } else {
        int posisiTitikDua = responRegistrasi.indexOf(':', posisiCREG);
        statusRegistrasi =
            responRegistrasi.substring(posisiTitikDua + 1).toInt();
      }

      // 1 = jaringan asal, 5 = roaming.
      if (statusRegistrasi == 1 || statusRegistrasi == 5) {
        jaringanSiap = true;
        break;
      }
    }

    delay(1000);
  }

  if (!jaringanSiap) {
    Serial.println("Sinkronisasi gagal: SIM800L belum terdaftar di jaringan.");
    return false;
  }

  // Beri waktu agar modem menerima informasi waktu dari operator.
  delay(2000);

  // Coba beberapa kali karena +CCLK kadang belum langsung diperbarui.
  for (int percobaan = 1; percobaan <= 5; percobaan++) {
    String respon = kirimATBacaRespon("AT+CCLK?", 1500);

    int posisiCCLK = respon.indexOf("+CCLK:");
    int petikAwal = respon.indexOf('"', posisiCCLK);
    int petikAkhir = respon.indexOf('"', petikAwal + 1);

    if (posisiCCLK >= 0 && petikAwal >= 0 && petikAkhir > petikAwal) {
      String nilai = respon.substring(petikAwal + 1, petikAkhir);
      nilai.trim();

      // Format umum SIM800L:
      // yy/MM/dd,hh:mm:ss+zz
      // Contoh WIB: 26/08/04,21:30:00+28
      if (nilai.length() >= 17) {
        int tahun = 2000 + nilai.substring(0, 2).toInt();
        int bulan = nilai.substring(3, 5).toInt();
        int tanggal = nilai.substring(6, 8).toInt();
        int jam = nilai.substring(9, 11).toInt();
        int menit = nilai.substring(12, 14).toInt();
        int detik = nilai.substring(15, 17).toInt();

        bool waktuValid =
            tahun >= 2024 && tahun <= 2099 &&
            bulan >= 1 && bulan <= 12 &&
            tanggal >= 1 && tanggal <= 31 &&
            jam >= 0 && jam <= 23 &&
            menit >= 0 && menit <= 59 &&
            detik >= 0 && detik <= 59;

        if (waktuValid) {
          int offsetJaringanMenit = ZONA_WAKTU_MENIT;

          // +28 berarti 28 x 15 menit = UTC+7.
          if (nilai.length() >= 20 &&
              (nilai.charAt(17) == '+' || nilai.charAt(17) == '-')) {
            int satuan15Menit = nilai.substring(18).toInt();
            offsetJaringanMenit = satuan15Menit * 15;

            if (nilai.charAt(17) == '-') {
              offsetJaringanMenit = -offsetJaringanMenit;
            }
          }

          DateTime waktuLokalJaringan(
              tahun, bulan, tanggal, jam, menit, detik);

          // Ubah waktu jaringan ke UTC, lalu ke zona target.
          int64_t epochUTC =
              (int64_t)waktuLokalJaringan.unixtime() -
              ((int64_t)offsetJaringanMenit * 60LL);

          int64_t epochTarget =
              epochUTC + ((int64_t)ZONA_WAKTU_MENIT * 60LL);

          if (epochTarget > 0 && epochTarget <= 0xFFFFFFFFLL) {
            DateTime waktuHasil((uint32_t)epochTarget);
            rtc.adjust(waktuHasil);

            tanggalSinkronRTCTerakhir = waktuHasil.day();
            bulanSinkronRTCTerakhir = waktuHasil.month();
            tahunSinkronRTCTerakhir = waktuHasil.year();

            Serial.print("RTC sebelum sinkronisasi: ");
            Serial.println(formatTanggalJam(rtcSebelum));
            Serial.print("RTC setelah sinkronisasi: ");
            Serial.println(formatTanggalJam(rtc.now()));

            return true;
          }
        }
      }
    }

    Serial.print("Waktu jaringan belum valid. Percobaan ");
    Serial.print(percobaan);
    Serial.println("/5");
    delay(2000);
  }

  Serial.println("Sinkronisasi gagal: operator belum memberikan waktu valid.");
  Serial.println("RTC tetap memakai waktu sebelumnya.");
  return false;
}
// =====================================================
// COMMAND SIM
// =====================================================
void kirimCommandSIM(String command, unsigned long timeout) {
  sim800.println(command);
  unsigned long mulai = millis();
  while (millis() - mulai < timeout) {
    while (sim800.available()) {
      char c = sim800.read();
      Serial.write(c);
    }
  }
  Serial.println();
}
// =====================================================
// KIRIM SMS
// =====================================================
bool kirimSMS(String nomor, String pesan) {
  Serial.println();
  Serial.println("Mengirim SMS...");
  Serial.print("Tujuan: ");
  Serial.println(nomor);
  Serial.println("Isi SMS:");
  Serial.println(pesan);
  sim800.println("AT+CMGF=1");
  delay(500);
  sim800.print("AT+CMGS=\"");
  sim800.print(nomor);
  sim800.println("\"");
  delay(1000);
  sim800.print(pesan);
  delay(500);
  sim800.write(26);
  delay(6000);
  String respon = "";
  while (sim800.available()) {
    respon += char(sim800.read());
  }
  Serial.println("Respon SIM800L:");
  Serial.println(respon);
  if (respon.indexOf("OK") >= 0 || respon.indexOf("+CMGS") >= 0) {
    Serial.println("SMS berhasil dikirim.");
    return true;
  } else {
    Serial.println("SMS belum terkonfirmasi berhasil.");
    return false;
  }
}
// =====================================================
// CEK SMS MASUK
// =====================================================
void cekSMSMasuk() {
  while (sim800.available()) {
    char c = sim800.read();
    if (c == '\n') {
      bufferSIM.trim();
      if (bufferSIM.length() > 0) {
        prosesBarisSIM(bufferSIM);
      }
      bufferSIM = "";
    } else if (c != '\r') {
      bufferSIM += c;
    }
  }
}
// =====================================================
// PROSES BARIS SIM
// =====================================================
void prosesBarisSIM(String baris) {
  Serial.print("SIM800L masuk: ");
  Serial.println(baris);
  if (baris.startsWith("+CMTI:")) {
    int indexSMS = ambilIndexDariCMTI(baris);
    bacaSMSDariIndex(indexSMS);
    return;
  }
  if (baris.startsWith("+CMT:")) {
    nomorPengirimTerakhir = ambilNomorDariHeaderSMS(baris);
    menungguIsiSMS = true;
    Serial.print("SMS dari: ");
    Serial.println(nomorPengirimTerakhir);
    return;
  }
  if (menungguIsiSMS) {
    menungguIsiSMS = false;
    String pesan = baris;
    pesan.trim();
    pesan.toUpperCase();
    prosesPerintahSMS(nomorPengirimTerakhir, pesan);
  }
}
// =====================================================
// AMBIL NOMOR PENGIRIM
// =====================================================
String ambilNomorDariHeaderSMS(String header) {
  int posisi = 0;
  while (true) {
    int petikPertama = header.indexOf("\"", posisi);
    if (petikPertama < 0) {
      break;
    }
    int petikKedua = header.indexOf("\"", petikPertama + 1);
    if (petikKedua < 0) {
      break;
    }
    String isi = header.substring(petikPertama + 1, petikKedua);
    if (isi.startsWith("+")) {
      return isi;
    }
    posisi = petikKedua + 1;
  }
  return NOMOR_PETANI;
}
// =====================================================
// INDEX SMS
// =====================================================
int ambilIndexDariCMGL(String header) {
  int titikDua = header.indexOf(":");
  int koma = header.indexOf(",", titikDua + 1);
  if (titikDua >= 0 && koma > titikDua) {
    String angka = header.substring(titikDua + 1, koma);
    angka.trim();
    return angka.toInt();
  }
  return -1;
}
int ambilIndexDariCMTI(String baris) {
  int koma = baris.lastIndexOf(",");
  if (koma >= 0) {
    String angka = baris.substring(koma + 1);
    angka.trim();
    return angka.toInt();
  }
  return -1;
}
// =====================================================
// BACA SMS BERDASARKAN INDEX
// =====================================================
void bacaSMSDariIndex(int indexSMS) {
  if (indexSMS < 0) {
    return;
  }
  Serial.print("Membaca SMS index: ");
  Serial.println(indexSMS);
  sim800.print("AT+CMGR=");
  sim800.println(indexSMS);
  unsigned long mulai = millis();
  String nomorPengirim = NOMOR_PETANI;
  String isiPesan = "";
  bool barisBerikutnyaIsiPesan = false;
  while (millis() - mulai < 5000) {
    if (sim800.available()) {
      String baris = sim800.readStringUntil('\n');
      baris.trim();
      if (baris.length() == 0) {
        continue;
      }
      Serial.print("CMGR: ");
      Serial.println(baris);
      if (baris.startsWith("+CMGR:")) {
        nomorPengirim = ambilNomorDariHeaderSMS(baris);
        barisBerikutnyaIsiPesan = true;
      } else if (barisBerikutnyaIsiPesan && !baris.startsWith("OK") && !baris.startsWith("ERROR")) {
        isiPesan = baris;
        isiPesan.trim();
        isiPesan.toUpperCase();
        barisBerikutnyaIsiPesan = false;
      }
    }
  }
  if (isiPesan.length() > 0) {
    prosesPerintahSMS(nomorPengirim, isiPesan);
    hapusSMS(indexSMS);
  }
}
// =====================================================
// BACA SMS BELUM DIBACA
// =====================================================
void bacaSMSBelumDibacaDariMemori() {
  Serial.println();
  Serial.println("Mengecek SMS belum dibaca di memori SIM...");
  sim800.println("AT+CMGL=\"REC UNREAD\"");
  unsigned long mulai = millis();
  int indexSMS = -1;
  String nomorPengirim = NOMOR_PETANI;
  String isiPesan = "";
  bool barisBerikutnyaIsiPesan = false;
  while (millis() - mulai < 7000) {
    if (sim800.available()) {
      String baris = sim800.readStringUntil('\n');
      baris.trim();
      if (baris.length() == 0) {
        continue;
      }
      Serial.print("CMGL: ");
      Serial.println(baris);
      if (baris.startsWith("+CMGL:")) {
        if (isiPesan.length() > 0 && indexSMS >= 0) {
          prosesPerintahSMS(nomorPengirim, isiPesan);
          hapusSMS(indexSMS);
        }
        indexSMS = ambilIndexDariCMGL(baris);
        nomorPengirim = ambilNomorDariHeaderSMS(baris);
        isiPesan = "";
        barisBerikutnyaIsiPesan = true;
      } else if (barisBerikutnyaIsiPesan && !baris.startsWith("OK") && !baris.startsWith("ERROR")) {
        isiPesan = baris;
        isiPesan.trim();
        isiPesan.toUpperCase();
        barisBerikutnyaIsiPesan = false;
      }
    }
  }
  if (isiPesan.length() > 0 && indexSMS >= 0) {
    prosesPerintahSMS(nomorPengirim, isiPesan);
    hapusSMS(indexSMS);
  }
  Serial.println("Cek SMS memori selesai.");
}
// =====================================================
// HAPUS SMS
// =====================================================
void hapusSMS(int indexSMS) {
  if (indexSMS < 0) {
    return;
  }
  Serial.print("Menghapus SMS index: ");
  Serial.println(indexSMS);
  sim800.print("AT+CMGD=");
  sim800.println(indexSMS);
  delay(1000);
  while (sim800.available()) {
    Serial.write((uint8_t)sim800.read());
  }
  Serial.println();
}
// =====================================================
// MODE AKTIF SMS
// =====================================================
void modeAktifTerimaSMS() {
  Serial.println();
  Serial.println("Mode aktif menerima SMS dimulai.");
  Serial.print("Durasi aktif: ");
  Serial.print(DURASI_MODE_AKTIF_SMS_MS / 1000);
  Serial.println(" detik");
  unsigned long mulai = millis();
  while (millis() - mulai < DURASI_MODE_AKTIF_SMS_MS) {
    /*
      Cek SMS.
    */
    cekSMSMasuk();
    /*
      PERBAIKAN PENTING:
      Push button terus diperiksa
      selama 5 menit mode aktif.
    */
    cekTombolSudahDisiram();
    delay(20);
  }
  Serial.println("Mode aktif menerima SMS selesai.");
}
// =====================================================
// PROSES PERINTAH SMS
// =====================================================
void prosesPerintahSMS(String nomorPengirim, String pesan) {
  Serial.println();
  Serial.println("Perintah SMS diterima");
  Serial.print("Dari   : ");
  Serial.println(nomorPengirim);
  Serial.print("Pesan  : ");
  Serial.println(pesan);
  waktuSekarang = rtc.now();
  if (pesan == "STATUS") {
    String balasan = buatSMSStatusManual();
    kirimSMS(nomorPengirim, balasan);
  } else if (pesan == "HAMA ON") {
    fiturHamaAktif = true;
    String balasan = "";
    balasan += "FITUR DETEKSI HAMA AKTIF\n\n";
    balasan += "Waktu: ";
    balasan += formatTanggalJam(waktuSekarang);
    balasan += "\n\n";
    balasan += "Anda akan diberikan notifikasi jika hama terdeteksi di sawah Anda.";
    kirimSMS(nomorPengirim, balasan);
  } else if (pesan == "HAMA OFF") {
    fiturHamaAktif = false;
    notifikasiHamaTertunda = false;
    jumlahDeteksiHamaDalamJendela = 0;
    matikanAktuatorHama();
    String balasan = "";
    balasan += "FITUR DETEKSI HAMA MATI\n\n";
    balasan += "Waktu: ";
    balasan += formatTanggalJam(waktuSekarang);
    balasan += "\n\n";
    balasan += "Mode hemat daya aktif.\n";
    balasan += "Sistem tidak akan mendeteksi hama di sekitar sawah.";
    kirimSMS(nomorPengirim, balasan);
  } else if (pesan == "SUDAH DISIRAM" || pesan == "RESET AIR") {
    prosesTombolSudahDisiram();
  } else {
    String balasan = "";
    balasan += "PERINTAH TIDAK DIKENALI\n\n";
    balasan += "Gunakan perintah:\n";
    balasan += "STATUS\n";
    balasan += "HAMA ON\n";
    balasan += "HAMA OFF\n";
    balasan += "SUDAH DISIRAM";
    kirimSMS(nomorPengirim, balasan);
  }
}
// =====================================================
// SENSOR AIR
// =====================================================
float bacaJarakSekali() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long durasi = pulseIn(ECHO_PIN, HIGH, TIMEOUT_ECHO);
  if (durasi == 0) {
    return NAN;
  }
  float jarak = durasi * 0.0343 / 2.0;
  if (jarak > JSN_MAX_BACA_CM) {
    return NAN;
  }
  /*
    Jangan paksa hasil yang berada di area buta menjadi
    JSN_MIN_BACA_CM karena dapat menghasilkan tinggi air
    palsu. Anggap pembacaan tersebut sebagai sensor error.
  */
  if (jarak < JSN_MIN_BACA_CM) {
    return NAN;
  }
  return jarak;
}
// =====================================================
// MEDIAN SENSOR
// =====================================================
float bacaJarakMedian() {
  float data[JUMLAH_SAMPEL];
  int jumlahValid = 0;
  for (int i = 0; i < JUMLAH_SAMPEL; i++) {
    float hasil = bacaJarakSekali();
    if (!isnan(hasil)) {
      data[jumlahValid] = hasil;
      jumlahValid++;
    }
    delay(100);
  }
  if (jumlahValid == 0) {
    return NAN;
  }
  for (int i = 0; i < jumlahValid - 1; i++) {
    for (int j = i + 1; j < jumlahValid; j++) {
      if (data[j] < data[i]) {
        float temp = data[i];
        data[i] = data[j];
        data[j] = temp;
      }
    }
  }
  return data[jumlahValid / 2];
}
// =====================================================
// HITUNG TINGGI AIR
// =====================================================
float hitungTinggiAir(float jarakSensor) {
  if (isnan(jarakSensor)) {
    return NAN;
  }

  /*
    Rumus:
    tinggi air = jarak sensor ke tanah
                 - jarak sensor ke permukaan air
                 + koreksi kalibrasi
  */
  float tinggiAir = JARAK_SENSOR_KE_TANAH_CM
                    - jarakSensor
                    + KOREKSI_TINGGI_AIR_CM;

  // Nilai negatif berarti permukaan air belum di atas tanah.
  if (tinggiAir < 0) {
    tinggiAir = 0;
  }

  return tinggiAir;
}
// =====================================================
// PENENTUAN STATUS AIR
// =====================================================
StatusAir tentukanStatusAir(float jarakSensor, float tinggiAir) {
  if (isnan(jarakSensor) || isnan(tinggiAir)) {
    return SENSOR_ERROR;
  }
  /*
    STATUS:
    >= 25 cm
    BERLEBIH
    20 - <25 cm
    CUKUP
    <20 cm
    KURANG
  */
  if (tinggiAir >= 25.0) {
    return BERLEBIH;
  } else if (tinggiAir >= 20.0) {
    return CUKUP;
  } else {
    return KURANG;
  }
}
// =====================================================
// NAMA STATUS
// =====================================================
String namaStatus(StatusAir status) {
  switch (status) {
  case BERLEBIH:
    return "BERLEBIH";
  case CUKUP:
    return "CUKUP";
  case KURANG:
    return "KURANG";
  default:
    return "SENSOR ERROR";
  }
}
// =====================================================
// REKOMENDASI
// =====================================================
String rekomendasi(StatusAir status) {
  switch (status) {
  case BERLEBIH:
    return "Air berlebih. Kurangi atau hentikan pengairan sesuai kebutuhan tanaman.";
  case CUKUP:
    return "Air cukup. Pertahankan kondisi pengairan.";
  case KURANG:
    return "Air kurang. Lakukan pengairan sesuai kebutuhan tanaman.";
  default:
    return "Cek posisi sensor dan kabel.";
  }
}
// =====================================================
// LED STATUS
// =====================================================
void updateLEDStatus(StatusAir status) {
  matikanSemuaLED();
  if (status == CUKUP) {
    digitalWrite(LED_HIJAU, HIGH);
  } else if (status == BERLEBIH) {
    digitalWrite(LED_KUNING, HIGH);
  } else if (status == KURANG) {
    digitalWrite(LED_MERAH, HIGH);
  } else {
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_HIJAU, HIGH);
      digitalWrite(LED_KUNING, HIGH);
      digitalWrite(LED_MERAH, HIGH);
      delay(200);
      matikanSemuaLED();
      delay(200);
    }
  }
}
void matikanSemuaLED() {
  digitalWrite(LED_HIJAU, LOW);
  digitalWrite(LED_KUNING, LOW);
  digitalWrite(LED_MERAH, LOW);
}
// =====================================================
// SMS STATUS AIR
// =====================================================
bool prosesSMSAir(StatusAir status, float tinggiAir) {
  unsigned long epochSekarang = waktuSekarang.unixtime();
  bool smsAirDiprioritaskan = false;

  if (status == BERLEBIH &&
      bolehKirimSMS(waktuSmsPeringatanTerakhir, epochSekarang)) {
    smsAirDiprioritaskan = true;
    String pesan = buatSMSBerlebih(tinggiAir);
    bool sukses = kirimSMS(NOMOR_PETANI, pesan);
    if (sukses) {
      waktuSmsPeringatanTerakhir = epochSekarang;
    }
  } else if (status == KURANG &&
             bolehKirimSMS(waktuSmsKurangTerakhir, epochSekarang)) {
    smsAirDiprioritaskan = true;
    String pesan = buatSMSKurang(tinggiAir);
    bool sukses = kirimSMS(NOMOR_PETANI, pesan);
    if (sukses) {
      waktuSmsKurangTerakhir = epochSekarang;
    }
  }

  return smsAirDiprioritaskan;
}
// =====================================================
// COOLDOWN SMS
// =====================================================
bool bolehKirimSMS(unsigned long waktuSmsTerakhir, unsigned long epochSekarang) {
  if (waktuSmsTerakhir == 0) {
    return true;
  }
  if (epochSekarang - waktuSmsTerakhir >= COOLDOWN_SMS_DETIK) {
    return true;
  }
  return false;
}
// =====================================================
// RESET OTOMATIS COOLDOWN SAAT AIR KEMBALI AMAN
// =====================================================
void resetCooldownSMSJikaAirAman(StatusAir status) {
  /*
    Pada klasifikasi yang sedang dipakai kode ini,
    status CUKUP adalah kondisi aman/non-alert.
    Ambang status tidak diubah karena tidak termasuk
    bagian yang diminta untuk disesuaikan.
  */
  if (status != CUKUP) {
    return;
  }

  if (waktuSmsPeringatanTerakhir != 0 || waktuSmsKurangTerakhir != 0) {
    waktuSmsPeringatanTerakhir = 0;
    waktuSmsKurangTerakhir = 0;
    Serial.println("Level air kembali CUKUP. Cooldown SMS air direset otomatis.");
  }
}
// =====================================================
// BUZZER KONDISI AIR KRITIS
// =====================================================
void aktifkanBuzzerAirKritis(StatusAir status) {
  /*
    Status KURANG digunakan sebagai kondisi kritis pada
    klasifikasi kode saat ini. Ambang level tidak diubah.
  */
  if (status != KURANG) {
    return;
  }

  Serial.println("Kondisi air kritis/KURANG. Buzzer peringatan aktif.");
  digitalWrite(BUZZER_PIN, HIGH);
  delay(DURASI_BUZZER_AIR_KRITIS_MS);
  digitalWrite(BUZZER_PIN, LOW);
}
// =====================================================
// REKAP HAMA PER HARI KALENDER
// =====================================================
void perbaruiCounterHamaHarian(DateTime waktu) {
  if (tanggalCounterHama < 0 || bulanCounterHama < 0 || tahunCounterHama < 0) {
    tanggalCounterHama = waktu.day();
    bulanCounterHama = waktu.month();
    tahunCounterHama = waktu.year();
    return;
  }

  bool tanggalBerubah = tanggalCounterHama != waktu.day() ||
                        bulanCounterHama != waktu.month() ||
                        tahunCounterHama != waktu.year();

  if (!tanggalBerubah) {
    return;
  }

  jumlahDeteksiHamaKemarin = jumlahDeteksiHama;
  jumlahDeteksiHama = 0;
  tanggalCounterHama = waktu.day();
  bulanCounterHama = waktu.month();
  tahunCounterHama = waktu.year();

  Serial.print("Rekap hama hari sebelumnya: ");
  Serial.print(jumlahDeteksiHamaKemarin);
  Serial.println(" kali.");
}
// =====================================================
// SMS HARIAN
// =====================================================
bool prosesSMSHarian(float tinggiAir, StatusAir status) {
  bool jamSesuai = waktuSekarang.hour() == JAM_SMS_HARIAN;
  bool menitSesuai = waktuSekarang.minute() >= MENIT_SMS_HARIAN_AWAL &&
                     waktuSekarang.minute() <= MENIT_SMS_HARIAN_AKHIR;
  if (!jamSesuai || !menitSesuai) {
    return false;
  }
  bool sudahKirimHariIni = tanggalSmsHarianTerakhir == waktuSekarang.day() &&
                           bulanSmsHarianTerakhir == waktuSekarang.month() &&
                           tahunSmsHarianTerakhir == waktuSekarang.year();
  if (sudahKirimHariIni) {
    return false;
  }

  String pesan = buatSMSStatusHarian(tinggiAir, status);
  bool sukses = kirimSMS(NOMOR_PETANI, pesan);
  if (sukses) {
    tanggalSmsHarianTerakhir = waktuSekarang.day();
    bulanSmsHarianTerakhir = waktuSekarang.month();
    tahunSmsHarianTerakhir = waktuSekarang.year();
  }

  // Status harian tetap mengambil prioritas siklus ini, termasuk jika
  // modem belum mengonfirmasi pengiriman agar SMS hama tidak mendahului.
  return true;
}
// =====================================================
// PUSH BUTTON
// =====================================================
void cekTombolSudahDisiram() {
  bool bacaan = digitalRead(TOMBOL_SUDAH_DISIRAM);
  /*
    Jika ada perubahan HIGH ke LOW
    atau LOW ke HIGH.
  */
  if (bacaan != statusTombolTerakhir) {
    waktuDebounceTombol = millis();
    Serial.print("Perubahan tombol: ");
    if (bacaan == LOW) {
      Serial.println("LOW / DITEKAN");
    } else {
      Serial.println("HIGH / DILEPAS");
    }
  }
  /*
    Tunggu sampai bacaan stabil.
  */
  if (millis() - waktuDebounceTombol > DEBOUNCE_TOMBOL_MS) {
    if (bacaan != statusTombolStabil) {
      statusTombolStabil = bacaan;
      /*
        INPUT_PULLUP:
        LOW = tombol ditekan.
      */
      if (statusTombolStabil == LOW) {
        Serial.println();
        Serial.println("=================================");
        Serial.println("PUSH BUTTON DITEKAN");
        Serial.println("=================================");
        prosesTombolSudahDisiram();
      }
    }
  }
  statusTombolTerakhir = bacaan;
}
// =====================================================
// PROSES PENGAIRAN
// =====================================================
void prosesTombolSudahDisiram() {
  waktuSekarang = rtc.now();
  /*
    Reset cooldown peringatan air.
  */
  waktuSmsPeringatanTerakhir = 0;
  waktuSmsKurangTerakhir = 0;
  /*
    Tambah counter pengairan.
  */
  jumlahPengairan++;
  bool logEEPROMBerhasil = simpanLogPengairanEEPROM(waktuSekarang);
  Serial.println();
  Serial.println("TOMBOL SUDAH DISIRAM DITEKAN");
  Serial.print("Waktu: ");
  Serial.println(formatTanggalJam(waktuSekarang));
  Serial.print("Jumlah pengairan: ");
  Serial.println(jumlahPengairan);
  if (logEEPROMBerhasil) {
    Serial.println("Timestamp pengairan berhasil disimpan ke EEPROM AT24C32.");
  } else {
    Serial.println("PERINGATAN: timestamp pengairan gagal disimpan ke EEPROM AT24C32.");
  }
  Serial.println("Cooldown SMS status berlebih dan kurang direset.");
  /*
    Feedback LED.
  */
  kedipHijau(3);
  /*
    SMS konfirmasi.
  */
  String pesan = buatSMSPengairanTercatat();
  kirimSMS(NOMOR_PETANI, pesan);
}
// =====================================================
// KEDIP LED
// =====================================================
void kedipHijau(int jumlahKedip) {
  matikanSemuaLED();
  for (int i = 0; i < jumlahKedip; i++) {
    digitalWrite(LED_HIJAU, HIGH);
    delay(200);
    digitalWrite(LED_HIJAU, LOW);
    delay(200);
  }
}
// =====================================================
// VALIDASI PIR
// =====================================================
bool validasiPIRBerulang() {
  int jumlahHigh = 0;
  for (int i = 0; i < JUMLAH_SAMPEL_VALIDASI_PIR; i++) {
    if (digitalRead(PIR_PIN) == HIGH) {
      jumlahHigh++;
    }
    if (i < JUMLAH_SAMPEL_VALIDASI_PIR - 1) {
      delay(JEDA_SAMPEL_VALIDASI_PIR_MS);
    }
  }
  jumlahDeteksiPIRValidasi = jumlahHigh;
  Serial.print("Validasi PIR: ");
  Serial.print(jumlahHigh);
  Serial.print("/");
  Serial.println(JUMLAH_SAMPEL_VALIDASI_PIR);
  return jumlahHigh >= BATAS_VALIDASI_PIR;
}
// =====================================================
// DETEKSI HAMA
// =====================================================
bool prosesDeteksiHama() {
  if (!fiturHamaAktif) {
    matikanAktuatorHama();
    return false;
  }
  int statusPIR = digitalRead(PIR_PIN);
  if (statusPIR != HIGH) {
    matikanAktuatorHama();
    jumlahDeteksiPIRValidasi = 0;
    return false;
  }
  unsigned long epochSekarang = waktuSekarang.unixtime();
  Serial.println();
  Serial.println("PIR AKTIF");
  Serial.print("Waktu deteksi: ");
  Serial.println(formatJamMenit(waktuSekarang));
  if (!validasiPIRBerulang()) {
    Serial.println("Gerakan diabaikan karena validasi PIR belum cukup.");
    return false;
  }
  jumlahDeteksiHama++;
  jumlahDeteksiHamaDalamJendela++;
  Serial.println("HAMA TERVALIDASI");
  bool cooldownAktuatorSelesai =
      waktuAktuatorHamaTerakhir == 0 ||
      epochSekarang - waktuAktuatorHamaTerakhir >= COOLDOWN_AKTUATOR_HAMA_DETIK;
  if (cooldownAktuatorSelesai) {
    if (rtc.getTemperature() >= BATAS_SUHU_TINGGI_C) {
      Serial.println("Aktuator hama tidak dinyalakan karena suhu box tinggi.");
    } else {
      Serial.println("Buzzer dan LED strobo aktif dengan pola acak.");
      aktifkanAktuatorHama();
      waktuAktuatorHamaTerakhir = epochSekarang;
    }
  } else {
    Serial.println("Aktuator masih cooldown. Buzzer tidak dinyalakan ulang.");
  }
  if (jumlahDeteksiHamaDalamJendela > 3) {
    notifikasiHamaTertunda = true;
    Serial.println("Notifikasi hama masuk antrean setelah prioritas SMS air.");
  }

  return true;
}
// =====================================================
// SMS HAMA SETELAH PRIORITAS AIR
// =====================================================
bool prosesSMSHama() {
  if (!fiturHamaAktif || !notifikasiHamaTertunda) {
    return false;
  }

  unsigned long epochSekarang = waktuSekarang.unixtime();
  if (!bolehKirimSMS(waktuSmsHamaTerakhir, epochSekarang)) {
    Serial.println("SMS hama masih cooldown dan tetap disimpan dalam antrean.");
    return false;
  }

  Serial.println("Tidak ada SMS air yang perlu didahulukan. Mengirim SMS hama.");
  String pesan = buatSMSHama();
  bool sukses = kirimSMS(NOMOR_PETANI, pesan);
  if (sukses) {
    waktuSmsHamaTerakhir = epochSekarang;
    jumlahDeteksiHamaDalamJendela = 0;
    waktuAwalJendelaHama = epochSekarang;
    notifikasiHamaTertunda = false;
  }

  return true;
}
// =====================================================
// JENDELA DETEKSI HAMA
// =====================================================
void prosesJendelaHama() {
  unsigned long epochSekarang = waktuSekarang.unixtime();

  // Jangan hapus jumlah deteksi ketika SMS hama sedang menunggu giliran
  // setelah SMS ketinggian air.
  if (notifikasiHamaTertunda) {
    return;
  }

  if (epochSekarang - waktuAwalJendelaHama >= JENDELA_HAMA_DETIK) {
    jumlahDeteksiHamaDalamJendela = 0;
    waktuAwalJendelaHama = epochSekarang;
  }
}
// =====================================================
// AKTUATOR HAMA
// =====================================================
void aktifkanAktuatorHama() {
  unsigned long waktuMulai = millis();
  while (millis() - waktuMulai < DURASI_AKTUATOR_HAMA) {
    unsigned long durasiBunyi = random(150, 700);
    unsigned long durasiDiam = random(120, 500);
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_STROBE_1, HIGH);
    delay(durasiBunyi);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_STROBE_1, LOW);
    delay(durasiDiam);
  }
  matikanAktuatorHama();
}
void matikanAktuatorHama() {
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_STROBE_1, LOW);
}
// =====================================================
// SMS BERLEBIH
// =====================================================
String buatSMSBerlebih(float tinggiAir) {
  String pesan = "";
  pesan += "STATUS AIR BERLEBIH\n\n";
  pesan += "Level air di sawah melebihi batas cukup.\n";
  pesan += "Tinggi air: ";
  pesan += String(tinggiAir, 1);
  pesan += " cm\n";
  pesan += "Waktu: ";
  pesan += formatTanggalJam(waktuSekarang);
  pesan += "\n\n";
  pesan += "Periksa kondisi lahan dan kurangi pengairan bila diperlukan.";
  return pesan;
}
// =====================================================
// SMS KURANG
// =====================================================
String buatSMSKurang(float tinggiAir) {
  String pesan = "";
  pesan += "STATUS AIR KURANG\n\n";
  pesan += "Level air di sawah berada di bawah batas cukup.\n";
  pesan += "Tinggi air: ";
  pesan += String(tinggiAir, 1);
  pesan += " cm\n";
  pesan += "Waktu: ";
  pesan += formatTanggalJam(waktuSekarang);
  pesan += "\n\n";
  pesan += "Lakukan pengairan sesuai kebutuhan tanaman.";
  return pesan;
}
// =====================================================
// SMS HAMA
// =====================================================
String buatSMSHama() {
  String pesan = "";
  pesan += "HAMA AKTIF TERDETEKSI\n\n";
  pesan += "Waktu: ";
  pesan += formatTanggalJam(waktuSekarang);
  pesan += "\n";
  pesan += "Kondisi: Hama terdeteksi berulang\n";
  pesan += "Jumlah deteksi: ";
  pesan += String(jumlahDeteksiHamaDalamJendela);
  pesan += " kali\n";
  pesan += "Sistem: Buzzer dan strobo menyala\n\n";
  pesan += "Segera periksa sawah Anda.";
  return pesan;
}
// =====================================================
// SMS STATUS HARIAN
// =====================================================
String buatSMSStatusHarian(float tinggiAir, StatusAir status) {
  String pesan = "";
  pesan += "STATUS HARIAN SIPADI\n\n";
  if (isnan(tinggiAir)) {
    pesan += "Tinggi air: ERROR\n";
    pesan += "Status: SENSOR ERROR\n";
  } else {
    pesan += "Tinggi air: ";
    pesan += String(tinggiAir, 1);
    pesan += " cm\n";
    pesan += "Status: ";
    pesan += namaStatus(status);
    pesan += "\n";
  }
  pesan += "Fitur hama: ";
  pesan += (fiturHamaAktif ? "AKTIF" : "MATI");
  pesan += "\n";
  pesan += "Deteksi hama hari sebelumnya: ";
  pesan += String(jumlahDeteksiHamaKemarin);
  pesan += " kali\n";
  pesan += "Pengairan tercatat: ";
  pesan += String(jumlahPengairan);
  pesan += " kali\n";
  pesan += "Suhu: ";
  pesan += String(rtc.getTemperature(), 1);
  pesan += " C\n";
  pesan += "Waktu: ";
  pesan += formatTanggalJam(waktuSekarang);
  pesan += "\n\n";
  pesan += "Sistem SIPADI aktif dan berjalan normal.";
  return pesan;
}
// =====================================================
// SMS PENGAIRAN
// =====================================================
String buatSMSPengairanTercatat() {
  String pesan = "";
  pesan += "PENGAIRAN TERCATAT\n\n";
  pesan += "Tombol Sudah Disiram telah ditekan.\n";
  pesan += "Waktu: ";
  pesan += formatTanggalJam(waktuSekarang);
  pesan += "\n\n";
  pesan += "Cooldown peringatan air telah direset.";
  return pesan;
}
// =====================================================
// STATUS MANUAL
// =====================================================
String buatSMSStatusManual() {
  waktuSekarang = rtc.now();
  float jarakSensor = bacaJarakMedian();
  float tinggiAir = hitungTinggiAir(jarakSensor);
  StatusAir status = tentukanStatusAir(jarakSensor, tinggiAir);
  dataJarakTerakhir = jarakSensor;
  dataTinggiAirTerakhir = tinggiAir;
  statusAirTerakhir = status;
  String pesan = "";
  pesan += "STATUS SIPADI\n\n";
  if (isnan(tinggiAir)) {
    pesan += "Tinggi air: ERROR\n";
    pesan += "Status: SENSOR ERROR\n";
  } else {
    pesan += "Tinggi air: ";
    pesan += String(tinggiAir, 1);
    pesan += " cm\n";
    pesan += "Status: ";
    pesan += namaStatus(status);
    pesan += "\n";
  }
  pesan += "Fitur hama: ";
  pesan += (fiturHamaAktif ? "AKTIF" : "MATI");
  pesan += "\n";
  pesan += "Total hama: ";
  pesan += String(jumlahDeteksiHama);
  pesan += " kali\n";
  pesan += "Pengairan: ";
  pesan += String(jumlahPengairan);
  pesan += " kali\n";
  pesan += "Suhu: ";
  pesan += String(rtc.getTemperature(), 1);
  pesan += " C\n";
  pesan += "Waktu: ";
  pesan += formatTanggalJam(waktuSekarang);
  pesan += "\n\n";
  pesan += rekomendasi(status);
  return pesan;
}
// =====================================================
// FORMAT WAKTU
// =====================================================
String formatJamMenit(DateTime waktu) {
  char buffer[6];
  sprintf(buffer, "%02d:%02d", waktu.hour(), waktu.minute());
  return String(buffer);
}
String formatTanggal(DateTime waktu) {
  char buffer[11];
  sprintf(buffer, "%02d/%02d/%04d", waktu.day(), waktu.month(), waktu.year());
  return String(buffer);
}
String formatTanggalJam(DateTime waktu) {
  char buffer[20];
  sprintf(buffer, "%02d/%02d/%04d %02d:%02d", waktu.day(), waktu.month(), waktu.year(),
          waktu.hour(), waktu.minute());
  return String(buffer);
}
// =====================================================
// EEPROM AT24C32: LOGGING TIMESTAMP PENGAIRAN
// =====================================================
bool eepromBacaBlok(uint16_t alamat, uint8_t *data, uint8_t panjang) {
  if (!eepromIrigasiTersedia || panjang == 0) {
    return false;
  }

  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((uint8_t)(alamat >> 8));
  Wire.write((uint8_t)(alamat & 0xFF));
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t diterima = Wire.requestFrom((uint8_t)EEPROM_I2C_ADDRESS, panjang);
  if (diterima != panjang) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < panjang; i++) {
    data[i] = Wire.read();
  }
  return true;
}

bool eepromTulisBlok(uint16_t alamat, const uint8_t *data, uint8_t panjang) {
  if (!eepromIrigasiTersedia || panjang == 0) {
    return false;
  }

  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((uint8_t)(alamat >> 8));
  Wire.write((uint8_t)(alamat & 0xFF));
  for (uint8_t i = 0; i < panjang; i++) {
    Wire.write(data[i]);
  }

  if (Wire.endTransmission() != 0) {
    return false;
  }

  // Waktu internal write cycle AT24C32.
  delay(6);
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
      (uint8_t)(totalLog & 0xFF)};
  return eepromTulisBlok(EEPROM_ALAMAT_INDEX_BERIKUTNYA, metadata, 4);
}

bool initEEPROMIrigasi() {
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  if (Wire.endTransmission() != 0) {
    eepromIrigasiTersedia = false;
    Serial.println("EEPROM AT24C32 alamat 0x57 tidak terdeteksi.");
    return false;
  }

  eepromIrigasiTersedia = true;
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
    Serial.println("Inisialisasi area log EEPROM AT24C32...");
    if (!eepromTulisBlok(EEPROM_ALAMAT_MAGIC, EEPROM_MAGIC, 4) ||
        !eepromTulisMetadata(0, 0)) {
      eepromIrigasiTersedia = false;
      Serial.println("Gagal menginisialisasi EEPROM AT24C32.");
      return false;
    }
  }

  uint16_t indexBerikutnya = eepromBacaUInt16(EEPROM_ALAMAT_INDEX_BERIKUTNYA);
  uint16_t totalLog = eepromBacaUInt16(EEPROM_ALAMAT_TOTAL_LOG);

  if (indexBerikutnya >= EEPROM_KAPASITAS_RECORD) {
    indexBerikutnya = 0;
    eepromTulisMetadata(indexBerikutnya, totalLog);
  }

  if ((int)totalLog > jumlahPengairan) {
    jumlahPengairan = totalLog;
  }

  Serial.print("EEPROM AT24C32 siap. Total log pengairan: ");
  Serial.println(totalLog);
  return true;
}

bool simpanLogPengairanEEPROM(DateTime waktu) {
  if (!eepromIrigasiTersedia) {
    return false;
  }

  uint16_t indexBerikutnya = eepromBacaUInt16(EEPROM_ALAMAT_INDEX_BERIKUTNYA);
  uint16_t totalLog = eepromBacaUInt16(EEPROM_ALAMAT_TOTAL_LOG);

  if (indexBerikutnya >= EEPROM_KAPASITAS_RECORD) {
    indexBerikutnya = 0;
  }

  uint16_t alamatRecord = EEPROM_ALAMAT_DATA +
                           (indexBerikutnya * EEPROM_UKURAN_RECORD);

  // Format record 8 byte:
  // marker, tahun-hi, tahun-lo, bulan, tanggal, jam, menit, detik.
  uint16_t tahun = waktu.year();
  uint8_t record[EEPROM_UKURAN_RECORD] = {
      0xA5,
      (uint8_t)(tahun >> 8),
      (uint8_t)(tahun & 0xFF),
      (uint8_t)waktu.month(),
      (uint8_t)waktu.day(),
      (uint8_t)waktu.hour(),
      (uint8_t)waktu.minute(),
      (uint8_t)waktu.second()};

  if (!eepromTulisBlok(alamatRecord, record, EEPROM_UKURAN_RECORD)) {
    return false;
  }

  indexBerikutnya = (indexBerikutnya + 1) % EEPROM_KAPASITAS_RECORD;
  if (totalLog < 65535) {
    totalLog++;
  }

  return eepromTulisMetadata(indexBerikutnya, totalLog);
}
// =====================================================
// SERIAL MONITOR
// =====================================================
void tampilkanHasil(float jarakSensor, float tinggiAir, StatusAir status, bool hamaTerdeteksi) {
  Serial.println();
  Serial.println("---------- DATA SIPADI ----------");
  Serial.print("Waktu              : ");
  Serial.println(formatTanggalJam(waktuSekarang));
  Serial.print("Suhu RTC           : ");
  Serial.print(rtc.getTemperature(), 1);
  Serial.println(" C");
  if (isnan(jarakSensor)) {
    Serial.println("Jarak sensor-air   : Gagal terbaca");
    Serial.println("Tinggi air         : Gagal dihitung");
  } else {
    Serial.print("Jarak sensor-air   : ");
    Serial.print(jarakSensor, 1);
    Serial.println(" cm");
    Serial.print("Tinggi air         : ");
    Serial.print(tinggiAir, 1);
    Serial.println(" cm");
  }
  Serial.print("Status air         : ");
  Serial.println(namaStatus(status));
  Serial.print("Rekomendasi        : ");
  Serial.println(rekomendasi(status));
  Serial.print("Fitur hama         : ");
  if (fiturHamaAktif) {
    Serial.println("AKTIF");
  } else {
    Serial.println("MATI");
  }
  Serial.print("Status PIR         : ");
  if (hamaTerdeteksi) {
    Serial.println("HAMA TERDETEKSI");
  } else {
    Serial.println("Tidak ada gerakan");
  }
  Serial.print("Total deteksi hama : ");
  Serial.print(jumlahDeteksiHama);
  Serial.println(" kali");
  Serial.print("Deteksi hama window: ");
  Serial.print(jumlahDeteksiHamaDalamJendela);
  Serial.println(" kali");
  Serial.print("Validasi PIR       : ");
  Serial.print(jumlahDeteksiPIRValidasi);
  Serial.println(" trigger");
  Serial.print("Jumlah pengairan   : ");
  Serial.print(jumlahPengairan);
  Serial.println(" kali");
  Serial.println("---------------------------------");
}
// =====================================================
// PENYEBAB BANGUN
// =====================================================
void tampilkanPenyebabBangun() {
  bangunDariTombol = false;
  esp_sleep_wakeup_cause_t penyebabBangun = esp_sleep_get_wakeup_cause();
  Serial.print("Penyebab ESP32 bangun: ");
  switch (penyebabBangun) {
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Timer deep sleep");
    break;
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Sensor PIR membangunkan sistem");
    break;
  case ESP_SLEEP_WAKEUP_EXT1: {
    uint64_t sumberWakeup = esp_sleep_get_ext1_wakeup_status();
    if (sumberWakeup & (1ULL << TOMBOL_SUDAH_DISIRAM)) {
      Serial.println("Push button SUDAH DISIRAM membangunkan sistem");
      bangunDariTombol = true;
    } else {
      Serial.println("Wake-up EXT1 dari GPIO lain");
    }
    break;
  }
  case ESP_SLEEP_WAKEUP_UNDEFINED:
    Serial.println("Boot normal atau reset manual");
    break;
  default:
    Serial.println("Penyebab lain");
    break;
  }
}
// =====================================================
// DEEP SLEEP
// =====================================================
void masukDeepSleep() {
  Serial.println();
  Serial.println("Persiapan masuk deep sleep.");
  matikanAktuatorHama();
  matikanSemuaLED();
  Serial.print("ESP32 akan tidur selama ");
  Serial.print(DURASI_DEEP_SLEEP_DETIK);
  Serial.println(" detik.");
  Serial.println("Push button GPIO 33 aktif sebagai wake-up.");
  Serial.flush();
  /*
    WAKE-UP DARI TIMER.
  */
  esp_sleep_enable_timer_wakeup(DURASI_DEEP_SLEEP_DETIK * FAKTOR_DETIK_KE_MIKRODETIK);
  rtc_gpio_pullup_en((gpio_num_t)TOMBOL_SUDAH_DISIRAM);
  rtc_gpio_pulldown_dis((gpio_num_t)TOMBOL_SUDAH_DISIRAM);
  /*
    RTC peripheral tetap aktif
    agar internal pull-up tombol terjaga.
  */
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  /*
    Tombol GPIO 33 membangunkan
    ESP32 ketika LOW.
  */
  esp_sleep_enable_ext1_wakeup(1ULL << TOMBOL_SUDAH_DISIRAM, ESP_EXT1_WAKEUP_ALL_LOW);
  /*
    PIR GPIO 27 tetap sebagai
    wake-up saat fitur hama aktif.
  */
  if (fiturHamaAktif) {
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIR_PIN, 1);
  }
  Serial.println("Masuk deep sleep sekarang...");
  Serial.flush();
  delay(100);
  esp_deep_sleep_start();
}