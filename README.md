# Jarvis v1.7 — ESP32-S3 N16R8 robot

Sıfırdan yazılmış, modüler firmware. xiaozhi kodundan hiçbir şey yok. Sunucuyla ilgili tek
kısım onunla konuşma şekli (`xiaozhi_link`), o da tek bir arayüzün (`server_link.h`) arkasında:
başka bir ses API'sine geçmek için sadece yeni bir link yazmak yeterli.

Robotun bütün yazılımı bu depodan derlenir ve tek bir `jarvis.bin` içinde gelir. Flash'ta
önceden duran hiçbir şeye (assets bölümü vb.) bağlı değildir; boş bir ESP32-S3 N16R8'e
yüklendiğinde de eksiksiz çalışır.

## Özellikler

| Bölüm | Ne yapar |
|---|---|
| Sohbet | Wake word "Jarvis" veya BOOT tuşu → dinleme → sunucunun cevabı hoparlörden. Robot konuşurken BOOT tuşu cevabı keser ve yeniden dinler. Yarı çift yönlü (robot konuşurken mikrofon gönderilmez). |
| Wake word | Espressif esp-sr WakeNet9 `wn9_jarvis_tts`. Model her derlemede esp-sr'in kendi araçlarıyla (`movemodel.py` / `pack_model.py`) paketlenir ve uygulamaya gömülür (291 KB, flash'tan okunur, RAM'e kopyalanmaz). Model seçimi `sdkconfig.defaults`: `CONFIG_SR_WN_WN9_JARVIS_TTS=y`. |
| Ses | I2S mikrofon 16 kHz → Opus (60 ms, mono) → sunucu; sunucudan Opus 24 kHz → hoparlör. |
| Sunucu bağlantısı | xiaozhi.me: HTTPS yapılandırma isteği, MQTT (TLS 8883) kontrol mesajları, UDP + AES-128-CTR ses. Aktivasyon kodu ekranda gösterilir. |
| Araçlar (MCP) | `self.get_device_status`, `self.robot.move` (dön / ileri / geri), `self.robot.stop`, `self.camera.take_photo` (fotoğrafı çekip sunucunun görme servisine sorar), `self.audio_speaker.set_volume`, `self.robot.express` (yüz ifadesi). |
| Kamera | OV2640, esp32-camera sürücüsü, sadece fotoğraf anında açılır ve kapanır (sürekli akış yok). XGA 1024×768 JPEG (`camera.h` `kPhotoSize`; `cam sweep upload` bütün boyutları ölçer); pozlama ve beyaz dengesi için 3 kare atılır, 4. gönderilir. Önizleme ekranı yatayda soldan sağa tam doldurur (240×180). |
| Hareket | Jiroskopla dönüşler (öğrenilen erken durma payı), süreyle düz gidiş, PWM en az %80 (kural). Ayarlar NVS `robot`. |
| IMU | MPU-6050, 10 ms'de bir register okuma (FIFO yok), açılışta kalibrasyon. |
| Güç | INA219: 3.5 V uyarı / 3.3 V koruma / 3.6 V geri dönüş. |
| Yüz | ST7789 + LVGL 9: gözler, ifadeler, altyazılar. Altyazılar hazır bitmap font: Noto Sans 14 px (Türkçe harflerle, Ğ hariç), doğrudan flash'tan okunur; çalışma anında çizim yok, RAM kullanmaz. |
| Wi-Fi | Her denemeden önce tam tarama, bilinen ağın en güçlü erişim noktasına BSSID ile bağlanma, olmazsa yeniden tarama (NeuroVision / NetworkManager yöntemi). Ev modeminde güç kesintisinden sonraki 30–76 sn'lik gecikme böyle 5–10 sn'ye indi. |
| Wi-Fi kurulumu | Kayıtlı ağ yoksa ya da 30 sn'de bağlanılamazsa robot kendi ağını açar: **Jarvis04112026**, şifresiz (ekranda yazar). Telefon bağlanınca kurulum sayfası kendiliğinden açılır (DNS yönlendirme, bağlantı kontrol adreslerinin yönlendirilmesi, DHCP seçenek 114). Adres: http://192.168.4.1. Ağ seçilir, şifre yazılır; bağlanınca NVS'ye kaydedilir ve kurulum ağı kapanır. Yanlış şifre sayfada gösterilir. |
| Konsol | Telnet port 23 (`nc 192.168.1.125 23`): `help`, `status`, `crash`, `log on/off`, `log last [satır]`, `cam`, `wake`, `convo`, `turn`, `dist`, `drive`, `stop`, `imu`, `tune`, `set`, `save`, `gyro_scale`, `volume`, `wifi <ssid> <şifre>`, `ota <url>`, `reboot`, `reboot cold` (güç kesintisi gibi: modeme ayrıldığını bildirmeden yeniden başlar, Wi-Fi testi için). |
| Çökme kaydı | Panik nedeni RTC belleğinde saklanır, `crash` ile okunur (`addr2line` ile `build/jarvis.elf`). |
| Güncelleme | `tools/ota.sh [--build]`: Mac'ten Wi-Fi üzerinden. Bootloader'da rollback açık; yeni sürüm Wi-Fi ve konsol ayağa kalkınca kendini geçerli işaretler. |

## Bellek (PSRAM)

- Büyük tamponlar PSRAM'de: ses paketleri, JPEG, HTTP gövdeleri, MQTT birleştirme, log geçmişi (24 KB), TLS (mbedTLS dış bellek).
- Flash'a yazmayan görevlerin yığınları PSRAM'de: sohbet kontrolü (6 KB), mikrofon + Opus kodlayıcı (32 KB), hoparlör (16 KB), LVGL (8 KB).
- NVS'ye yazan görevler (wifi, tools, konsol) iç RAM yığınında; dahili RAM DMA'ya (kamera 16 KB, I2S, Wi-Fi) ayrılır.
- `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=1024`: 1 KB'tan büyük ayırmalar PSRAM'e gider.

## Modüller (`main/`)

```
app_main.cc        açılış sırası
pins.h             bütün pinler
i2c_bus  imu  motors  power  audio  camera  face     donanım
wifi  wifi_portal  settings  identity  status         sistem
console  crash                                          bakım
server_link.h      sunucu arayüzü  ←  xiaozhi_link (tek uygulama)
opus_codec  conversation  wake_word  tools              sohbet
fonts/             font_noto_sans_common_14_1.bin (altyazı fontu, bkz. fonts/README.txt)
font_bitmap        hazır bitmap fontu flash'tan yükler
```

## Derleme ve yükleme

ESP-IDF 5.5.2:

```bash
. ~/esp/esp-idf/export.sh
idf.py build                      # bağımlılıklar dependencies.lock ile indirilir
tools/ota.sh                      # Wi-Fi üzerinden (robot 192.168.1.125)
```

USB ile yalnızca uygulama (NVS, ayarlar ve assets bölümü korunur):

```bash
python -m esptool --chip esp32s3 -p /dev/cu.usbmodem* -b 921600 write_flash 0x20000 build/jarvis.bin
python -m esptool --chip esp32s3 -p /dev/cu.usbmodem* erase_region 0xd000 0x2000   # ota_0'dan aç
```

Boş bir karta tam kurulum: `idf.py -p /dev/cu.usbmodem* flash`. Sonra robot kurulum ağını açar, Wi-Fi telefondan seçilir.

Bölümler (`partitions.csv`, 16 MB): nvs 0x9000, otadata 0xd000, phy 0xf000, ota_0 0x20000, ota_1 0x410000 (4 MB'lık iki bölüm), assets 0x800000 (v1.7 kullanmaz).

## Bilinen konular

- OV2640 uzun süre kullanılmadığında (~1 saat) kare vermeyebiliyor; düzelmesi için güç kesip vermek gerekiyor. Sebebi henüz bulunmadı.
- Ev modemi (TurkTelekom, kanal 13) güç kesintisinden sonra eski bağlantıyı bir süre tutup yeni bağlantı isteklerini cevapsız bırakabiliyor; taze taramayla yeniden deneme bunu birkaç saniyeye indiriyor.
