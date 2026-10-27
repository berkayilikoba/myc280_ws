# `ajan` — Gemini (agy CLI) Alt Ajan Yöneticisi

`ajan`, Gemini (`agy` CLI) üzerinde arka planda alt ajan görevleri başlatmak, izlemek ve yapılandırılmış raporlar üretmek için geliştirilmiş hafif bir orkestrasyon aracıdır.

- **Bağımsızlık:** `agy-delegate` eklentisinden veya `agy-bridge` katmanından tamamen bağımsızdır; doğrudan `agy` ikilisini çağırır.
- **Sıfır Dış Bağımlılık:** Python 3.10+ ve yalnızca standart kütüphane (`os`, `sys`, `json`, `subprocess`, `threading` vb.) ile çalışır.
- **Süreç Güvenliği:** Süreç grupları (`start_new_session`) üzerinden yönetim sağlar; SIGTERM yanıt vermezse 5 saniye sonra SIGKILL ile zorlar.
- **İzin Ön Denetimi:** `settings.json` izin kurallarını çalıştırmadan önce kontrol eder, eksikse otomatik yedekleyerek ekleyebilir.

---

## 1. Ortam Değişkenleri

| Değişken | Varsayılan Değer | Açıklama |
|---|---|---|
| `$AJAN_AGY` | `agy` (PATH üzerinde) | Çağrılacak `agy` çalıştırılabilir ikilisinin yolu |
| `$AJAN_DIZIN` | `~/.local/state/ajan` | Görevlerin saklandığı durum kök dizini |
| `$AJAN_AYAR` | `~/.gemini/antigravity-cli/settings.json` | Gemini izin ayar dosyası |

---

## 2. Durum ve Dizin Mimarisi

Her görev, `$AJAN_DIZIN/<etiket>/` altında kendi yalıtılmış ortamında barındırılır:

```text
~/.local/state/ajan/<etiket>/
├── meta.json         # Durum, PID, model, son adım, token toplamı, tur geçmişi
├── GEMINI.md         # Sabit ajan kuralları + görev yönergesi
├── calisma/          # agy çalışma dizini (cwd)
├── olaylar.jsonl     # Ham stream-json olay akışı
├── yanit.md          # Modelin nihai yanıt metni
├── rapor.json        # Yapılandırılmış JSON raporu
└── hata.log          # agy stderr ve hata kayıtları
```

Eğer bir etiket canlı çalışırken yeniden `baslat` çağrılırsa işlem **kod 3** ile reddedilir. Görev daha önce bitmişse, önceki klasör `<etiket>.<zaman>` olarak otomatik arşivlenir.

---

## 3. İzin Yönetimi ve Ön Kontrol

Gemini `settings.json` içerisindeki izinler şu biçimdedir:
```json
{
  "permissions": {
    "allow": [
      "write_file(/home/user/limo_ws/saha)",
      "command(colcon)",
      "command(pytest)"
    ]
  }
}
```

- **Yazma İzni:** `write_file(<mutlak_dizin>)` kuralı o dizini ve altındaki tüm dosya/klasörleri kapsar.
- **Komut İzni:** `command(<önek>)` kuralı birebir eşleşmelidir.
- **Ön Kontrol:** `baslat` sırasında eksik izin tespit edilirse, eklenmesi gereken kurallar ekrana basılır ve program **çıkış kodu 2** ile sonlanır.
- **Otomatik Ekleme:** `--izin-ekle` bayrağı verilirse, mevcut `settings.json` dosyası `settings.json.bak-<zaman>` olarak yedeklenir ve eksik kurallar otomatik eklenerek göreve devam edilir.

---

## 4. Model Takma Adları

`--model` parametresinde aşağıdaki kısaltmalar kullanılabilir (varsayılan: `flash`):

- `flash` → `gemini-3.8-flash-high`
- `flash-orta` → `gemini-3.8-flash-medium`
- `argon` → `gemini-4-argon-high`
- `argon-orta` → `gemini-4-argon-medium`
- Diğer tüm tam model kimlikleri (ör. `claude-sonnet-5-5-high`, `gpt-oss-120b-medium`) doğrudan geçirilir.

---

## 5. Sabit Ajan Kuralları ve Raporlama JSON'u

`ajan`, görevi başlatırken `GEMINI.md` dosyasının başına aşağıdaki Türkçe bağlayıcı talimatları ekler:
1. Yalnız izin verilen `--yaz` dizinlerine yazabilir.
2. Yalnız izin verilen `--komut` listesindeki komutları çalıştırabilir.
3. Görev dışındaki dosyaları değiştiremez/silemez, var olan testleri gevşetemez.
4. Yanıtının sonuna aşağıdaki JSON bloğunu eklemelidir:

```json
```json
{
  "ozet": "Yapılan işin kısa özeti",
  "degisen_dosyalar": ["/workspace/kod.py"],
  "calistirilan_testler": ["pytest: PASSED"],
  "acik_konular": [],
  "basarili": true
}
```
```

`ajan` bekçi süreci bu bloğu ayrıştırıp `rapor.json` dosyasına dönüştürür. Blok bulunamazsa yazılan dosyalar ve yanıttan `basarili: null` olarak otomatik rapor türetilir.

---

## 6. Komut Referansı

### 6.1 `baslat` — Yeni Görev Başlatma
```bash
# Metin mesajıyla başlat ve arka planda çalıştır
python3 ajan.py baslat gorev1 -m "Kodu refaktör et" --yaz /proje/src --komut "pytest"

# Dosyadan talimat oku, model belirt ve bitene kadar bekle
python3 ajan.py baslat gorev2 -g talimat.md -m "Ek not" --model argon --effort high --bekle

# Eksik izinleri otomatik settings.json'a ekleyerek başlat
python3 ajan.py baslat gorev3 -m "Hızlı test" --yaz /proje --komut "colcon" --izin-ekle

# Terminali tamamen kapatmak için sandbox zorla
python3 ajan.py baslat gorev4 -m "Salt metin analizi" --sandbox
```

### 6.2 `devam` — Aynı Bağlamda Yeni Tur
Önceki turun `conversation_id` bağlamını koruyarak yeni bir tur başlatır:
```bash
python3 ajan.py devam gorev1 -m "Test sonuçlarına göre hata ayıklama yap" --bekle
```

### 6.3 `durum` — Görev Tablosu / JSON
```bash
# Tüm görevleri ASCII tablo olarak göster
python3 ajan.py durum

# Tek görevin durumunu göster
python3 ajan.py durum gorev1

# Otomasyon için JSON çıktısı al
python3 ajan.py durum --json
```

Tablo çıktısı:
```text
ETİKET    MODEL                  DURUM    TUR  SÜRE   SON ADIM                 DOSYA  TOKEN
-----------------------------------------------------------------------------------------
gorev1    gemini-3.8-flash-high  bitti    2    14.2s  write_file test_son.py   2      5420
gorev2    gemini-4-argon-high    calisiyor 1   5.1s   run_command pytest       1      1200
```
*Not: PID'si sonlanmış ancak meta kaydında 'calisiyor' kalmış görevler otomatik `kayip` durumuna düşer.*

### 6.4 `bekle` — Görevlerin Bitmesini Bekleme
```bash
# Belirli görevleri bekle (tümü bitti ise çıkış kodu 0, aksi 1)
python3 ajan.py bekle gorev1 gorev2

# 60 saniye zaman aşımı ile bekle
python3 ajan.py bekle --zaman-asimi 60
```

### 6.5 `rapor` — Rapor Görüntüleme
```bash
# Yapılandırılmış raporu okunur formatta bas
python3 ajan.py rapor gorev1

# Ajanın ürettiği ham son yanıtı (yanit.md) göster
python3 ajan.py rapor gorev1 --ham
```

### 6.6 `izle` — Canlı Olay Akışı
Olay akışını (`olaylar.jsonl`) canlı olarak `tail -f` benzeri takip eder:
```bash
python3 ajan.py izle gorev1
```

### 6.7 `iptal` — Görevi Durdurma
Süreç grubuna önce `SIGTERM` gönderir. 5 saniye içinde sonlanmazsa `SIGKILL` uygular:
```bash
python3 ajan.py iptal gorev1
```

### 6.8 `temizle` — Eski Görevleri Silme
Belirtilen günden daha eski tamamlanmış görev klasörlerini temizler (varsayılan: 7 gün):
```bash
python3 ajan.py temizle --gun 3
```

### 6.9 `izin kontrol` — İzinleri Bağımsız Denetleme
```bash
# Eksik izin kontrolü (eksikse kod 2, tamamsa 0)
python3 ajan.py izin kontrol --yaz /proje/yol --komut "pytest"

# Eksik izinleri settings.json'a otomatik ekleme
python3 ajan.py izin kontrol --yaz /proje/yol --komut "pytest" --izin-ekle
```

---

## 7. Testleri Çalıştırma

Testler gerçek `agy` ikilisini çağırmaz; sahte `stream-json` üreten izole bir `fake_agy` betiği üzerinden çalışır:

```bash
pytest saha/araclar/test_ajan.py -v
```
