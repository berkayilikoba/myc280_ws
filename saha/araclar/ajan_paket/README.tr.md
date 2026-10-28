# `ajan` — Gemini (agy CLI) Alt Ajan Yöneticisi

`ajan`, Google Gemini (`agy` CLI) üzerinde arka planda alt ajan görevleri başlatmak, izlemek, aynı bağlamda devam ettirmek ve yapılandırılmış raporlar üretmek için geliştirilmiş hafif bir orkestrasyon aracıdır. Özellikle **Claude Code** veya yerel/otomasyon orkestratörlerinin uygulama işlerini Gemini alt ajanlarına devretmesi için optimize edilmiştir.

> **Mimari Notu:** `ajan.py` tek dosyadır (`ajan.py tek dosyadır`). Hiçbir üçüncü taraf bağımlılığı yoktur; Python 3.10+ ve yalnızca standart kütüphane modülleri ile çalışır.

---

## 1. Ne İşe Yarar

Çoklu ajan ve eşli programlama (pair-programming) iş akışlarında, ana orkestratör (örneğin Claude Code veya bir yazılımcı) mimariyi tasarlar, kabul ölçütlerini belirler ve testleri yürütür; kodlama, yeniden düzenleme veya düzeltme işlerini ise alt ajana delege eder.

`ajan` bu işleyişi şu özelliklerle sağlar:
- **Doğrudan CLI Çağrısı:** Doğrudan `agy` ikilisini `--output-format stream-json` ile çalıştırır; IDE eklentisi veya harici sunucu katmanı gerektirmez.
- **Süreç Grubu Yalıtımı:** Görevleri arka planda bağımsız süreç gruplarında (`start_new_session`) yönetir; SIGTERM yanıt vermezse 5 saniye sonra SIGKILL ile sonlandırır.
- **İzin Ön Denetimi:** `settings.json` izin kurallarını başlatmadan önce kontrol eder; eksik izin nedeniyle görevin başarısız olmasını önler.
- **Durum Yönetimi ve Bağlam Koruma:** `conversation_id` değerini turlar arasında korur; ajanın aynı bağlamda düzeltme yapmasını sağlar.
- **Otomatik Test ve Doğrulama Döngüsü:** Yerleşik `dogrula` komutu ile yerel testleri çalıştırır; başarısızlık durumunda çıktıyı ajana geri besleyerek düzeltme turlarını otomatik yürütür.

---

## 2. Kurulum

`ajan.py` tek bir dosyadır ve `saha/araclar/ajan.py` konumunda bulunur.

### Doğrudan Kullanım
Kurulum yapmadan doğrudan çalıştırabilirsiniz:
```bash
python3 /path/to/saha/araclar/ajan.py --help
```

### CLI Paketi Olarak Kurulum
`saha/araclar/ajan.py` dosyasını bu paket köküne (`ajan_paket/`) kopyalayın:
```bash
cp /path/to/saha/araclar/ajan.py .
pip install .
```
Geliştirici modu (editable) için:
```bash
pip install -e .
```
Bu işlem sisteminize küresel `ajan` komutunu kaydeder.

---

## 3. Ortam Değişkenleri

| Değişken | Varsayılan Değer | Açıklama |
|---|---|---|
| `AJAN_AGY` | `agy` (PATH üzerinde) | Çağrılacak `agy` çalıştırılabilir ikilisinin yolu |
| `AJAN_DIZIN` | `$XDG_STATE_HOME/ajan` veya `~/.local/state/ajan` | Görevlerin saklandığı durum kök dizini |
| `AJAN_AYAR` | `~/.gemini/antigravity-cli/settings.json` | Gemini izin ayar dosyası |
| `AJAN_DIL` | `tr` | Çıktı ve ileti dili (`tr`: Türkçe, `en`: İngilizce) |

---

## 4. İzin Modeli

Gemini / Antigravity izinleri `settings.json` içerisinde şu biçimde tanımlanır:
```json
{
  "permissions": {
    "allow": [
      "write_file(/proje/dizini)",
      "command(pytest)",
      "command(git diff)"
    ]
  }
}
```

- **Yazma İzni:** `write_file(<mutlak_dizin>)` kuralı belirtilen dizini ve altındaki tüm alt dosya/klasörleri kapsar.
- **Komut İzni:** `command(<önek>)` kuralı komut önekleriyle birebir eşleşir.
- **Ön Denetim:** `ajan baslat` çalıştırıldığında `--yaz` ve `--komut` parametrelerinin `settings.json` tarafından kapsanıp kapsanmadığı denetlenir. Eksik izin varsa program **çıkış kodu 2** ile durur ve eklenmesi gereken kuralları listeler.
- **Otomatik İzin Ekleme:** `--izin-ekle` bayrağı verilirse, mevcut `settings.json` dosyası `settings.json.bak-<zaman>` olarak yedeklenir ve eksik kurallar otomatik eklenerek göreve başlanır.
- **Sandbox Modu:** Komut listesi boşsa (`--komut` verilmemişse), terminal kapatılarak `--sandbox` modu otomatik uygulanır.

---

## 5. stream-json ve Conversation Yapısı

`ajan`, arka plan bekçi süreci (`_bekci`) üzerinden `agy`'yi `--output-format stream-json` ile yönetir:

1. **`init` Olayı:** Oturum parametrelerini kaydeder ve `conversation_id` kimliğini alır.
2. **`step_update` Olayı:** Canlı adım akışını izler:
   - Dosya yazma araçları (`write_file`, `edit_file` vb.) tespit edildiğinde yazılan dosyalar `meta.json` içine eklenir.
   - Komut çalıştırma araçları (`run_command`) tespit edildiğinde çalıştırılan komutlar kaydedilir.
   - Token harcamaları (`input_tokens`, `output_tokens`, `total_tokens`) kümülatif güncellenir.
3. **`result` Olayı:** Nihai durum kodunu, yanıt metnini ve süreyi yakalar.
4. **Bağlam Koruma:** `devam` çağrıldığında veya `dogrula` turlarında `agy`'ye `--conversation <conversation_id>` aktarılarak aynı bağlam korunur.

---

## 6. Tipik Orkestratör Döngüsü

Birincil orkestratörün (ör. Claude Code) Gemini alt ajanını yönlendirdiği standart akış şöyledir:

```
[Claude Code] ──► 1. Görev Hazırla & Başlat ──► [ajan baslat ETIKET ...]
                        │
                        ▼
                  2. Bitişi Bekle           ──► [ajan bekle ETIKET] (arka plan)
                        │
                        ▼
                  3. Özeti / Raporu İncele  ──► [ajan rapor ETIKET]
                        │
                        ▼
                  4. Testleri Doğrula       ──► (Yerel test çalıştırma)
                        │
                 ┌──────┴──────┐
              Geçti          Kaldı
                 │             │
                 ▼             ▼
              [Bitti]    5. Geri Besleme     ──► [ajan devam ETIKET -m "..."]
                                                 (veya otomatik: ajan dogrula)
```

1. **Başlatma (`baslat`):**
   ```bash
   ajan baslat gorev1 -m "Ayrıştırıcı mantığını refaktör et" --yaz /proje/src --komut "pytest"
   ```
2. **Bekleme (`bekle`):**
   ```bash
   ajan bekle gorev1
   ```
3. **Raporlama (`rapor`):**
   ```bash
   ajan rapor gorev1
   ```
4. **Bağımsız Doğrulama Testi:**
   Orkestratör testleri kendisi çalıştırır.
5. **Düzeltme / Döngü (`devam` veya `dogrula`):**
   Test başarısız olursa aynı bağlamda devam ettirilir:
   ```bash
   ajan devam gorev1 -m "42. satırda KeyError alındı. Testleri gevşetmeden kök nedeni düzelt." --bekle
   ```
   Veya otomatik sınama ve düzeltme döngüsü:
   ```bash
   ajan dogrula gorev1 --test "pytest tests/test_parser.py" --tur 3
   ```

---

## 7. Komut Referansı

### `baslat`
Yeni bir arka plan alt ajan görevi başlatır.
```bash
ajan baslat <etiket> (-g <gorev_dosyasi...> | -m <mesaj>) [--model <model>] [--effort <effort>] [--yaz <dizin>] [--komut <önek>] [--sure <sure>] [--sandbox] [--izin-ekle] [--bekle]
```
- Desteklenen model takma adları: `flash` (`gemini-3.8-flash-high`), `flash-orta` (`gemini-3.8-flash-medium`), `argon` (`gemini-4-argon-high`), `argon-orta` (`gemini-4-argon-medium`) veya tam model kimlikleri.

### `devam`
Önceki turun konuşma bağlamını koruyarak yeni bir tur başlatır.
```bash
ajan devam <etiket> -m <takip_mesaji> [--bekle]
```

### `dogrula`
Belirtilen yerel test komutunu çalıştırır; başarısız olursa çıktıyı ajana ileterek en fazla `--tur` adedince otomatik düzeltme döngüsü yürütür.
```bash
ajan dogrula <etiket> --test "<komut>" [--tur <maks_tur>] [--cwd <calisma_dizini>] [--zaman-asimi <saniye>]
```

### `durum`
Tüm görevlerin veya tek bir görevin tablosal veya JSON özetini verir.
```bash
ajan durum [<etiket>] [--json]
```

### `bekle`
Belirtilen görevler tamamlanana kadar bekler. Hepsi başarılı bittiyse `0`, aksi halde `1` döner.
```bash
ajan bekle [<etiket>...] [--zaman-asimi <saniye>]
```

### `rapor`
Ajanın oluşturduğu yapılandırılmış JSON raporunu veya ham yanıtını gösterir.
```bash
ajan rapor <etiket> [--ham]
```

### `izle`
Canlı olay akışını (`olaylar.jsonl`) terminalde takip eder.
```bash
ajan izle <etiket>
```

### `iptal`
Çalışan bir göreve önce `SIGTERM`, 5 saniye içinde kapanmazsa `SIGKILL` göndererek sonlandırır.
```bash
ajan iptal <etiket>
```

### `temizle`
Belirtilen günden daha eski tamamlanmış görev kayıtlarını siler (varsayılan: 7 gün).
```bash
ajan temizle [--gun <gun_sayisi>]
```

### `izin kontrol`
`settings.json` izinlerini görev başlatmadan bağımsız denetler.
```bash
ajan izin kontrol [--yaz <dizin>] [--komut <önek>] [--izin-ekle]
```

---

## 8. Sınırlamalar

- **Yerel `agy` CLI Zorunluluğu:** Ortamda Google Antigravity (`agy`) CLI ikilisinin kurulu ve kimlik doğrulaması yapılmış olması gerekir.
- **Tek Makine / Yerel Süreç Sınırı:** Süreç yönetimi yerel süreç grupları (`killpg`) üzerinedir; ağ üzerinden dağıtık düğüm yönetimini içermez.
- **Sandbox Sınırları:** Komut çalıştırma izni verildiğinde `settings.json` dosyasına ilgili izin kurallarının eklenmiş olması zorunludur.

---

## 9. Lisans

lisans: belirlenecek
