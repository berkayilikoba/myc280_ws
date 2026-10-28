---
name: ajan
description: Gemini alt ajanı ile çalışma (`agy` CLI üzerinde `ajan` aracı). Uygulama işini Gemini'ye devretmek, durumunu izlemek, aynı konuşmada düzeltme istemek ve sonucu doğrulamak için kullan. Kullanıcı "geminiye ver", "flash'a ver", "alt ajana ver", "delege et" dediğinde ya da büyük/tekrarlı uygulama işlerinde.
---

# ajan — Gemini Alt Ajanı ile Çalışma Rehberi

Araç: `ajan` (sistem `PATH` üzerinde veya `python3 ajan.py`). `agy-delegate` eklentisinden bağımsızdır; doğrudan `agy` CLI'ı `stream-json` ile çağırır.
Rol paylaşımı: **Orkestratör (Claude Code) planlar, görev tanımını ve kabul ölçütlerini yazar, testleri çalıştırır, denetler; ajan uygular.**

## Model Seçimi
- `--model flash` (`gemini-3.8-flash-high`): varsayılan, çoğu geliştirme ve refaktör işi.
- `--model argon` (`gemini-4-argon-high`): zorlu görevler (mimari tasarım, karmaşık matematik/algoritma).
- Kullanıcı aksini belirtmedikçe varsayılan olarak `flash` tercih edilir.

## Standart Akış
1. **Görev dosyası hazırla** (`tasks/GOREV_X.md` veya `tasks/TASK_X.md`):
   - Bağlam ve okunacak referans dosyaları belirt.
   - Değiştirilecek dosyaları ve beklenen arayüzleri/imzaları açıkça tanımla.
   - Kabul ölçütlerini, test beklentilerini ve "başka dosyaya dokunma" kuralını ekle.
2. **Başlat** (arka planda çalışır, hemen döner):
   ```bash
   ajan baslat ETIKET -g tasks/GOREV_X.md --model flash --yaz <hedef_dizin> [--oku <referans_dizin>]
   ```
   - Yazma izni yoksa çıkış kodu 2 verir ve eksik kuralı bildirir; onaylı çalışma alanı ise `--izin-ekle` bayrağı ile başlat.
   - Komut izni (`--komut "pytest"`) yalnızca kesinlikle gerekliyse verilmelidir; komut belirtilmezse terminal sandbox korumasıyla kapatılır.
   - Aynı etiket aktif çalışırken ikinci başlatma engellenir (çıkış kodu 3).
3. **Bitişi bekle**:
   ```bash
   ajan bekle ETIKET
   ```
   - Ara durum kontrolü: `ajan durum` (son adım, yazılan dosya sayısı, token).
   - Canlı olay akışı: `ajan izle ETIKET`.
4. **Denetle ve Test Et**:
   ```bash
   ajan rapor ETIKET
   ```
   - Rapor çıktısındaki değişen dosyaları incele (`git diff`).
   - Testleri orkestratör ortamında bizzat çalıştır.
5. **Düzeltme Döngüsü**:
   - Testler başarısız olursa yeni görev açmak yerine aynı konuşmada devam et (bağlam korunur):
     ```bash
     ajan devam ETIKET -m "Test çıktısı: <çıktı özeti>. Kök nedeni düzelt, testleri gevşetme." --bekle
     ```
   - Ya da otomatik test ve düzeltme döngüsünü başlat:
     ```bash
     ajan dogrula ETIKET --test "pytest tests/test_kod.py" --tur 3
     ```
6. **İptal ve Temizlik**:
   - Çalışan görevi durdurmak için: `ajan iptal ETIKET`
   - Eski görevleri temizlemek için: `ajan temizle --gun 7`

## Paralellik
- Farklı dosyalara dokunan görevler eşzamanlı olarak paralel başlatılabilir.
- Aynı dosyaları değiştiren iki görevi paralel çalıştırma.
- Bağımlı görevlerde, arayüzü görev metninde sabitleyerek paralel geliştirme yapılabilir.

## Temel Kurallar
- Kullanıcının doğrudan açtığı etkileşimli `agy` oturumuna müdahale etme.
- İzinleri her zaman en dar dizin ve net komut önekleriyle ver.
- Alt ajanın yazdığı testlerde sınır değerlerin veya kabul ölçütlerinin gevşetilmediğini mutlaka denetle.
