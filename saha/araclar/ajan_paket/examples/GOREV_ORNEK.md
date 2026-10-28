# Görev: Kullanıcı Kimlik Doğrulama Modülü Refaktörü

## 1. Bağlam ve Amaç
Mevcut `src/auth/service.py` dosyasındaki token doğrulama mantığı eski şifreleme yöntemini kullanmaktadır. Bu görevde, token doğrulama mantığı güvenli standartlara geçirilecek ve süre aşımı kontrolü eklenecektir.

## 2. İlgili Dosyalar
- **Önce OKU:**
  - `src/auth/interfaces.py` (Arayüz ve veri yapıları)
  - `tests/test_auth.py` (Mevcut testler)
- **Değiştirilecek Dosyalar:**
  - `src/auth/service.py`
  - `tests/test_auth.py` (Yalnızca yeni test senaryoları eklenebilir)

## 3. Kabul Ölçütleri
1. `validate_token(token_str: str) -> AuthPayload` fonksiyonu geçersiz veya süresi dolmuş tokenlar için `TokenExpiredError` ya da `InvalidTokenError` fırlatmalıdır.
2. Var olan test senaryolarının hiçbiri gevşetilmemeli veya silinmemelidir.
3. Yeni süre aşımı mantığı için `tests/test_auth.py` dosyasına en az 2 birim test eklenmelidir.
4. Sıfır dış bağımlılık: Harici üçüncü taraf paket eklenmemeli, yalnızca standart Python kütüphaneleri kullanılmalıdır.
5. Görevde istenmeyen hiçbir dosyaya dokunulmamalıdır.

## 4. Tamamlama Raporu
İşlemi tamamladığında yanıtının en sonuna şu formatta bir JSON bloğu ekle:

```json
{
  "ozet": "Token doğrulama mantığı güvenli standartlara geçirildi ve süre aşımı kontrolleri eklendi.",
  "degisen_dosyalar": [
    "src/auth/service.py",
    "tests/test_auth.py"
  ],
  "calistirilan_testler": [
    "pytest tests/test_auth.py: 12 passed"
  ],
  "acik_konular": [],
  "basarili": true
}
```
