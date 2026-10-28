# Gece görevi F5: görev yöneticisine pil farkındalığı ve şarja dönüş (karar katmanı)
ÖNEMLİ: terminal/komut çalıştırma iznin YOK; yalnız dosya oku/yaz. Claude test edecek.
Dosyalar: src/limo_autonomy/limo_autonomy/gorev_yoneticisi.py (saf durum makinesi), gorev_dugumu.py,
src/limo_autonomy/config/nesneler_lab_oda.yaml, src/limo_autonomy/test/test_gorev_yoneticisi.py (mevcut testleri
bozma; ekle). kavrama_dugumu.py'ye DOKUNMA (başka görev). Önce bu dosyaları OKU.
Gerçek robot: limo_base `/limo_status` (limo_msgs/msg/LimoStatus: header, vehicle_state, control_mode,
float64 battery_voltage, error_code, motion_mode) yayınlar. LIMO 3S Li-ion: tam ~12.6 V, nominal 11.1 V.
1. Yaml: `sarj: {x, y, yaw_derece}` (lab_oda: ev ile aynı (0,0,0) olabilir) ve `pil: {uyari_v: 11.0, kritik_v: 10.5,
   tampon_s: 10}` (gerilim tampon_s boyunca eşik altında kalırsa olay; ani düşüşlerde tetiklenme).
2. Durum makinesi (saf): yeni olay türü 'pil' (veri: gerilim, t). Kural:
   - BOSTA'da gerilim < uyari_v → yeni görev isteklerini REDDET ('HATA pil düşük'), SARJA_GIT başlat.
   - Görev sürerken uyari_v altı: mevcut görevi BİTİR (yeni görev kabul etme), görev bitince SARJA_GIT.
   - kritik_v altı: KAVRAMAYA_GIT ya da KOL_KAVRA öncesindeyse görevi iptal et (kavrama_iptal) ve SARJA_GIT; nesne
     TUTULUYORSA (KOL_KAVRA başarılı, TASI/KOL_BIRAK) nesneyi BIRAKMADAN doğrudan SARJA_GIT (taşıma hedefi şarj
     noktası değilse bile; durum ayrıntısına 'nesne tutuluyor' yaz).
   - SARJA_GIT başarısızsa 2 kez yeniden dene, sonra 'BASARISIZ sarj' ve BOSTA'da kal (istek reddi sürer).
   - SARJDA durumu: gerilim uyari_v + 0.4 V'u geçince BOSTA'ya dön ve istekleri yeniden kabul et.
   Tüm geçişler gecmis'e.
3. gorev_dugumu.py: `/limo_status` aboneliği (limo_msgs import edilemezse uyar ve pil özelliğini kapat), ayrıca
   simde test için `/gorev/sahte_pil` (std_msgs/Float32) aboneliği (parametre `sahte_pil`, vars. false) — biri
   'pil' olayını besler. Durum mesajlarında pil gerilimi (1 Hz özet).
4. Testler: BOSTA'da düşük pil → istek reddi + SARJA_GIT; görev sürerken uyarı → görev biter sonra şarj; kritik +
   nesne tutuluyor → bırakmadan şarj; kritik + kavrama öncesi → iptal + şarj; tampon_s'den kısa düşüş tetiklemez;
   şarj sonrası eşik üstü → BOSTA ve istek kabul.
