# Gece görevi N20: renk_algi uzak mesafede derinlik arka plana düşüyor (~18 cm menzil sıçraması)
ÖNEMLİ: terminal iznin YOK; yalnız dosya oku/yaz. Claude test edip simde ölçecek.

## Ölçüm (Claude, saha/gece/algi_sim_test.sh, zemindeki elma r=0.035 m, kaydedici ölçüm anındaki gerçek pozla)
Robot gövde çerçevesinde gerçek vs ölçülen (x ileri):
- 2.3 m'den uzakta ölçümlerin ~%60'ı menzilde +0.15…+0.20 m İLERİ (ör. gerçek 3.51 → 3.70, 2.72 → 2.90);
  diğerleri +0.03…+0.05. Yanal (y) hata hep ~1 cm. 2.3 m'den yakında sıçrama yok.
- Ayrıca menzille büyüyen sistematik +ileri sapma: 0.83 m'de +2.4 cm, 1.5 m'de +3 cm, 3 m'de +4 cm.
Yorum: uzakta elma ~10 px yarıçaplı; 7x7 derinlik penceresi (derinlik_medyan_al, renk_algi.py:472) leke
kenarına/arkasındaki zemine taşıyor, medyan arka plana kayıyor. Sistematik kısmın bir bölümü yüzey→merkez
düzeltmesinin yokluğu olabilir: derinlik nesnenin ÖN YÜZEYİNİ ölçer; merkez ~r_nesne daha uzaktadır —
bu sapmanın tersine olurdu, o yüzden yüzey/merkez düzeltmesinin şu an nasıl yapıldığını kodda kontrol et ve
modül belgesinde açıkça yaz (varsa nerede, yoksa neden).

## İstenen (src/limo_autonomy/limo_autonomy/renk_algi.py + test/test_renk_algi.py)
1. Derinliği pencere yerine LEKE MASKESİ içindeki geçerli piksellerden al (renk→derinlik eşlemesi mevcut iki
   aşamalı yapıyla; maskeyi derinlik görüntüsüne eşlemek pahalıysa leke kutusunun iç bölgesini kullan, kenardan
   %25 içeri). Ön yüzeye en yakın sağlam değer: düşük yüzdelik (parametre `derinlik_yuzdelik`, vars. 30), medyan
   değil. Piksel sayısı < `min_derinlik_piksel` (vars. 6) ise eski pencere-medyan yoluna dön.
2. Tutarlılık: seçilen derinliğin etrafındaki ±`derinlik_bant_m` (vars. 0.05) içindeki piksel oranı <%40 ise
   ölçümü yayınlama (karışık arka plan) — durum metni değişmesin, yalnız sayaç/hata ayıklama günlüğü.
3. `maks_menzil_m` parametresi (vars. 3.0): bunun ötesindeki ölçümler yayınlanmaz (kavrama son yaklaşması
   ~1.5 m içinde başlar; uzak ölçüm yalnız NAV_Q planına yardım eder).
4. Saf fonksiyonlar + testler: iki modlu derinlik dizisinde (ön yüzey 2.00 m %50, arka plan 2.18 m %50)
   yüzdelik seçiminin ~2.00 döndüğü; arka plan ağırlıklıysa (%80) tutarlılık reddi; az piksel → yedek yol;
   maks_menzil reddi. Mevcut testleri kırma.
Önce renk_algi.py ve test_renk_algi.py'yi TAMAMEN OKU (N17 eklemeleri: kenar_payi_px, boyut_denetimi var).
