# Görev J3: oda_doku.py — zemin boşluk doldurma yamasını düzelt

Dosya: saha/oda/oda_doku.py (yalnız bu). J1'de eklenen `--zemin_doldur yama` davranışını düzelt.

Sorun (Claude gözlemi, cikti/doku_j1/doku/zemin.png): seçilen 64x64 yama "en keskin" bölge olduğu için içinde
siyah bant çizgisi ve beyaz parlama var; aynı yama düz ızgara halinde döşenince belirgin kareli desen oluşuyor.

İstenen:
1. Yama seçimi: görülen ve tamamen geçerli (maske 1) bölgeler arasından, yerel ortalama rengi zemin medyan
   rengine yakın (Lab ΔE < 12) ve içinde güçlü kenar OLMAYAN (Canny kenar pikseli oranı < %2), parlaklık
   standart sapması düşük bölgeleri aday say; adaylar arasından ağırlığı yüksek olanı seç. Yama boyutu
   `--yama_boyut` (vars. 96 teksel).
2. Birden çok aday (en iyi 4–8, birbirinden en az bir yama boyutu uzak) seç; döşerken her hücre için rastgele
   (sabit tohum 0) aday + rastgele yatay/dikey yansıma uygula; hücreler arasında %25 örtüşmeyle doğrusal
   karıştırma (feathering) yap ki ızgara izi kalmasın.
3. Döşenen dolgunun ortalama rengini, en yakın görülen bölgenin yerel ortalama rengine (büyük Gauss bulanık
   görülen doku, sigma ~40 teksel) göre ölçekle ki geçişte ton sıçraması olmasın; dolgu ile görülen bölge
   sınırında 10 teksel yumuşak kenar.
4. Uygun aday bulunamazsa eski medyan+blur yöntemine düş ve uyar.
Kodu çalıştıramazsın; Claude test edecek. Kısa özetle bitir.
