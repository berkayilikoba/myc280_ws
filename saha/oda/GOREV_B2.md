# Görev B2: oda_yapi.py düzeltmesi — sahte duvar parçalarını ele

Dosya: /home/gunes/limo_robot/limo_ws/saha/oda/oda_yapi.py (mevcut dosyayı DÜZENLE; diğer davranışları koru).

Sorun (gerçek haritada görüldü): birleştirilmiş Hough parçalarından bazıları dolu hücreler boyunca değil, iki ayrı duvar
kümesi arasında çapraz geçiyor (sol duvarda X şeklinde kesişen iki parça; x≈-0.9 y≈1.9 civarında çapraz parça).

Düzeltme:
1. Her son duvar parçası için doluluk yoğunluğu: parça boyunca her 1 hücrede bir örnek nokta al; noktanın 1.5 hücre
   yarıçapında dolu hücre varsa "dolu say". oran = dolu sayılan / örnek. oran < --min_doluluk (0.6) olan parçayı at.
2. Kesişen parçalar: iki parça birbirini kesiyorsa (uç noktalardan > 0.1 m içeride) ve açı farkı > 20° ise, düşük
   doluluk oranlı olanı at.
3. YAML'daki her duvara "doluluk" alanı ekle (0-1, 3 basamak).
4. Özette atılan parça sayısını yaz.
CLI'ye --min_doluluk ekle. Kabul: lab_oda.yaml ile kalan duvarların hepsinin doluluk >= 0.6.
