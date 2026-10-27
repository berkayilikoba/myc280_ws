# Görev F: oda_dunya.py geliştirmesi — iç mekân ışığı, dokular, sade çarpışma mesh'i

Dosya: /home/gunes/limo_robot/limo_ws/saha/oda/oda_dunya.py (mevcut dosyayı DÜZENLE, mevcut davranışı koru).

1. IŞIK (zorunlu; tavan güneşi kesiyor, iç mekân kapkaranlık render ediliyor):
   - scene ambient 0.65 0.65 0.65 1, background 0.7 0.7 0.7 1.
   - tavanın 0.15 m altında, oda_siniri sınırlayıcı kutusunda --isik_aralik (2.0 m) ızgarasında, yalnız oda_siniri
     çokgeninin İÇİNDE kalan noktalara <light type="point">: diffuse 0.55 0.55 0.52 1, specular 0.1 0.1 0.1 1,
     attenuation range 8, constant 0.4, linear 0.08, quadratic 0.01, cast_shadows false. Işık sayısını yorumla yaz.
   - --isik_aralik 0 verilirse ışık ekleme.
2. DOKULAR (isteğe bağlı --doku_dizin D): D/doku.yaml (Görev D çıktısı) varsa:
   - duvarlar: doku.yaml'daki duvar i (p1, p2 aynı sırada, yuz_normali, malzeme "Oda/duvar_<i>") için duvar kutusunun
     visual'ında <material><script><uri>file://<mutlak D>/materials/scripts</uri><uri>file://<mutlak D>/materials/textures</uri>
     <name>Oda/duvar_<i></name></script></material> (renk yerine). Kutu dokusu her yüze aynı uygulanır — kabul.
     DAHA İYİ: dokulu yüz için kutu yerine ayrıca duvar yüzeyinde ince (0.002 m) bir plane/box visual ekle (yuz_normali
     yönünde kalinlik/2 + 0.001 içeride), ona dokuyu ver; kutunun kendisi düz duvar rengi kalsın.
   - zemin: doku.yaml zemin.kutu boyutunda ayrı ince visual (z = 0.001), malzeme Oda/zemin.
3. SADE ÇARPIŞMA: --carpisma_mesh <obj> verilirse mobilya_mesh collision'ı onu kullansın (visual ayrıntılı kalsın).
   Ayrıca küçük yardımcı fonksiyon/CLI: `oda_dunya.py --sadelestir <giris.obj> <cikis.obj> --ucgen 20000` open3d ile
   simplify_quadric_decimation yapıp düz OBJ (malzemesiz) yazsın (bu kullanımda dünya üretme).
4. Kabul: mevcut CLI aynen çalışır; --doku_dizin yoksa/bulunamazsa uyarı + düz renk; XML geçerli.
