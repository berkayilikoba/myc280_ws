# Görev AJAN2: ajan.py düzeltmeleri — 15 testin 6'sı kalıyor (Claude çalıştırdı)
Dosyalar: saha/araclar/ajan.py, saha/araclar/test_ajan.py. Testleri GEVŞETME; test hatalıysa nedenini yanıtta açıkla.

Claude teşhisi (kök neden 1): YARIŞ. baslat meta'yı "pid": None ile yazıp bekçiyi Popen ile başlatıyor; agy'yi
bekçi başlatıp pid'i sonra yazıyor. bekle/durum hemen bakınca is_pid_alive(None) == False → "kayip".
Düzeltme: baslat bekçinin pid'ini (Popen(...).pid) meta["bekci_pid"]'e HEMEN yazsın. Canlılık = bekci_pid canlı
VEYA pid canlı. pid None ve bekçi canlıysa "calisiyor (başlıyor)". Bekçi bitişte meta'yı son kez yazıp çıkar;
bekçi ölü + durum "calisiyor" → "kayip".
Kök neden 2 (olası): meta.json eşzamanlı yazılıyor/okunuyor. save_meta atomik olsun (aynı dizinde geçici dosya +
os.replace); load_meta JSON hatasında 3 kez 0.05 s arayla yeniden denesin.
Diğer başarısızlıklar (aşağıdaki çıktı): her birini kök nedenine kadar izle ve düzelt.

Pytest çıktısı (özet):
_______________________ test_baslat_bekle_bitti_ve_rapor _______________________
E       AssertionError: Hata çıktısı: 
E       assert 1 == 0
E        +  where 1 = CompletedProcess(args=['/usr/bin/python3', '/home/gunes/limo_robot/limo_ws/saha/araclar/ajan.py', 'baslat', 'gorev1', '-m', 'İlk deneme', '--bekle'], returncode=1, stdout='[gorev1] kayip (süreç y
_______________________ test_izin_ekle_yedekleyip_ekler ________________________
E       assert 1 == 0
E        +  where 1 = CompletedProcess(args=['/usr/bin/python3', '/home/gunes/limo_robot/limo_ws/saha/araclar/ajan.py', 'baslat', 'izin_ekle...n_ekle_dizin)\n  + command(pytest -k unit)\n[izin_ekle_gorev] kayip (süreç 
__________________ test_devam_ikinci_turda_conversation_gecer __________________
E       AssertionError: assert 1 == 0
E        +  where 1 = CompletedProcess(args=['/usr/bin/python3', '/home/gunes/limo_robot/limo_ws/saha/araclar/ajan.py', 'baslat', 'tur_gorev...1. Tur İsteği', '--bekle'], returncode=1, stdout='[tur_gorev] kayip (süre�
_________________________ test_hata_sonucu_hata_durumu _________________________
E       AssertionError: assert 'calisiyor' == 'hata'
E         
E         - hata
E         + calisiyor
____________________ test_model_takma_adlari_cli_uzerinden _____________________
E       FileNotFoundError: [Errno 2] No such file or directory: '/tmp/pytest-of-gunes/pytest-17/test_model_takma_adlari_cli_uz0/fake_agy.log'
___________________ test_yazilan_dosya_yolu_cikarimi_akista ____________________
E       AssertionError: assert 1 == 0
E        +  where 1 = CompletedProcess(args=['/usr/bin/python3', '/home/gunes/limo_robot/limo_ws/saha/araclar/ajan.py', 'baslat', 'farkli_an...ti', '--bekle'], returncode=1, stdout='[farkli_anahtar_gorev] kayip (süreç 
FAILED test_ajan.py::test_baslat_bekle_bitti_ve_rapor - AssertionError: Hata ...
FAILED test_ajan.py::test_izin_ekle_yedekleyip_ekler - assert 1 == 0
FAILED test_ajan.py::test_devam_ikinci_turda_conversation_gecer - AssertionEr...
FAILED test_ajan.py::test_hata_sonucu_hata_durumu - AssertionError: assert 'c...
FAILED test_ajan.py::test_model_takma_adlari_cli_uzerinden - FileNotFoundErro...
FAILED test_ajan.py::test_yazilan_dosya_yolu_cikarimi_akista - AssertionError...

Kodu çalıştıramazsın; Claude testleri yeniden çalıştıracak. Kısa özetle bitir: her başarısız test için neden + düzeltme.
