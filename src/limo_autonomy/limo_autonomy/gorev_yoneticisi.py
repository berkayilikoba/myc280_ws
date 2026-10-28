"""LIMO + myCobot görev yöneticisi: 'nesneyi al, getir' karar katmanı durum makinesi.

ROS'tan bağımsız, olay güdümlü saf Python modülü.
Saat (t) dışarıdan verilir.

Durum Akışı:
  BOSTA -> KAVRAMAYA_GIT (kavrama hedefi gönder) -> [HAZIR] KOL_KAVRA -> [başarılı]
  TASI (birakma pozuna NavigateToPose) -> [başarılı] KOL_BIRAK -> [başarılı]
  EVE_DON -> BOSTA (sonuç 'TAMAM').
  SARJA_GIT -> SARJDA -> BOSTA (şarj tamamlandığında).

Hata ve Yeniden Deneme Politikası:
  - Kavrama HATA: 1 kez yeniden dene (yeniden hedef gönder), sonra 'BASARISIZ kavrama'.
  - Kol kavra başarısız: kavramayı 1 kez yeniden dene, sonra 'BASARISIZ kavrama'.
  - Taşıma başarısız: 2 kez yeniden dene (toplam 3 deneme), sonra 'BASARISIZ tasima'
    ve olduğun yerde KOL_BIRAK yapmadan eve dönmeyi dene.
  - Şarj başarısız: 2 kez yeniden dene (toplam 3 deneme), sonra 'BASARISIZ sarj'
    ve BOSTA durumunda kal (yeni istek reddi sürer).
  - İptal her durumda: aktif eylemi iptal et ve BOSTA durumuna geç.
  - Pil yönetimi:
    * BOSTA'da gerilim < uyari_v -> istekleri REDDET ('HATA pil düşük'), SARJA_GIT başlat.
    * Görev sürerken uyari_v altı: mevcut görevi BİTİR, bitince SARJA_GIT.
    * kritik_v altı: KAVRAMAYA_GIT / KOL_KAVRA öncesindeyse görevi iptal et (kavrama_iptal) ve SARJA_GIT;
      nesne TUTULUYORSA nesneyi BIRAKMADAN doğrudan SARJA_GIT ('SARJA_GIT nesne tutuluyor').
    * SARJDA durumunda: gerilim > uyari_v + 0.4 V olunca BOSTA'ya dön ve istekleri yeniden kabul et.
    * Gerilim eşiklerinin tetiklenmesi için gerilimin tampon_s boyunca eşik altında kalması gerekir.
  - Her durum geçişi `gecmis` listesine (zaman, eski, yeni, neden) eklenir.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import yaml


@dataclass(frozen=True)
class Poz2D:
    """2B düzlemsel hedef poz (x, y, yaw_derece)."""

    x: float
    y: float
    yaw_derece: float = 0.0

    def __getitem__(self, item: Any) -> float:
        if item == 0 or item == 'x':
            return self.x
        if item == 1 or item == 'y':
            return self.y
        if item == 2 or item in ('yaw_derece', 'yaw'):
            return self.yaw_derece
        raise KeyError(item)


@dataclass(frozen=True)
class NesneKonumu:
    """Harita çerçevesinde 3B nesne koordinatı."""

    x: float
    y: float
    z: float = 0.0

    def __getitem__(self, item: Any) -> float:
        if item == 0 or item == 'x':
            return self.x
        if item == 1 or item == 'y':
            return self.y
        if item == 2 or item == 'z':
            return self.z
        raise KeyError(item)


@dataclass
class Ayar:
    """Görev yöneticisi aşama zaman aşımları ve yeniden deneme sınırları."""

    kavrama_zaman_asimi: float = 240.0
    tasima_zaman_asimi: float = 180.0
    kol_zaman_asimi: float = 30.0
    eve_don_zaman_asimi: float = 180.0
    sarj_zaman_asimi: float = 180.0
    max_kavrama_yeniden_dene: int = 1
    max_tasima_yeniden_dene: int = 2
    max_sarj_yeniden_dene: int = 2


@dataclass
class PilAyari:
    """Pil koruma eşikleri ve tampon süresi."""

    uyari_v: float = 11.0
    kritik_v: float = 10.5
    tampon_s: float = 10.0


# Uyumluluk takma adı
PilAyar = PilAyari


@dataclass(frozen=True)
class Eylem:
    """Durum makinesinin dış dünyaya ürettiği eylem komutu."""

    tur: str  # 'kavrama_hedef'|'kavrama_iptal'|'kol_kavra'|'kol_birak'|'nav_git'|'nav_iptal'|'durum'
    veri: Any = None


class GorevYoneticisi:
    """Nesne alma, getirme ve şarj görevlerini yöneten durum makinesi."""

    GECERLI_EYLEM_TURLERI = {
        'kavrama_hedef',
        'kavrama_iptal',
        'kol_kavra',
        'kol_birak',
        'nav_git',
        'nav_iptal',
        'durum',
    }

    def __init__(
        self,
        nesneler: Dict[str, Any],
        ev: Any,
        birakma: Optional[Any] = None,
        sarj: Optional[Any] = None,
        ayar: Optional[Any] = None,
        pil_ayar: Optional[Any] = None,
        pil: Optional[Any] = None,
    ) -> None:
        """Görev yöneticisini başlatır."""
        self.nesneler: Dict[str, NesneKonumu] = {}
        for k, v in nesneler.items():
            if isinstance(v, NesneKonumu):
                self.nesneler[k] = v
            elif isinstance(v, dict):
                self.nesneler[k] = NesneKonumu(float(v['x']), float(v['y']), float(v.get('z', 0.0)))
            elif isinstance(v, (list, tuple)):
                self.nesneler[k] = NesneKonumu(float(v[0]), float(v[1]), float(v[2]) if len(v) > 2 else 0.0)
            else:
                raise ValueError(f"Geçersiz nesne konumu formatı: {k} -> {v}")

        self.ev: Poz2D = self._normalize_poz(ev)
        self.birakma: Poz2D = self._normalize_poz(birakma) if birakma is not None else self.ev
        self.sarj: Poz2D = self._normalize_poz(sarj) if sarj is not None else self.ev

        if ayar is None:
            self.ayar = Ayar()
        elif isinstance(ayar, dict):
            self.ayar = Ayar(**ayar)
        elif isinstance(ayar, Ayar):
            self.ayar = ayar
        else:
            raise ValueError(f"Geçersiz ayar tipi: {type(ayar)}")

        secilen_pil = pil_ayar if pil_ayar is not None else pil
        if secilen_pil is None:
            self.pil_ayar = PilAyari()
        elif isinstance(secilen_pil, dict):
            self.pil_ayar = PilAyari(**secilen_pil)
        elif isinstance(secilen_pil, PilAyari):
            self.pil_ayar = secilen_pil
        else:
            raise ValueError(f"Geçersiz pil ayarı tipi: {type(secilen_pil)}")

        self.durum: str = 'BOSTA'
        self.gecmis: List[Tuple[float, str, str, str]] = []
        self.aktif_nesne: Optional[str] = None
        self.asama_baslangic: float = 0.0
        self.kavrama_deneme: int = 0
        self.tasima_deneme: int = 0
        self.sarj_deneme: int = 0
        self._tasima_hatasi: bool = False

        # Pil durumu değişkenleri
        self.son_gerilim: Optional[float] = None
        self._uyari_baslangic: Optional[float] = None
        self._kritik_baslangic: Optional[float] = None
        self._pil_uyari: bool = False
        self._pil_kritik: bool = False
        self._gorev_bitince_sarja_git: bool = False
        self._sarj_basarisiz: bool = False
        self._nesne_tutuluyor: bool = False

    @property
    def pil_gerilim(self) -> Optional[float]:
        """En son okunan pil gerilimi."""
        return self.son_gerilim

    @staticmethod
    def _normalize_poz(p: Any) -> Poz2D:
        if isinstance(p, Poz2D):
            return p
        if isinstance(p, dict):
            return Poz2D(float(p['x']), float(p['y']), float(p.get('yaw_derece', p.get('yaw', 0.0))))
        if isinstance(p, (list, tuple)):
            return Poz2D(float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0)
        raise ValueError(f"Geçersiz poz formatı: {p}")

    @staticmethod
    def _is_success(veri: Any) -> bool:
        if isinstance(veri, bool):
            return veri
        if isinstance(veri, (int, float)):
            return veri != 0
        if isinstance(veri, str):
            return veri.strip().lower() in ('true', '1', 'basarili', 'tamam', 'ok', 'succeeded', 'success')
        return False

    def _gecis(self, yeni: str, t: float, neden: str) -> None:
        """Durum geçişini uygular ve geçmişe kaydeder."""
        eski = self.durum
        self.durum = yeni
        self.asama_baslangic = float(t)
        self.gecmis.append((float(t), eski, yeni, str(neden)))

    def _istek_reddedilmeli_mi(self) -> bool:
        """Düşük pil veya şarj sürecinde yeni istek reddi kontrolü."""
        if self._pil_uyari or self._pil_kritik or self._sarj_basarisiz:
            return True
        if self.durum in ('SARJA_GIT', 'SARJDA'):
            return True
        return False

    def istek(self, ad: str, t: float = 0.0) -> List[Eylem]:
        """Kullanıcıdan gelen nesne alma isteğini veya iptal komutunu işler."""
        if ad == 'iptal':
            return self.olay('iptal', None, t)

        if self._istek_reddedilmeli_mi():
            return [Eylem(tur='durum', veri='HATA pil düşük')]

        if self.durum != 'BOSTA':
            return [Eylem(tur='durum', veri='HATA mesgul')]

        if ad not in self.nesneler:
            return [Eylem(tur='durum', veri='HATA bilinmeyen nesne')]

        self.aktif_nesne = ad
        self.kavrama_deneme = 0
        self.tasima_deneme = 0
        self._tasima_hatasi = False
        self._nesne_tutuluyor = False
        self._gecis('KAVRAMAYA_GIT', t, neden=f'istek: {ad}')

        return [
            Eylem(tur='durum', veri=f'KAVRAMAYA_GIT {ad}'),
            Eylem(tur='kavrama_hedef', veri=self.nesneler[ad]),
        ]

    def olay(self, tur: str, veri: Any = None, t: float = 0.0) -> List[Eylem]:
        """Sistemden gelen olayları işler (kavrama_durum, kol_sonuc, nav_sonuc, iptal, zaman, pil)."""
        if tur == 'iptal':
            return self._iptal_et(t)

        if tur == 'pil':
            return self._olay_pil(veri, t)

        if tur == 'kavrama_durum':
            return self._olay_kavrama_durum(veri, t)

        if tur == 'kol_sonuc':
            return self._olay_kol_sonuc(veri, t)

        if tur == 'nav_sonuc':
            return self._olay_nav_sonuc(veri, t)

        if tur == 'zaman':
            return self._olay_zaman(t)

        return []

    def _iptal_et(self, t: float) -> List[Eylem]:
        """Tüm eylemleri durdurup BOSTA durumuna geçer."""
        if self.durum == 'BOSTA':
            return [Eylem(tur='durum', veri='BOSTA')]

        eylemler: List[Eylem] = []
        if self.durum == 'KAVRAMAYA_GIT':
            eylemler.append(Eylem(tur='kavrama_iptal', veri=None))
        elif self.durum in ('TASI', 'EVE_DON', 'SARJA_GIT'):
            eylemler.append(Eylem(tur='nav_iptal', veri=None))

        self.aktif_nesne = None

        if self._pil_uyari or self._pil_kritik:
            self.sarj_deneme = 0
            self._gecis('SARJA_GIT', t, neden='iptal sonrasi pil dusuk, sarja gidiliyor')
            eylemler.append(Eylem(tur='durum', veri='IPTAL'))
            durum_metni = 'SARJA_GIT nesne tutuluyor' if self._nesne_tutuluyor else 'SARJA_GIT'
            eylemler.append(Eylem(tur='durum', veri=durum_metni))
            eylemler.append(Eylem(tur='nav_git', veri=self.sarj))
            return eylemler

        self._gecis('BOSTA', t, neden='iptal')
        eylemler.append(Eylem(tur='durum', veri='IPTAL'))
        return eylemler

    def _olay_pil(self, veri: Any, t: float) -> List[Eylem]:
        """Pil gerilimi olayını işler, eşik ve tampon kontrollerini yürütür."""
        gerilim = None
        if isinstance(veri, (int, float)):
            gerilim = float(veri)
        elif isinstance(veri, (list, tuple)):
            if len(veri) > 0 and isinstance(veri[0], (int, float)):
                gerilim = float(veri[0])
            if len(veri) > 1 and isinstance(veri[1], (int, float)) and t == 0.0:
                t = float(veri[1])
        elif isinstance(veri, dict):
            for k in ('gerilim', 'battery_voltage', 'voltage', 'v'):
                if k in veri and isinstance(veri[k], (int, float)):
                    gerilim = float(veri[k])
                    break
            if 't' in veri and isinstance(veri['t'], (int, float)) and t == 0.0:
                t = float(veri['t'])

        if gerilim is None:
            return []

        self.son_gerilim = gerilim

        # SARJDA durumu: gerilim uyari_v + 0.4 V'u geçince BOSTA'ya dön ve istekleri yeniden kabul et
        if self.durum == 'SARJDA':
            if gerilim > (self.pil_ayar.uyari_v + 0.4):
                self._pil_uyari = False
                self._pil_kritik = False
                self._gorev_bitince_sarja_git = False
                self._sarj_basarisiz = False
                self._uyari_baslangic = None
                self._kritik_baslangic = None
                self._nesne_tutuluyor = False
                self._gecis('BOSTA', t, neden='sarj tamamlandi')
                return [Eylem(tur='durum', veri='BOSTA')]
            return []

        # Başarısız şarj sonrası gerilim yükselmişse durumu temizle
        if self.durum == 'BOSTA' and self._sarj_basarisiz:
            if gerilim > (self.pil_ayar.uyari_v + 0.4):
                self._pil_uyari = False
                self._pil_kritik = False
                self._gorev_bitince_sarja_git = False
                self._sarj_basarisiz = False
                self._uyari_baslangic = None
                self._kritik_baslangic = None
                self._nesne_tutuluyor = False
                return [Eylem(tur='durum', veri='BOSTA')]

        # Eşik ve tampon süresi takibi
        if gerilim < self.pil_ayar.kritik_v:
            if self._kritik_baslangic is None:
                self._kritik_baslangic = t
        else:
            self._kritik_baslangic = None

        if gerilim < self.pil_ayar.uyari_v:
            if self._uyari_baslangic is None:
                self._uyari_baslangic = t
        else:
            self._uyari_baslangic = None

        tampon = self.pil_ayar.tampon_s

        kritik_tetiklendi = (
            self._kritik_baslangic is not None
            and (t - self._kritik_baslangic) >= tampon
        )
        uyari_tetiklendi = (
            self._uyari_baslangic is not None
            and (t - self._uyari_baslangic) >= tampon
        )

        if kritik_tetiklendi:
            if not self._pil_kritik:
                self._pil_kritik = True
                self._pil_uyari = True
                return self._tetikle_kritik_pil(t)
            return []

        if uyari_tetiklendi:
            if not self._pil_uyari:
                self._pil_uyari = True
                return self._tetikle_uyari_pil(t)
            return []

        return []

    def _tetikle_kritik_pil(self, t: float) -> List[Eylem]:
        """Kritik gerilim eşiği tetiklendiğinde uygulanacak geçişler."""
        eski_durum = self.durum

        if eski_durum in ('SARJA_GIT', 'SARJDA'):
            return []

        if eski_durum == 'BOSTA':
            self.sarj_deneme = 0
            self._gecis('SARJA_GIT', t, neden='kritik pil, sarja gidiliyor')
            return [
                Eylem(tur='durum', veri='SARJA_GIT'),
                Eylem(tur='nav_git', veri=self.sarj),
            ]

        # KAVRAMAYA_GIT ya da KOL_KAVRA öncesindeyse görevi iptal et (kavrama_iptal) ve SARJA_GIT
        if eski_durum in ('KAVRAMAYA_GIT', 'KOL_KAVRA'):
            self.aktif_nesne = None
            self.sarj_deneme = 0
            self._gecis('SARJA_GIT', t, neden='kritik pil, kavrama iptal')
            return [
                Eylem(tur='kavrama_iptal', veri=None),
                Eylem(tur='durum', veri='SARJA_GIT'),
                Eylem(tur='nav_git', veri=self.sarj),
            ]

        # Nesne TUTULUYORSA (KOL_KAVRA başarılı, TASI/KOL_BIRAK) nesneyi BIRAKMADAN doğrudan SARJA_GIT
        if eski_durum in ('TASI', 'KOL_BIRAK'):
            self._nesne_tutuluyor = True
            self.sarj_deneme = 0
            self._gecis('SARJA_GIT', t, neden='kritik pil, nesne tutuluyor')
            eylemler = []
            if eski_durum == 'TASI':
                eylemler.append(Eylem(tur='nav_iptal', veri=None))
            eylemler.append(Eylem(tur='durum', veri='SARJA_GIT nesne tutuluyor'))
            eylemler.append(Eylem(tur='nav_git', veri=self.sarj))
            return eylemler

        # EVE_DON durumundaysa
        if eski_durum == 'EVE_DON':
            self.sarj_deneme = 0
            self._gecis('SARJA_GIT', t, neden='kritik pil, eve donus kesildi sarja gidiliyor')
            eylemler = [Eylem(tur='nav_iptal', veri=None)]
            durum_metni = 'SARJA_GIT nesne tutuluyor' if (self._nesne_tutuluyor or self._tasima_hatasi) else 'SARJA_GIT'
            eylemler.append(Eylem(tur='durum', veri=durum_metni))
            eylemler.append(Eylem(tur='nav_git', veri=self.sarj))
            return eylemler

        return []

    def _tetikle_uyari_pil(self, t: float) -> List[Eylem]:
        """Uyarı gerilim eşiği tetiklendiğinde uygulanacak geçişler."""
        if self.durum == 'BOSTA':
            self.sarj_deneme = 0
            self._gecis('SARJA_GIT', t, neden='pil dusuk, sarja gidiliyor')
            return [
                Eylem(tur='durum', veri='SARJA_GIT'),
                Eylem(tur='nav_git', veri=self.sarj),
            ]

        if self.durum in ('KAVRAMAYA_GIT', 'KOL_KAVRA', 'TASI', 'KOL_BIRAK', 'EVE_DON'):
            # Görev sürerken uyari_v altı: mevcut görevi BİTİR, bitince SARJA_GIT
            self._gorev_bitince_sarja_git = True
            return []

        return []

    def _olay_kavrama_durum(self, veri: Any, t: float) -> List[Eylem]:
        """Kavrama düğümünden gelen durum mesajlarını değerlendirir."""
        if self.durum != 'KAVRAMAYA_GIT':
            return []

        s = str(veri).strip()
        if s.startswith('HAZIR'):
            self._gecis('KOL_KAVRA', t, neden='kavrama hazir')
            return [
                Eylem(tur='durum', veri='KOL_KAVRA'),
                Eylem(tur='kol_kavra', veri=None),
            ]

        if s.startswith('HATA'):
            if self.kavrama_deneme < self.ayar.max_kavrama_yeniden_dene:
                self.kavrama_deneme += 1
                self._gecis('KAVRAMAYA_GIT', t, neden=f'kavrama hata yeniden deneme ({self.kavrama_deneme})')
                nesne = self.nesneler[self.aktif_nesne] if self.aktif_nesne else None
                return [
                    Eylem(tur='durum', veri=f'KAVRAMAYA_GIT {self.aktif_nesne}'),
                    Eylem(tur='kavrama_hedef', veri=nesne),
                ]
            else:
                self.aktif_nesne = None
                if self._gorev_bitince_sarja_git or self._pil_uyari:
                    self.sarj_deneme = 0
                    self._gecis('SARJA_GIT', t, neden='kavrama basarisiz, sarja gidiliyor')
                    return [
                        Eylem(tur='durum', veri='BASARISIZ kavrama'),
                        Eylem(tur='durum', veri='SARJA_GIT'),
                        Eylem(tur='nav_git', veri=self.sarj),
                    ]
                else:
                    self._gecis('BOSTA', t, neden='kavrama basarisiz')
                    return [
                        Eylem(tur='durum', veri='BASARISIZ kavrama'),
                    ]

        # Ara durumlar durum geçişine yol açmaz
        return []

    def _olay_kol_sonuc(self, veri: Any, t: float) -> List[Eylem]:
        """Kol kavrama veya bırakma servis sonucunu değerlendirir."""
        basarili = self._is_success(veri)

        if self.durum == 'KOL_KAVRA':
            if basarili:
                self._nesne_tutuluyor = True
                self._gecis('TASI', t, neden='kol kavrama basarili')
                return [
                    Eylem(tur='durum', veri='TASI'),
                    Eylem(tur='nav_git', veri=self.birakma),
                ]
            else:
                if self.kavrama_deneme < self.ayar.max_kavrama_yeniden_dene:
                    self.kavrama_deneme += 1
                    self._gecis(
                        'KAVRAMAYA_GIT',
                        t,
                        neden=f'kol kavrama basarisiz, kavrama yeniden deneme ({self.kavrama_deneme})',
                    )
                    nesne = self.nesneler[self.aktif_nesne] if self.aktif_nesne else None
                    return [
                        Eylem(tur='durum', veri=f'KAVRAMAYA_GIT {self.aktif_nesne}'),
                        Eylem(tur='kavrama_hedef', veri=nesne),
                    ]
                else:
                    self.aktif_nesne = None
                    if self._gorev_bitince_sarja_git or self._pil_uyari:
                        self.sarj_deneme = 0
                        self._gecis('SARJA_GIT', t, neden='kol kavrama basarisiz, sarja gidiliyor')
                        return [
                            Eylem(tur='durum', veri='BASARISIZ kavrama'),
                            Eylem(tur='durum', veri='SARJA_GIT'),
                            Eylem(tur='nav_git', veri=self.sarj),
                        ]
                    else:
                        self._gecis('BOSTA', t, neden='kol kavrama basarisiz, denemeler tukendi')
                        return [
                            Eylem(tur='durum', veri='BASARISIZ kavrama'),
                        ]

        if self.durum == 'KOL_BIRAK':
            self._nesne_tutuluyor = False
            self._gecis('EVE_DON', t, neden='kol birakma tamamlandi')
            return [
                Eylem(tur='durum', veri='EVE_DON'),
                Eylem(tur='nav_git', veri=self.ev),
            ]

        return []

    def _olay_nav_sonuc(self, veri: Any, t: float) -> List[Eylem]:
        """Nav2 eylem sonucunu değerlendirir."""
        basarili = self._is_success(veri)

        if self.durum == 'TASI':
            if basarili:
                self._gecis('KOL_BIRAK', t, neden='tasima basarili')
                return [
                    Eylem(tur='durum', veri='KOL_BIRAK'),
                    Eylem(tur='kol_birak', veri=None),
                ]
            else:
                if self.tasima_deneme < self.ayar.max_tasima_yeniden_dene:
                    self.tasima_deneme += 1
                    self._gecis('TASI', t, neden=f'tasima basarisiz yeniden deneme ({self.tasima_deneme})')
                    return [
                        Eylem(tur='durum', veri='TASI'),
                        Eylem(tur='nav_git', veri=self.birakma),
                    ]
                else:
                    self._tasima_hatasi = True
                    self._gecis('EVE_DON', t, neden='tasima basarisiz, eve donuluyor')
                    return [
                        Eylem(tur='durum', veri='BASARISIZ tasima'),
                        Eylem(tur='durum', veri='EVE_DON'),
                        Eylem(tur='nav_git', veri=self.ev),
                    ]

        if self.durum == 'EVE_DON':
            self.aktif_nesne = None
            if self._gorev_bitince_sarja_git or self._pil_uyari or self._pil_kritik:
                self.sarj_deneme = 0
                self._gecis('SARJA_GIT', t, neden='gorev tamamlandi, sarja gidiliyor')
                eylemler: List[Eylem] = []
                if not self._tasima_hatasi:
                    eylemler.append(Eylem(tur='durum', veri='TAMAM'))
                durum_metni = 'SARJA_GIT nesne tutuluyor' if (self._nesne_tutuluyor or self._tasima_hatasi) else 'SARJA_GIT'
                eylemler.append(Eylem(tur='durum', veri=durum_metni))
                eylemler.append(Eylem(tur='nav_git', veri=self.sarj))
                return eylemler
            else:
                self._gecis('BOSTA', t, neden='eve donus tamamlandi')
                if self._tasima_hatasi:
                    return [
                        Eylem(tur='durum', veri='BOSTA'),
                    ]
                else:
                    return [
                        Eylem(tur='durum', veri='TAMAM'),
                    ]

        if self.durum == 'SARJA_GIT':
            if basarili:
                self._gecis('SARJDA', t, neden='sarj noktasina ulasildi')
                return [
                    Eylem(tur='durum', veri='SARJDA'),
                ]
            else:
                if self.sarj_deneme < self.ayar.max_sarj_yeniden_dene:
                    self.sarj_deneme += 1
                    self._gecis(
                        'SARJA_GIT',
                        t,
                        neden=f'sarja git basarisiz yeniden deneme ({self.sarj_deneme})',
                    )
                    durum_metni = 'SARJA_GIT nesne tutuluyor' if self._nesne_tutuluyor else 'SARJA_GIT'
                    return [
                        Eylem(tur='durum', veri=durum_metni),
                        Eylem(tur='nav_git', veri=self.sarj),
                    ]
                else:
                    self._sarj_basarisiz = True
                    self._gecis('BOSTA', t, neden='sarja git basarisiz, denemeler tukendi')
                    return [
                        Eylem(tur='durum', veri='BASARISIZ sarj'),
                    ]

        return []

    def _olay_zaman(self, t: float) -> List[Eylem]:
        """Aşama zaman aşımlarını denetler."""
        # Zaman akışında pil tamponu takibi
        if self.son_gerilim is not None and (self._uyari_baslangic is not None or self._kritik_baslangic is not None):
            pil_eylemleri = self._olay_pil(self.son_gerilim, t)
            if pil_eylemleri:
                return pil_eylemleri

        gecen = max(0.0, float(t) - self.asama_baslangic)

        if self.durum == 'KAVRAMAYA_GIT':
            if gecen >= self.ayar.kavrama_zaman_asimi:
                if self.kavrama_deneme < self.ayar.max_kavrama_yeniden_dene:
                    self.kavrama_deneme += 1
                    self._gecis('KAVRAMAYA_GIT', t, neden=f'kavrama zaman asimi yeniden deneme ({self.kavrama_deneme})')
                    nesne = self.nesneler[self.aktif_nesne] if self.aktif_nesne else None
                    return [
                        Eylem(tur='kavrama_iptal', veri=None),
                        Eylem(tur='durum', veri=f'KAVRAMAYA_GIT {self.aktif_nesne}'),
                        Eylem(tur='kavrama_hedef', veri=nesne),
                    ]
                else:
                    self.aktif_nesne = None
                    if self._gorev_bitince_sarja_git or self._pil_uyari:
                        self.sarj_deneme = 0
                        self._gecis('SARJA_GIT', t, neden='kavrama zaman asimi, sarja gidiliyor')
                        return [
                            Eylem(tur='kavrama_iptal', veri=None),
                            Eylem(tur='durum', veri='BASARISIZ kavrama'),
                            Eylem(tur='durum', veri='SARJA_GIT'),
                            Eylem(tur='nav_git', veri=self.sarj),
                        ]
                    else:
                        self._gecis('BOSTA', t, neden='kavrama zaman asimi')
                        return [
                            Eylem(tur='kavrama_iptal', veri=None),
                            Eylem(tur='durum', veri='BASARISIZ kavrama'),
                        ]

        elif self.durum == 'TASI':
            if gecen >= self.ayar.tasima_zaman_asimi:
                if self.tasima_deneme < self.ayar.max_tasima_yeniden_dene:
                    self.tasima_deneme += 1
                    self._gecis('TASI', t, neden=f'tasima zaman asimi yeniden deneme ({self.tasima_deneme})')
                    return [
                        Eylem(tur='nav_iptal', veri=None),
                        Eylem(tur='durum', veri='TASI'),
                        Eylem(tur='nav_git', veri=self.birakma),
                    ]
                else:
                    self._tasima_hatasi = True
                    self._gecis('EVE_DON', t, neden='tasima zaman asimi, eve donuluyor')
                    return [
                        Eylem(tur='nav_iptal', veri=None),
                        Eylem(tur='durum', veri='BASARISIZ tasima'),
                        Eylem(tur='durum', veri='EVE_DON'),
                        Eylem(tur='nav_git', veri=self.ev),
                    ]

        elif self.durum == 'KOL_KAVRA':
            if gecen >= self.ayar.kol_zaman_asimi:
                if self.kavrama_deneme < self.ayar.max_kavrama_yeniden_dene:
                    self.kavrama_deneme += 1
                    self._gecis(
                        'KAVRAMAYA_GIT',
                        t,
                        neden=f'kol kavrama zaman asimi, kavrama yeniden deneme ({self.kavrama_deneme})',
                    )
                    nesne = self.nesneler[self.aktif_nesne] if self.aktif_nesne else None
                    return [
                        Eylem(tur='durum', veri=f'KAVRAMAYA_GIT {self.aktif_nesne}'),
                        Eylem(tur='kavrama_hedef', veri=nesne),
                    ]
                else:
                    self.aktif_nesne = None
                    if self._gorev_bitince_sarja_git or self._pil_uyari:
                        self.sarj_deneme = 0
                        self._gecis('SARJA_GIT', t, neden='kol kavrama zaman asimi, sarja gidiliyor')
                        return [
                            Eylem(tur='durum', veri='BASARISIZ kavrama'),
                            Eylem(tur='durum', veri='SARJA_GIT'),
                            Eylem(tur='nav_git', veri=self.sarj),
                        ]
                    else:
                        self._gecis('BOSTA', t, neden='kol kavrama zaman asimi')
                        return [
                            Eylem(tur='durum', veri='BASARISIZ kavrama'),
                        ]

        elif self.durum == 'KOL_BIRAK':
            if gecen >= self.ayar.kol_zaman_asimi:
                self._nesne_tutuluyor = False
                self._gecis('EVE_DON', t, neden='kol birakma zaman asimi, eve donuluyor')
                return [
                    Eylem(tur='durum', veri='EVE_DON'),
                    Eylem(tur='nav_git', veri=self.ev),
                ]

        elif self.durum == 'EVE_DON':
            if gecen >= self.ayar.eve_don_zaman_asimi:
                self.aktif_nesne = None
                if self._gorev_bitince_sarja_git or self._pil_uyari or self._pil_kritik:
                    self.sarj_deneme = 0
                    self._gecis('SARJA_GIT', t, neden='eve donus zaman asimi, sarja gidiliyor')
                    durum_metni = 'SARJA_GIT nesne tutuluyor' if (self._nesne_tutuluyor or self._tasima_hatasi) else 'SARJA_GIT'
                    return [
                        Eylem(tur='nav_iptal', veri=None),
                        Eylem(tur='durum', veri=durum_metni),
                        Eylem(tur='nav_git', veri=self.sarj),
                    ]
                else:
                    self._gecis('BOSTA', t, neden='eve donus zaman asimi')
                    return [
                        Eylem(tur='nav_iptal', veri=None),
                        Eylem(tur='durum', veri='BOSTA'),
                    ]

        elif self.durum == 'SARJA_GIT':
            if gecen >= self.ayar.sarj_zaman_asimi:
                if self.sarj_deneme < self.ayar.max_sarj_yeniden_dene:
                    self.sarj_deneme += 1
                    self._gecis(
                        'SARJA_GIT',
                        t,
                        neden=f'sarja git zaman asimi yeniden deneme ({self.sarj_deneme})',
                    )
                    durum_metni = 'SARJA_GIT nesne tutuluyor' if self._nesne_tutuluyor else 'SARJA_GIT'
                    return [
                        Eylem(tur='nav_iptal', veri=None),
                        Eylem(tur='durum', veri=durum_metni),
                        Eylem(tur='nav_git', veri=self.sarj),
                    ]
                else:
                    self._sarj_basarisiz = True
                    self._gecis('BOSTA', t, neden='sarja git zaman asimi')
                    return [
                        Eylem(tur='nav_iptal', veri=None),
                        Eylem(tur='durum', veri='BASARISIZ sarj'),
                    ]

        return []


def load_nesneler(path: str) -> Dict[str, Any]:
    """YAML dosyasından ev, bırakma, şarj ve nesne koordinatlarını yükler ve doğrular."""
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError(f"Geçersiz YAML içeriği '{path}': sözlük bekleniyordu.")

    if 'nesneler' not in data or not isinstance(data['nesneler'], dict):
        raise ValueError(f"'{path}' yapılandırmasında 'nesneler' sözlüğü bulunmalıdır.")

    nesneler: Dict[str, NesneKonumu] = {}
    for ad, n_data in data['nesneler'].items():
        if not isinstance(n_data, dict):
            raise ValueError(f"'{ad}' nesnesi için koordinat sözlüğü bekleniyordu.")
        if 'x' not in n_data or 'y' not in n_data:
            raise ValueError(f"'{ad}' nesnesinde zorunlu 'x' veya 'y' eksik.")
        try:
            nx = float(n_data['x'])
            ny = float(n_data['y'])
            nz = float(n_data.get('z', 0.0))
        except (TypeError, ValueError) as err:
            raise ValueError(f"'{ad}' koordinatları sayısal olmalıdır.") from err
        nesneler[str(ad)] = NesneKonumu(nx, ny, nz)

    if 'ev' not in data or not isinstance(data['ev'], dict):
        raise ValueError(f"'{path}' yapılandırmasında 'ev' pozu bulunmalıdır.")

    ev_data = data['ev']
    if 'x' not in ev_data or 'y' not in ev_data:
        raise ValueError(f"'{path}' ev pozunda 'x' veya 'y' eksik.")
    try:
        ex = float(ev_data['x'])
        ey = float(ev_data['y'])
        eyaw = float(ev_data.get('yaw_derece', ev_data.get('yaw', 0.0)))
    except (TypeError, ValueError) as err:
        raise ValueError("Ev koordinatları sayısal olmalıdır.") from err
    ev = Poz2D(ex, ey, eyaw)

    birakma: Poz2D = ev
    if 'birakma' in data and data['birakma'] is not None:
        b_data = data['birakma']
        if not isinstance(b_data, dict):
            raise ValueError(f"'{path}' birakma pozu bir sözlük olmalıdır.")
        if 'x' in b_data and 'y' in b_data:
            try:
                bx = float(b_data['x'])
                by = float(b_data['y'])
                byaw = float(b_data.get('yaw_derece', b_data.get('yaw', 0.0)))
                birakma = Poz2D(bx, by, byaw)
            except (TypeError, ValueError) as err:
                raise ValueError("Bırakma koordinatları sayısal olmalıdır.") from err

    sarj: Poz2D = ev
    if 'sarj' in data and data['sarj'] is not None:
        s_data = data['sarj']
        if not isinstance(s_data, dict):
            raise ValueError(f"'{path}' sarj pozu bir sözlük olmalıdır.")
        if 'x' in s_data and 'y' in s_data:
            try:
                sx = float(s_data['x'])
                sy = float(s_data['y'])
                syaw = float(s_data.get('yaw_derece', s_data.get('yaw', 0.0)))
                sarj = Poz2D(sx, sy, syaw)
            except (TypeError, ValueError) as err:
                raise ValueError("Sarj koordinatları sayısal olmalıdır.") from err

    pil_ayar = PilAyari()
    if 'pil' in data and data['pil'] is not None:
        p_data = data['pil']
        if not isinstance(p_data, dict):
            raise ValueError(f"'{path}' pil ayarları bir sözlük olmalıdır.")
        try:
            pil_ayar = PilAyari(
                uyari_v=float(p_data.get('uyari_v', 11.0)),
                kritik_v=float(p_data.get('kritik_v', 10.5)),
                tampon_s=float(p_data.get('tampon_s', 10.0)),
            )
        except (TypeError, ValueError) as err:
            raise ValueError("Pil ayarları sayısal olmalıdır.") from err

    ayar_dict = data.get('ayar', {})
    ayar = Ayar(**ayar_dict) if isinstance(ayar_dict, dict) else Ayar()

    return {
        'nesneler': nesneler,
        'ev': ev,
        'birakma': birakma,
        'sarj': sarj,
        'pil_ayar': pil_ayar,
        'pil': pil_ayar,
        'ayar': ayar,
    }


# mission.py üslubuyla Türkçe takma ad
nesneler_yukle = load_nesneler
