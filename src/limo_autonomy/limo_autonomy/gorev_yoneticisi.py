"""LIMO + myCobot görev yöneticisi: 'nesneyi al, getir' karar katmanı durum makinesi.

ROS'tan bağımsız, olay güdümlü saf Python modülü.
Saat (t) dışarıdan verilir.

Durum Akışı:
  BOSTA -> KAVRAMAYA_GIT (kavrama hedefi gönder) -> [HAZIR] KOL_KAVRA -> [başarılı]
  TASI (birakma pozuna NavigateToPose) -> [başarılı] KOL_BIRAK -> [başarılı]
  EVE_DON -> BOSTA (sonuç 'TAMAM').

Hata ve Yeniden Deneme Politikası:
  - Kavrama HATA: 1 kez yeniden dene (yeniden hedef gönder), sonra 'BASARISIZ kavrama'.
  - Kol kavra başarısız: kavramayı 1 kez yeniden dene, sonra 'BASARISIZ kavrama'.
  - Taşıma başarısız: 2 kez yeniden dene (toplam 3 deneme), sonra 'BASARISIZ tasima'
    ve olduğun yerde KOL_BIRAK yapmadan eve dönmeyi dene.
  - İptal her durumda: aktif eylemi iptal et ve BOSTA durumuna geç.
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
    max_kavrama_yeniden_dene: int = 1
    max_tasima_yeniden_dene: int = 2


@dataclass(frozen=True)
class Eylem:
    """Durum makinesinin dış dünyaya ürettiği eylem komutu."""

    tur: str  # 'kavrama_hedef'|'kavrama_iptal'|'kol_kavra'|'kol_birak'|'nav_git'|'nav_iptal'|'durum'
    veri: Any = None


class GorevYoneticisi:
    """Nesne alma ve getirme görevlerini yöneten durum makinesi."""

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
        ayar: Optional[Any] = None,
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

        if ayar is None:
            self.ayar = Ayar()
        elif isinstance(ayar, dict):
            self.ayar = Ayar(**ayar)
        elif isinstance(ayar, Ayar):
            self.ayar = ayar
        else:
            raise ValueError(f"Geçersiz ayar tipi: {type(ayar)}")

        self.durum: str = 'BOSTA'
        self.gecmis: List[Tuple[float, str, str, str]] = []
        self.aktif_nesne: Optional[str] = None
        self.asama_baslangic: float = 0.0
        self.kavrama_deneme: int = 0
        self.tasima_deneme: int = 0
        self._tasima_hatasi: bool = False

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

    def istek(self, ad: str, t: float = 0.0) -> List[Eylem]:
        """Kullanıcıdan gelen nesne alma isteğini veya iptal komutunu işler."""
        if ad == 'iptal':
            return self.olay('iptal', None, t)

        if self.durum != 'BOSTA':
            return [Eylem(tur='durum', veri='HATA mesgul')]

        if ad not in self.nesneler:
            return [Eylem(tur='durum', veri='HATA bilinmeyen nesne')]

        self.aktif_nesne = ad
        self.kavrama_deneme = 0
        self.tasima_deneme = 0
        self._tasima_hatasi = False
        self._gecis('KAVRAMAYA_GIT', t, neden=f'istek: {ad}')

        return [
            Eylem(tur='durum', veri=f'KAVRAMAYA_GIT {ad}'),
            Eylem(tur='kavrama_hedef', veri=self.nesneler[ad]),
        ]

    def olay(self, tur: str, veri: Any = None, t: float = 0.0) -> List[Eylem]:
        """Sistemden gelen olayları işler (kavrama_durum, kol_sonuc, nav_sonuc, iptal, zaman)."""
        if tur == 'iptal':
            return self._iptal_et(t)

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
        elif self.durum in ('TASI', 'EVE_DON'):
            eylemler.append(Eylem(tur='nav_iptal', veri=None))

        self._gecis('BOSTA', t, neden='iptal')
        self.aktif_nesne = None
        eylemler.append(Eylem(tur='durum', veri='IPTAL'))
        return eylemler

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
                self._gecis('BOSTA', t, neden='kavrama basarisiz')
                self.aktif_nesne = None
                return [
                    Eylem(tur='durum', veri='BASARISIZ kavrama'),
                ]

        # Ara durumlar (PLAN, NAV, HIZALA, YAKLAS vb.) durum geçişine yol açmaz
        return []

    def _olay_kol_sonuc(self, veri: Any, t: float) -> List[Eylem]:
        """Kol kavrama veya bırakma servis sonucunu değerlendirir."""
        basarili = self._is_success(veri)

        if self.durum == 'KOL_KAVRA':
            if basarili:
                self._gecis('TASI', t, neden='kol kavrama basarili')
                return [
                    Eylem(tur='durum', veri='TASI'),
                    Eylem(tur='nav_git', veri=self.birakma),
                ]
            else:
                # kol kavra başarısız -> geri çekil yok, kavramayı 1 kez yeniden dene
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
                    self._gecis('BOSTA', t, neden='kol kavrama basarisiz, denemeler tukendi')
                    self.aktif_nesne = None
                    return [
                        Eylem(tur='durum', veri='BASARISIZ kavrama'),
                    ]

        if self.durum == 'KOL_BIRAK':
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
                # TASI başarısız -> 2 kez yeniden dene, sonra 'BASARISIZ tasima' ve KOL_BIRAK YAPMA, eve dönmeyi dene
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
            self._gecis('BOSTA', t, neden='eve donus tamamlandi')
            self.aktif_nesne = None
            if self._tasima_hatasi:
                return [
                    Eylem(tur='durum', veri='BOSTA'),
                ]
            else:
                return [
                    Eylem(tur='durum', veri='TAMAM'),
                ]

        return []

    def _olay_zaman(self, t: float) -> List[Eylem]:
        """Aşama zaman aşımlarını denetler."""
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
                    self._gecis('BOSTA', t, neden='kavrama zaman asimi')
                    self.aktif_nesne = None
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
                    self._gecis('BOSTA', t, neden='kol kavrama zaman asimi')
                    self.aktif_nesne = None
                    return [
                        Eylem(tur='durum', veri='BASARISIZ kavrama'),
                    ]

        elif self.durum == 'KOL_BIRAK':
            if gecen >= self.ayar.kol_zaman_asimi:
                self._gecis('EVE_DON', t, neden='kol birakma zaman asimi, eve donuluyor')
                return [
                    Eylem(tur='durum', veri='EVE_DON'),
                    Eylem(tur='nav_git', veri=self.ev),
                ]

        elif self.durum == 'EVE_DON':
            if gecen >= self.ayar.eve_don_zaman_asimi:
                self._gecis('BOSTA', t, neden='eve donus zaman asimi')
                self.aktif_nesne = None
                return [
                    Eylem(tur='nav_iptal', veri=None),
                    Eylem(tur='durum', veri='BOSTA'),
                ]

        return []


def load_nesneler(path: str) -> Dict[str, Any]:
    """YAML dosyasından ev, bırakma ve nesne koordinatlarını yükler ve doğrular."""
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

    ayar_dict = data.get('ayar', {})
    ayar = Ayar(**ayar_dict) if isinstance(ayar_dict, dict) else Ayar()

    return {
        'nesneler': nesneler,
        'ev': ev,
        'birakma': birakma,
        'ayar': ayar,
    }


# mission.py üslubuyla Türkçe takma ad
nesneler_yukle = load_nesneler
