"""
test_ajan.py - ajan aracının birim ve entegrasyon testleri.
GERÇEK agy ikilisi çağrılmadan, sahte agy (fake_agy) ile test edilir.
"""

from __future__ import annotations

import json
import os
import signal
import stat
import subprocess
import sys
import time
from typing import Any
import pytest

from ajan import (
    MODEL_ALIASES,
    add_permissions,
    check_permissions,
    extract_command,
    extract_dogrulama_ozet,
    extract_file_path,
    extract_rapor,
    get_last_error_from_log,
    is_pid_alive,
    path_covers,
    resolve_model,
)

AJAN_PY = os.path.abspath(os.path.join(os.path.dirname(__file__), "ajan.py"))


# ---------------------------------------------------------------------------
# Sahte agy Fixture
# ---------------------------------------------------------------------------
@pytest.fixture
def fake_agy(tmp_path: Any) -> tuple[str, str]:
    """
    Sahte agy ikilisi betiği oluşturur.
    Argümanları fake_agy.log dosyasına JSON olarak kaydeder.
    FAKE_AGY_MODE değişkenine göre stream-json davranışı sergiler.
    """
    log_file = tmp_path / "fake_agy.log"
    script_file = tmp_path / "fake_agy.py"

    code = f"""#!/usr/bin/env python3
import json
import os
import sys
import time

log_file = {repr(str(log_file))}
mode = os.environ.get("FAKE_AGY_MODE", "success")

# Argümanları kaydet
with open(log_file, "a", encoding="utf-8") as f:
    f.write(json.dumps({{"argv": sys.argv[1:], "cwd": os.getcwd()}}) + "\\n")

# --conversation argümanı kontrolü
conv_id = "fake-conv-uuid-123"
for i, arg in enumerate(sys.argv):
    if arg == "--conversation" and i + 1 < len(sys.argv):
        conv_id = sys.argv[i + 1]

if mode == "sleep":
    init_ev = {{"event": "init", "conversation_id": conv_id, "init": {{"model": "test-model"}}}}
    print(json.dumps(init_ev), flush=True)
    step_ev = {{
        "event": "step_update",
        "step_update": {{
            "conversation_id": conv_id,
            "step_index": 1,
            "state": "ACTIVE",
            "step_type": "tool",
            "tool_name": "read_file",
            "tool_info": {{"parameters": {{"Path": "/tmp/sleep_test.py"}}}},
        }}
    }}
    print(json.dumps(step_ev), flush=True)
    # Bekle (sinyalleri kabul et)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        sys.exit(0)

elif mode == "error":
    init_ev = {{"event": "init", "conversation_id": conv_id, "init": {{"model": "test-model"}}}}
    print(json.dumps(init_ev), flush=True)
    res_ev = {{
        "event": "result",
        "result": {{
            "conversation_id": conv_id,
            "status": "ERROR",
            "response": "Kritik bir hata meydana geldi.",
            "duration_seconds": 0.5,
        }}
    }}
    print(json.dumps(res_ev), flush=True)
    sys.stderr.write("ERROR: Model çalışma hatası oluştu\\n")
    sys.exit(1)

elif mode == "varied_keys":
    init_ev = {{"event": "init", "conversation_id": conv_id, "init": {{"model": "test-model"}}}}
    print(json.dumps(init_ev), flush=True)
    
    # Farklı parametre anahtarlarıyla write_file adımları
    tools = [
        ("write_file", {{"Path": "/workspace/path1.py"}}),
        ("edit_file", {{"TargetFile": "/workspace/target2.py"}}),
        ("replace_file_content", {{"AbsolutePath": "/workspace/abs3.py"}}),
        ("write_to_file", {{"path": "/workspace/path4.py"}}),
        ("custom_write", {{"file_path": "/workspace/filepath5.py"}}),
        ("save_file", {{"dest": "/workspace/custom6.py"}}),
        ("run_command", {{"CommandLine": "pytest -v"}}),
    ]
    for idx, (t_name, params) in enumerate(tools, start=1):
        step_ev = {{
            "event": "step_update",
            "step_update": {{
                "conversation_id": conv_id,
                "step_index": idx,
                "state": "DONE",
                "step_type": "tool",
                "tool_name": t_name,
                "tool_info": {{"parameters": params}},
                "usage": {{"input_tokens": 10 * idx, "output_tokens": 5 * idx, "total_tokens": 15 * idx}}
            }}
        }}
        print(json.dumps(step_ev), flush=True)
        
    res_ev = {{
        "event": "result",
        "result": {{
            "conversation_id": conv_id,
            "status": "SUCCESS",
            "response": "Tüm adımlar tamamlandı.\\n```json\\n{{\\"ozet\\": \\"Parametre anahtarları test edildi.\\", \\"degisen_dosyalar\\": [\\"/workspace/path1.py\\"], \\"calistirilan_testler\\": [\\"pytest -v: PASSED\\"], \\"acik_konular\\": [], \\"basarili\\": true}}\\n```",
            "duration_seconds": 1.2,
            "usage": {{"input_tokens": 100, "output_tokens": 50, "total_tokens": 150}}
        }}
    }}
    print(json.dumps(res_ev), flush=True)

elif mode == "no_json_block":
    init_ev = {{"event": "init", "conversation_id": conv_id, "init": {{"model": "test-model"}}}}
    print(json.dumps(init_ev), flush=True)
    res_ev = {{
        "event": "result",
        "result": {{
            "conversation_id": conv_id,
            "status": "SUCCESS",
            "response": "Bu yanıtta JSON bloğu bulunmamaktadır, serbest metin raporudur.",
            "duration_seconds": 0.8,
            "usage": {{"input_tokens": 50, "output_tokens": 20, "total_tokens": 70}}
        }}
    }}
    print(json.dumps(res_ev), flush=True)

elif mode == "fix_on_second_turn":
    init_ev = {{"event": "init", "conversation_id": conv_id, "init": {{"model": "test-model"}}}}
    print(json.dumps(init_ev), flush=True)
    fix_file = os.environ.get("FAKE_AGY_FIX_FILE", "")
    is_followup = any(arg == "--conversation" for arg in sys.argv)
    if is_followup and fix_file:
        try:
            with open(fix_file, "w", encoding="utf-8") as ff:
                ff.write("duzeltildi\\n")
        except Exception:
            pass
        step_ev = {{
            "event": "step_update",
            "step_update": {{
                "conversation_id": conv_id,
                "step_index": 1,
                "state": "DONE",
                "step_type": "tool",
                "tool_name": "write_file",
                "tool_info": {{"parameters": {{"Path": fix_file}}}},
            }}
        }}
        print(json.dumps(step_ev), flush=True)
    res_ev = {{
        "event": "result",
        "result": {{
            "conversation_id": conv_id,
            "status": "SUCCESS",
            "response": "İşlem yapıldı.\\n```json\\n{{\\"ozet\\": \\"Düzeltildi\\", \\"degisen_dosyalar\\": [], \\"calistirilan_testler\\": [], \\"acik_konular\\": [], \\"basarili\\": true}}\\n```",
            "duration_seconds": 0.5,
        }}
    }}
    print(json.dumps(res_ev), flush=True)

else:
    # default success
    init_ev = {{"event": "init", "conversation_id": conv_id, "init": {{"model": "test-model"}}}}
    print(json.dumps(init_ev), flush=True)
    step1 = {{
        "event": "step_update",
        "step_update": {{
            "conversation_id": conv_id,
            "step_index": 1,
            "state": "DONE",
            "step_type": "tool",
            "tool_name": "write_file",
            "tool_info": {{"parameters": {{"Path": "/tmp/test_cikti.py"}}}},
            "usage": {{"input_tokens": 80, "output_tokens": 20, "total_tokens": 100}}
        }}
    }}
    print(json.dumps(step1), flush=True)
    res_ev = {{
        "event": "result",
        "result": {{
            "conversation_id": conv_id,
            "status": "SUCCESS",
            "response": "Görev tamamlandı.\\n```json\\n{{\\"ozet\\": \\"Sonuç başarılı.\\", \\"degisen_dosyalar\\": [\\"/tmp/test_cikti.py\\"], \\"calistirilan_testler\\": [\\"pytest: PASSED\\"], \\"acik_konular\\": [], \\"basarili\\": true}}\\n```",
            "duration_seconds": 1.0,
            "usage": {{"input_tokens": 120, "output_tokens": 40, "total_tokens": 160}}
        }}
    }}
    print(json.dumps(res_ev), flush=True)
"""
    script_file.write_text(code, encoding="utf-8")
    script_file.chmod(script_file.stat().st_mode | stat.S_IEXEC)
    return str(script_file), str(log_file)


@pytest.fixture
def test_env(tmp_path: Any, fake_agy: tuple[str, str]) -> dict[str, Any]:
    """Testler için yalıtılmış ortam ve dizinleri hazırlar."""
    fake_bin, log_path = fake_agy
    state_dir = tmp_path / "ajan_state"
    settings_dir = tmp_path / "gemini_config"
    settings_path = settings_dir / "settings.json"

    state_dir.mkdir(parents=True, exist_ok=True)
    settings_dir.mkdir(parents=True, exist_ok=True)

    # Başlangıçta boş izinler
    settings_path.write_text(
        json.dumps({"permissions": {"allow": []}}, indent=2), encoding="utf-8"
    )

    env = os.environ.copy()
    env["AJAN_AGY"] = fake_bin
    env["AJAN_DIZIN"] = str(state_dir)
    env["AJAN_AYAR"] = str(settings_path)
    env["FAKE_AGY_LOG"] = log_path

    return {
        "env": env,
        "state_dir": state_dir,
        "settings_path": settings_path,
        "log_path": log_path,
    }


def run_ajan(
    args: list[str], env: dict[str, str], timeout: float = 10.0
) -> subprocess.CompletedProcess[str]:
    """ajan komutunu yalıtılmış ortamda çalıştırır."""
    return subprocess.run(
        [sys.executable, AJAN_PY] + args,
        env=env,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


# ---------------------------------------------------------------------------
# Birim Testler: Fonksiyonel Mantık
# ---------------------------------------------------------------------------
def test_model_takma_adlari() -> None:
    """Model takma adlarının doğru çözümlendiğini doğrular."""
    assert resolve_model("flash") == "gemini-3.8-flash-high"
    assert resolve_model("flash-orta") == "gemini-3.8-flash-medium"
    assert resolve_model("argon") == "gemini-4-argon-high"
    assert resolve_model("argon-orta") == "gemini-4-argon-medium"
    assert resolve_model(None) == "gemini-3.8-flash-high"
    assert resolve_model("claude-sonnet-5-5-high") == "claude-sonnet-5-5-high"


def test_path_covers_mantigi() -> None:
    """path_covers fonksiyonunun alt yolları doğru eşleştirdiğini test eder."""
    assert path_covers("/home/user/project", "/home/user/project")
    assert path_covers("/home/user/project", "/home/user/project/sub/file.py")
    assert not path_covers("/home/user/project/sub", "/home/user/project")
    assert not path_covers("/home/user/other", "/home/user/project")


def test_yazilan_dosya_yolu_cikarimi_parametreleri() -> None:
    """
    Parametre anahtar adları araçtan araca değiştiğinde (Path, TargetFile,
    AbsolutePath, path, file_path ve '/' ile başlayanlar) doğru çıkarıldığını test eder.
    """
    assert (
        extract_file_path({"Path": "/workspace/dosya1.py"})
        == "/workspace/dosya1.py"
    )
    assert (
        extract_file_path({"TargetFile": "/workspace/dosya2.py"})
        == "/workspace/dosya2.py"
    )
    assert (
        extract_file_path({"AbsolutePath": "/workspace/dosya3.py"})
        == "/workspace/dosya3.py"
    )
    assert (
        extract_file_path({"path": "/workspace/dosya4.py"})
        == "/workspace/dosya4.py"
    )
    assert (
        extract_file_path({"file_path": "/workspace/dosya5.py"})
        == "/workspace/dosya5.py"
    )
    # Anahtar listede yoksa, '/' ile başlayan değer kontrol edilir
    assert (
        extract_file_path({"hedef_dosya": "/workspace/ozel.txt", "adet": 42})
        == "/workspace/ozel.txt"
    )
    # Hiçbir yol yoksa None döner
    assert extract_file_path({"text": "merhaba"}) is None


def test_rapor_cikarma_ve_fallback() -> None:
    """JSON bloğu olan ve olmayan yanıtlardan rapor çıkarımını test eder."""
    # 1. Başarılı JSON bloğu
    text1 = """
    İnceleme tamamlandı.
    ```json
    {
        "ozet": "Testler başarıyla geçti",
        "degisen_dosyalar": ["a.py", "b.py"],
        "calistirilan_testler": ["colcon test: passed"],
        "acik_konular": [],
        "basarili": true
    }
    ```
    """
    r1 = extract_rapor(text1, ["a.py"], ["colcon test"])
    assert r1["basarili"] is True
    assert r1["ozet"] == "Testler başarıyla geçti"
    assert "b.py" in r1["degisen_dosyalar"]

    # 2. JSON bloğu içermeyen serbest metin (fallback: basarili: None)
    text2 = "Kod refaktör edildi ancak JSON bloğu eklenmedi."
    r2 = extract_rapor(text2, ["c.py"], ["ls"])
    assert r2["basarili"] is None
    assert "Kod refaktör" in r2["ozet"]
    assert r2["degisen_dosyalar"] == ["c.py"]
    assert r2["calistirilan_testler"] == ["ls"]


# ---------------------------------------------------------------------------
# Entegrasyon Testleri (Sahte agy ile CLI Akışları)
# ---------------------------------------------------------------------------
def test_baslat_bekle_bitti_ve_rapor(test_env: dict[str, Any]) -> None:
    """
    baslat -> bekle -> bitti döngüsü ve rapor.json çıkarımı testi.
    """
    env = test_env["env"]
    state_dir = test_env["state_dir"]

    # baslat ve bekle
    res = run_ajan(
        ["baslat", "gorev1", "-m", "İlk deneme", "--bekle"], env=env
    )
    assert res.returncode == 0, f"Hata çıktısı: {res.stderr}"

    task_dir = state_dir / "gorev1"
    assert task_dir.is_dir()

    # meta.json kontrolü
    meta_path = task_dir / "meta.json"
    assert meta_path.is_file()
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    assert meta["durum"] == "bitti"
    assert meta["conversation_id"] == "fake-conv-uuid-123"
    assert meta["token_toplami"]["total_tokens"] > 0

    # yanit.md kontrolü
    yanit_path = task_dir / "yanit.md"
    assert yanit_path.is_file()
    assert "Görev tamamlandı." in yanit_path.read_text(encoding="utf-8")

    # rapor.json kontrolü
    rapor_path = task_dir / "rapor.json"
    assert rapor_path.is_file()
    rapor = json.loads(rapor_path.read_text(encoding="utf-8"))
    assert rapor["basarili"] is True
    assert rapor["ozet"] == "Sonuç başarılı."

    # rapor komutunun okunabilir çıktısı
    res_rapor = run_ajan(["rapor", "gorev1"], env=env)
    assert res_rapor.returncode == 0
    assert "Başarılı: Evet" in res_rapor.stdout
    assert "Sonuç başarılı." in res_rapor.stdout

    # rapor --ham çıktısı
    res_ham = run_ajan(["rapor", "gorev1", "--ham"], env=env)
    assert res_ham.returncode == 0
    assert "Görev tamamlandı." in res_ham.stdout


def test_ayni_etiket_calisirken_ret(test_env: dict[str, Any]) -> None:
    """
    Bir etiket canlı çalışırken aynı etiketle yeni görev başlatmanın
    çıkış kodu 3 ile reddedildiğini test eder.
    """
    env = test_env["env"]
    env["FAKE_AGY_MODE"] = "sleep"

    # İlk görevi arka planda başlat
    res1 = run_ajan(["baslat", "mesgul_gorev", "-m", "Uyu"], env=env)
    assert res1.returncode == 0

    # Bekçinin agy'yi başlatması için kısa süre tanı
    time.sleep(0.6)

    # Aynı etiketle tekrar başlat
    res2 = run_ajan(
        ["baslat", "mesgul_gorev", "-m", "İkinci deneme"], env=env
    )
    assert res2.returncode == 3
    assert "canlı çalışıyor" in res2.stderr

    # Görevi iptal et
    run_ajan(["iptal", "mesgul_gorev"], env=env)


def test_eksik_izin_cikis_2_ve_onerilen_kural(test_env: dict[str, Any]) -> None:
    """
    settings.json'da kapsayan kural yoksa çıkış 2 verip
    önerilen kuralları listelediğini test eder.
    """
    env = test_env["env"]
    yaz_yol = "/var/custom/workspace"
    komut = "colcon build"

    res = run_ajan(
        [
            "baslat",
            "izin_testi",
            "-m",
            "Test",
            "--yaz",
            yaz_yol,
            "--komut",
            komut,
        ],
        env=env,
    )
    assert res.returncode == 2
    assert f"write_file({yaz_yol})" in res.stderr
    assert f"command({komut})" in res.stderr


def test_izin_ekle_yedekleyip_ekler(test_env: dict[str, Any]) -> None:
    """
    --izin-ekle bayrağının settings.json.bak-<zaman> yedeği alarak
    izinleri eklediğini ve görevi başarıyla başlattığını test eder.
    """
    env = test_env["env"]
    settings_path = test_env["settings_path"]
    yaz_yol = "/tmp/izin_ekle_dizin"
    komut = "pytest -k unit"

    res = run_ajan(
        [
            "baslat",
            "izin_ekle_gorev",
            "-m",
            "İzinli görev",
            "--yaz",
            yaz_yol,
            "--komut",
            komut,
            "--izin-ekle",
            "--bekle",
        ],
        env=env,
    )
    assert res.returncode == 0

    # Yedek dosyasının oluştuğunu doğrula
    bak_files = list(settings_path.parent.glob("settings.json.bak-*"))
    assert len(bak_files) >= 1

    # settings.json dosyasının güncellendiğini doğrula
    data = json.loads(settings_path.read_text(encoding="utf-8"))
    allow_rules = data["permissions"]["allow"]
    assert f"write_file({yaz_yol})" in allow_rules
    assert f"command({komut})" in allow_rules


def test_devam_ikinci_turda_conversation_gecer(
    test_env: dict[str, Any]
) -> None:
    """
    devam komutunun aynı conversation_id ile yeni tur başlattığını
    ve agy CLI'a --conversation parametresini geçtiğini test eder.
    """
    env = test_env["env"]
    log_path = test_env["log_path"]

    # 1. Tur
    res1 = run_ajan(
        ["baslat", "tur_gorev", "-m", "1. Tur İsteği", "--bekle"], env=env
    )
    assert res1.returncode == 0

    # 2. Tur
    res2 = run_ajan(
        ["devam", "tur_gorev", "-m", "2. Tur Takip Mesajı", "--bekle"], env=env
    )
    assert res2.returncode == 0

    # agy'ye iletilen argümanları kontrol et
    with open(log_path, "r", encoding="utf-8") as f:
        calls = [json.loads(line) for line in f if line.strip()]

    assert len(calls) == 2
    second_call_argv = calls[1]["argv"]
    assert "--conversation" in second_call_argv
    conv_idx = second_call_argv.index("--conversation")
    assert second_call_argv[conv_idx + 1] == "fake-conv-uuid-123"

    # meta.json tur geçmişi kontrolü
    meta_path = test_env["state_dir"] / "tur_gorev" / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    assert meta["tur"] == 2
    assert len(meta["turler"]) == 1
    assert meta["turler"][0]["tur"] == 1


def test_hata_sonucu_hata_durumu(test_env: dict[str, Any]) -> None:
    """
    Model ERROR statüsü döndürdüğünde durumun 'hata' olduğunu test eder.
    """
    env = test_env["env"]
    env["FAKE_AGY_MODE"] = "error"

    res = run_ajan(
        ["baslat", "hata_gorev", "-m", "Hata verecek görev", "--bekle"],
        env=env,
    )
    # bekle bitti dışındaki durumlarda 1 ile çıkar
    assert res.returncode == 1

    meta_path = test_env["state_dir"] / "hata_gorev" / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    assert meta["durum"] == "hata"
    assert meta["hata"] is not None


def test_iptal_surec_grubunu_oldurur(test_env: dict[str, Any]) -> None:
    """
    iptal komutunun çalışan süreç grubunu öldürdüğünü ve durumu
    'iptal' yaptığını test eder.
    """
    env = test_env["env"]
    env["FAKE_AGY_MODE"] = "sleep"

    run_ajan(["baslat", "iptal_gorev", "-m", "Uzun iş"], env=env)
    time.sleep(0.6)

    meta_path = test_env["state_dir"] / "iptal_gorev" / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    pid = meta.get("pid")
    assert pid is not None
    assert is_pid_alive(pid)

    res_iptal = run_ajan(["iptal", "iptal_gorev"], env=env)
    assert res_iptal.returncode == 0

    time.sleep(0.3)
    assert not is_pid_alive(pid)

    meta_after = json.loads(meta_path.read_text(encoding="utf-8"))
    assert meta_after["durum"] == "iptal"


def test_pid_olu_kayip_durumu(test_env: dict[str, Any]) -> None:
    """
    Durumu 'calisiyor' görünen ancak PID'si ölü olan görevlerin
    'kayip' olarak raporlandığını test eder.
    """
    env = test_env["env"]
    state_dir = test_env["state_dir"]

    # Sahte bir görev dizini oluştur
    dead_task_dir = state_dir / "olu_gorev"
    dead_task_dir.mkdir(parents=True, exist_ok=True)
    meta = {
        "etiket": "olu_gorev",
        "durum": "calisiyor",
        "pid": 99999999,  # Kesinlikle var olmayan PID
        "tur": 1,
        "sure_s": 5.0,
        "yazilan_dosyalar": [],
        "token_toplami": {"total_tokens": 10},
    }
    (dead_task_dir / "meta.json").write_text(
        json.dumps(meta), encoding="utf-8"
    )

    # durum tablosu kontrolü
    res = run_ajan(["durum", "olu_gorev", "--json"], env=env)
    assert res.returncode == 0
    data = json.loads(res.stdout)
    assert data["durum"] == "kayip"

    # bekle kontrolü: kayip görev için 1 döner
    res_bekle = run_ajan(["bekle", "olu_gorev"], env=env)
    assert res_bekle.returncode == 1
    assert "kayip" in res_bekle.stdout


def test_model_takma_adlari_cli_uzerinden(test_env: dict[str, Any]) -> None:
    """
    CLI üzerinden model takma adlarının (flash, argon vb.)
    tam id olarak agy'ye aktarıldığını test eder.
    """
    env = test_env["env"]
    log_path = test_env["log_path"]

    run_ajan(
        [
            "baslat",
            "model_gorev",
            "-m",
            "Model testi",
            "--model",
            "argon",
            "--bekle",
        ],
        env=env,
    )

    with open(log_path, "r", encoding="utf-8") as f:
        calls = [json.loads(line) for line in f if line.strip()]

    assert len(calls) >= 1
    argv = calls[0]["argv"]
    assert "--model" in argv
    m_idx = argv.index("--model")
    assert argv[m_idx + 1] == "gemini-4-argon-high"


def test_yazilan_dosya_yolu_cikarimi_akista(test_env: dict[str, Any]) -> None:
    """
    Canlı akışta farklı parametre anahtarlarıyla gelen yazma araçlarının
    meta['yazilan_dosyalar'] listesine eksiksiz toplandığını test eder.
    """
    env = test_env["env"]
    env["FAKE_AGY_MODE"] = "varied_keys"

    res = run_ajan(
        ["baslat", "farkli_anahtar_gorev", "-m", "Anahtar testi", "--bekle"],
        env=env,
    )
    assert res.returncode == 0

    meta_path = test_env["state_dir"] / "farkli_anahtar_gorev" / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    yazilanlar = meta["yazilan_dosyalar"]

    assert "/workspace/path1.py" in yazilanlar
    assert "/workspace/target2.py" in yazilanlar
    assert "/workspace/abs3.py" in yazilanlar
    assert "/workspace/path4.py" in yazilanlar
    assert "/workspace/filepath5.py" in yazilanlar
    assert "/workspace/custom6.py" in yazilanlar

    komutlar = meta["calistirilan_komutlar"]
    assert "pytest -v" in komutlar


def test_izin_kontrol_tek_basina(test_env: dict[str, Any]) -> None:
    """
    'ajan izin kontrol' alt komutunun bağımsız çalışmasını test eder.
    """
    env = test_env["env"]

    # 1. İzin yokken çıkış 2
    res1 = run_ajan(
        ["izin", "kontrol", "--yaz", "/tmp/dizin_a", "--komut", "pytest"],
        env=env,
    )
    assert res1.returncode == 2

    # 2. İzin eklendikten sonra çıkış 0
    res2 = run_ajan(
        [
            "izin",
            "kontrol",
            "--yaz",
            "/tmp/dizin_a",
            "--komut",
            "pytest",
            "--izin-ekle",
        ],
        env=env,
    )
    assert res2.returncode == 0

    # 3. Artık mevcut olduğu için tekrar çıkış 0
    res3 = run_ajan(
        ["izin", "kontrol", "--yaz", "/tmp/dizin_a", "--komut", "pytest"],
        env=env,
    )
    assert res3.returncode == 0
    assert "Tüm izinler mevcut." in res3.stdout


# ---------------------------------------------------------------------------
# Doğrulama (dogrula) Testleri
# ---------------------------------------------------------------------------
def test_extract_dogrulama_ozet() -> None:
    """extract_dogrulama_ozet fonksiyonunun hata satırlarını doğru yakaladığını test eder."""
    text1 = """
    running test...
    FAILED test_a.py::test_1 - AssertionError
    passed test_b.py
    ERROR: module not found
    """
    ozet1 = extract_dogrulama_ozet(text1)
    assert "FAILED test_a.py" in ozet1
    assert "ERROR: module not found" in ozet1

    # Hata anahtar kelimesi içermeyen çıktı
    text2 = "Compiling project\nBuild failed with exit code 1"
    ozet2 = extract_dogrulama_ozet(text2)
    assert "Build failed with exit code 1" in ozet2


def test_dogrula_ilk_denemede_gecen(test_env: dict[str, Any]) -> None:
    """İlk denemede başarılı olan doğrulama testini doğrular."""
    env = test_env["env"]
    res_baslat = run_ajan(
        ["baslat", "dogrula_gecen", "-m", "İlk iş", "--bekle"], env=env
    )
    assert res_baslat.returncode == 0

    res_dogrula = run_ajan(
        [
            "dogrula",
            "dogrula_gecen",
            "--test",
            f"{sys.executable} -c 'exit(0)'",
        ],
        env=env,
    )
    assert res_dogrula.returncode == 0
    assert "DOGRULANDI" in res_dogrula.stdout

    meta_path = test_env["state_dir"] / "dogrula_gecen" / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    assert len(meta.get("dogrulama", [])) == 1
    assert meta["dogrulama"][0]["tur"] == 1
    assert meta["dogrulama"][0]["cikis"] == 0

    res_durum = run_ajan(["durum", "dogrula_gecen", "--json"], env=env)
    assert res_durum.returncode == 0
    assert json.loads(res_durum.stdout)["dogrulama"] == "✓"


def test_dogrula_bir_turda_duzelen(
    test_env: dict[str, Any], tmp_path: Any
) -> None:
    """İlk turda başarısız olup takip turunda düzelen görevi doğrular."""
    env = test_env["env"]
    env["FAKE_AGY_MODE"] = "fix_on_second_turn"
    fix_file = tmp_path / "duzeltildi.txt"
    env["FAKE_AGY_FIX_FILE"] = str(fix_file)

    res_baslat = run_ajan(
        ["baslat", "dogrula_duzelen", "-m", "Düzeltme görevi", "--bekle"],
        env=env,
    )
    assert res_baslat.returncode == 0
    assert not fix_file.exists()

    test_cmd = (
        f"{sys.executable} -c \""
        f"import os, sys; "
        f"sys.exit(0 if os.path.exists({repr(str(fix_file))}) else 1)\""
    )
    res_dogrula = run_ajan(
        ["dogrula", "dogrula_duzelen", "--test", test_cmd, "--tur", "3"],
        env=env,
    )
    assert res_dogrula.returncode == 0
    assert "DOGRULANDI" in res_dogrula.stdout
    assert fix_file.is_file()

    meta_path = test_env["state_dir"] / "dogrula_duzelen" / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    assert len(meta.get("dogrulama", [])) == 2
    assert meta["dogrulama"][0]["tur"] == 1
    assert meta["dogrulama"][0]["cikis"] == 1
    assert meta["dogrulama"][1]["tur"] == 2
    assert meta["dogrulama"][1]["cikis"] == 0
    assert meta["tur"] == 2

    res_durum = run_ajan(["durum", "dogrula_duzelen", "--json"], env=env)
    assert res_durum.returncode == 0
    assert json.loads(res_durum.stdout)["dogrulama"] == "✓"


def test_dogrula_tur_sinirinda_basarisiz(test_env: dict[str, Any]) -> None:
    """Belirtilen maksimum tur sayısına ulaşılınca başarısız sonlandığını doğrular."""
    env = test_env["env"]
    res_baslat = run_ajan(
        ["baslat", "dogrula_sinir", "-m", "Sınır testi", "--bekle"], env=env
    )
    assert res_baslat.returncode == 0

    test_cmd = (
        f"{sys.executable} -c \""
        f"import sys; "
        f"sys.stderr.write('FAILED test_ornek - AssertionError\\n'); "
        f"sys.exit(1)\""
    )
    res_dogrula = run_ajan(
        ["dogrula", "dogrula_sinir", "--test", test_cmd, "--tur", "2"],
        env=env,
    )
    assert res_dogrula.returncode == 1
    assert "DOGRULANAMADI (2 tur)" in res_dogrula.stdout

    meta_path = test_env["state_dir"] / "dogrula_sinir" / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    assert len(meta.get("dogrulama", [])) == 2
    assert meta["dogrulama"][0]["cikis"] == 1
    assert meta["dogrulama"][1]["cikis"] == 1
    assert "FAILED" in meta["dogrulama"][0]["ozet"]

    res_durum = run_ajan(["durum", "dogrula_sinir", "--json"], env=env)
    assert res_durum.returncode == 0
    assert json.loads(res_durum.stdout)["dogrulama"] == "✗"


def test_dogrula_calisan_gorevde_ret(test_env: dict[str, Any]) -> None:
    """Canlı çalışan bir görev üzerinde doğrulamanın reddedildiğini test eder."""
    env = test_env["env"]
    env["FAKE_AGY_MODE"] = "sleep"

    res_baslat = run_ajan(
        ["baslat", "dogrula_uyuyan", "-m", "Uzun iş"], env=env
    )
    assert res_baslat.returncode == 0
    time.sleep(0.6)

    res_dogrula = run_ajan(
        [
            "dogrula",
            "dogrula_uyuyan",
            "--test",
            f"{sys.executable} -c 'exit(0)'",
        ],
        env=env,
    )
    assert res_dogrula.returncode != 0
    assert (
        "çalışıyor" in res_dogrula.stderr.lower()
        or "bitmemiş" in res_dogrula.stderr.lower()
    )

    run_ajan(["iptal", "dogrula_uyuyan"], env=env)
