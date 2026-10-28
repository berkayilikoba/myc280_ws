#!/usr/bin/env python3
"""
ajan - Gemini (agy CLI) üzerinde alt ajan orkestrasyon aracı.
Python 3.10+, yalnızca standart kütüphane.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import Any

# Model Takma Adları
MODEL_ALIASES: dict[str, str] = {
    "flash": "gemini-3.8-flash-high",
    "flash-orta": "gemini-3.8-flash-medium",
    "argon": "gemini-4-argon-high",
    "argon-orta": "gemini-4-argon-medium",
}

DEFAULT_MODEL = "gemini-3.8-flash-high"


def get_lang() -> str:
    """Etkin dili döner ('en' veya 'tr')."""
    return "en" if os.environ.get("AJAN_DIL", "").strip().lower() == "en" else "tr"


MESSAGES: dict[str, dict[str, Any]] = {
    "tr": {
        "err_task_file_not_found": "Hata: Görev dosyası bulunamadı: {fpath}",
        "err_task_input_required": "Hata: Görev için -g/--gorev-dosyalari veya -m/--mesaj gereklidir.",
        "err_task_running": "Hata: '{etiket}' etiketli görev şu anda canlı çalışıyor (PID: {pid}).",
        "err_task_not_found": "Hata: Görev bulunamadı: {etiket}",
        "err_meta_read": "Hata: Görev meta dosyası okunamadı: {etiket}",
        "err_task_not_finished": "Hata: Görev '{etiket}' henüz bitmemiş. Devam ettirilemez.",
        "err_task_not_finished_verify": "Hata: Görev '{etiket}' henüz bitmemiş (canlı çalışıyor). Doğrulama için önce bitmesini bekleyin.",
        "err_task_not_verifiable": "Hata: Görev '{etiket}' doğrulanabilir durumda değil ({durum}). 'bitti' ya da 'hata' durumunda olmalı.",
        "err_no_conv_id": "Hata: '{etiket}' görevinde conversation_id bulunamadı.",
        "err_missing_permissions": "Hata: settings.json dosyasında eksik izinler var:",
        "hint_add_permissions": "\nİzinleri otomatik eklemek için '--izin-ekle' bayrağını kullanın.",
        "permissions_added": "İzinler settings.json'a eklendi (Yedek: {bak}):",
        "task_started_bg": "Görev '{etiket}' arka planda başlatıldı.",
        "task_started_round": "Görev '{etiket}' (Tur {tur}) başlatıldı.",
        "no_registered_tasks": "Kayıtlı görev bulunamadı.",
        "no_tasks_to_clean": "Temizlenecek görev bulunamadı.",
        "task_deleted": "Silindi: {entry}",
        "tasks_cleaned": "Toplam {cleaned} eski görev temizlendi.",
        "task_cancelled": "Görev '{etiket}' iptal edildi.",
        "all_permissions_present": "Tüm izinler mevcut.",
        "missing_permissions_detected": "Eksik izinler tespit edildi. settings.json dosyasına eklenmesi gereken kurallar:",
        "no_response_yet": "Henüz yanıt oluşturulmamış.",
        "no_report_yet": "Henüz rapor oluşturulmamış.",
        "err_report_read": "Hata: Rapor dosyası okunamadı: {err}",
        "err_cwd_not_found": "Hata: Çalışma dizini bulunamadı: {cwd}",
        "watching_events": "[{etiket}] Olay akışı izleniyor (Çıkmak için Ctrl+C)...",
        "watching_stopped": "\nİzleme durduruldu.",
        "event_file_not_found": "Olay dosyası bulunamadı.",
        "report_title": "=== Görev Raporu: {etiket} ===",
        "report_success": "Başarılı: {val}",
        "report_summary": "\nÖzet:\n  {ozet}",
        "report_changed_files": "\nDeğişen / Yazılan Dosyalar:",
        "report_tests": "\nÇalıştırılan Testler / Komutlar:",
        "report_open_issues": "\nAçık Konular:",
        "none_item": "  - Yok",
        "yes": "Evet",
        "no": "Hayır",
        "not_specified": "Belirtilmedi",
        "wait_completed": "[{etiket}] bitti (tur: {tur}, süre: {sure:.1f}s, yazılan: {yazilan} dosya)",
        "wait_error": "[{etiket}] hata{msg} (tur: {tur}, süre: {sure:.1f}s)",
        "wait_lost": "[{etiket}] kayip (süreç yanıt vermiyor veya sonlandı)",
        "wait_cancelled": "[{etiket}] iptal edildi",
        "wait_other": "[{etiket}] {st} (tur: {tur})",
        "headers": [
            "ETİKET",
            "MODEL",
            "DURUM",
            "DOGRULAMA",
            "TUR",
            "SÜRE",
            "SON ADIM",
            "DOSYA",
            "TOKEN",
        ],
    },
    "en": {
        "err_task_file_not_found": "Error: Task file not found: {fpath}",
        "err_task_input_required": "Error: Either -g/--gorev-dosyalari or -m/--mesaj is required for task.",
        "err_task_running": "Error: Task '{etiket}' is currently running (PID: {pid}).",
        "err_task_not_found": "Error: Task not found: {etiket}",
        "err_meta_read": "Error: Could not read task meta file: {etiket}",
        "err_task_not_finished": "Error: Task '{etiket}' is not finished yet. Cannot continue.",
        "err_task_not_finished_verify": "Error: Task '{etiket}' is not finished yet (currently running). Wait for completion before verification.",
        "err_task_not_verifiable": "Error: Task '{etiket}' is not in a verifiable state ({durum}). Status must be 'bitti' or 'hata'.",
        "err_no_conv_id": "Error: conversation_id not found in task '{etiket}'.",
        "err_missing_permissions": "Error: Missing permissions in settings.json:",
        "hint_add_permissions": "\nUse '--izin-ekle' flag to automatically add permissions.",
        "permissions_added": "Permissions added to settings.json (Backup: {bak}):",
        "task_started_bg": "Task '{etiket}' started in the background.",
        "task_started_round": "Task '{etiket}' (Round {tur}) started.",
        "no_registered_tasks": "No registered tasks found.",
        "no_tasks_to_clean": "No tasks to clean.",
        "task_deleted": "Deleted: {entry}",
        "tasks_cleaned": "Total {cleaned} old tasks cleaned.",
        "task_cancelled": "Task '{etiket}' cancelled.",
        "all_permissions_present": "All permissions present.",
        "missing_permissions_detected": "Missing permissions detected. Rules that need to be added to settings.json:",
        "no_response_yet": "No response generated yet.",
        "no_report_yet": "No report generated yet.",
        "err_report_read": "Error: Could not read report file: {err}",
        "err_cwd_not_found": "Error: Working directory not found: {cwd}",
        "watching_events": "[{etiket}] Watching event stream (Ctrl+C to exit)...",
        "watching_stopped": "\nWatching stopped.",
        "event_file_not_found": "Event file not found.",
        "report_title": "=== Task Report: {etiket} ===",
        "report_success": "Successful: {val}",
        "report_summary": "\nSummary:\n  {ozet}",
        "report_changed_files": "\nChanged / Written Files:",
        "report_tests": "\nExecuted Tests / Commands:",
        "report_open_issues": "\nOpen Issues:",
        "none_item": "  - None",
        "yes": "Yes",
        "no": "No",
        "not_specified": "Not specified",
        "wait_completed": "[{etiket}] completed (round: {tur}, duration: {sure:.1f}s, written: {yazilan} files)",
        "wait_error": "[{etiket}] error{msg} (round: {tur}, duration: {sure:.1f}s)",
        "wait_lost": "[{etiket}] lost (process not responding or terminated)",
        "wait_cancelled": "[{etiket}] cancelled",
        "wait_other": "[{etiket}] {st} (round: {tur})",
        "headers": [
            "TAG",
            "MODEL",
            "STATUS",
            "VERIFY",
            "ROUND",
            "DURATION",
            "LAST STEP",
            "FILES",
            "TOKENS",
        ],
    },
}


def tr(key: str, **kwargs: Any) -> str:
    """Anahtar ve değişkenlere göre yerelleştirilmiş metni döner."""
    lang = get_lang()
    template = MESSAGES.get(lang, MESSAGES["tr"]).get(
        key, MESSAGES["tr"].get(key, "")
    )
    if kwargs:
        return template.format(**kwargs)
    return str(template)


def get_state_root() -> str:
    """Durum dizini kökünü döner ($AJAN_DIZIN, $XDG_STATE_HOME/ajan veya ~/.local/state/ajan)."""
    if "AJAN_DIZIN" in os.environ:
        return os.path.abspath(os.path.expanduser(os.environ["AJAN_DIZIN"]))
    xdg_state = os.environ.get("XDG_STATE_HOME")
    if xdg_state:
        return os.path.abspath(os.path.join(os.path.expanduser(xdg_state), "ajan"))
    return os.path.abspath(os.path.expanduser("~/.local/state/ajan"))


def get_settings_path() -> str:
    """Gemini settings.json yolunu döner ($AJAN_AYAR veya varsayılan ~/.gemini/antigravity-cli/settings.json)."""
    return os.path.abspath(
        os.path.expanduser(
            os.environ.get(
                "AJAN_AYAR",
                os.path.expanduser("~/.gemini/antigravity-cli/settings.json"),
            )
        )
    )


def get_agy_binary() -> str:
    """agy çalıştırılabilir ikilisinin yolunu döner ($AJAN_AGY veya agy)."""
    return os.environ.get("AJAN_AGY", "agy")


def resolve_model(model_name: str | None) -> str:
    """Model takma adını tam model kimliğine çözümler."""
    if not model_name:
        return DEFAULT_MODEL
    return MODEL_ALIASES.get(model_name, model_name)


def path_covers(parent: str, child: str) -> bool:
    """parent dizininin child yolunu kapsayıp kapsamadığını kontrol eder."""
    parent_abs = os.path.abspath(parent)
    child_abs = os.path.abspath(child)
    if parent_abs == child_abs:
        return True
    try:
        rel = os.path.relpath(child_abs, parent_abs)
        return rel != "." and not rel.startswith("..") and not os.path.isabs(rel)
    except ValueError:
        return False


def check_permissions(
    yaz_dirs: list[str], komut_list: list[str], settings_path: str
) -> list[str]:
    """
    settings.json dosyasını denetler, eksik izin kurallarını döner.
    write_file kuralı dizin ve alt yollarını kapsar.
    command kuralı birebir eşleşmelidir.
    """
    allow_rules: list[str] = []
    if os.path.isfile(settings_path):
        try:
            with open(settings_path, "r", encoding="utf-8") as f:
                data = json.load(f)
                allow_rules = (
                    data.get("permissions", {}).get("allow", [])
                    if isinstance(data, dict)
                    else []
                )
        except Exception:
            allow_rules = []

    missing: list[str] = []

    # Yazma dizinleri kontrolü
    for d in yaz_dirs:
        abs_d = os.path.abspath(os.path.expanduser(d))
        covered = False
        for rule in allow_rules:
            if rule.startswith("write_file(") and rule.endswith(")"):
                rule_p = os.path.abspath(
                    os.path.expanduser(rule[len("write_file(") : -1].strip())
                )
                if path_covers(rule_p, abs_d):
                    covered = True
                    break
        if not covered:
            missing.append(f"write_file({abs_d})")

    # Komut önekleri kontrolü
    for cmd in komut_list:
        cmd_str = cmd.strip()
        cmd_rule = f"command({cmd_str})"
        if cmd_rule not in allow_rules:
            missing.append(cmd_rule)

    return missing


def add_permissions(missing_rules: list[str], settings_path: str) -> str:
    """
    Eksik kuralları settings.json dosyasına ekler.
    Öncesinde settings.json.bak-<zaman> yedeği oluşturur.
    """
    os.makedirs(os.path.dirname(os.path.abspath(settings_path)), exist_ok=True)
    data: dict[str, Any] = {}

    backup_path = ""
    if os.path.isfile(settings_path):
        ts = datetime.now().strftime("%Y%m%d-%H%M%S")
        backup_path = f"{settings_path}.bak-{ts}"
        i = 1
        while os.path.exists(backup_path):
            backup_path = f"{settings_path}.bak-{ts}-{i}"
            i += 1
        shutil.copy2(settings_path, backup_path)
        try:
            with open(settings_path, "r", encoding="utf-8") as f:
                loaded = json.load(f)
                if isinstance(loaded, dict):
                    data = loaded
        except Exception:
            data = {}

    if "permissions" not in data or not isinstance(data["permissions"], dict):
        data["permissions"] = {}
    if "allow" not in data["permissions"] or not isinstance(
        data["permissions"]["allow"], list
    ):
        data["permissions"]["allow"] = []

    for rule in missing_rules:
        if rule not in data["permissions"]["allow"]:
            data["permissions"]["allow"].append(rule)

    with open(settings_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")

    return backup_path


def extract_file_path(params: dict[str, Any]) -> str | None:
    """
    Araç parametrelerinden dosya yolunu çıkarır.
    Sırasıyla Path, TargetFile, AbsolutePath, path, file_path anahtarlarını,
    bulunamazsa '/' ile başlayan string değerleri dener.
    """
    if not isinstance(params, dict):
        return None
    keys = ["Path", "TargetFile", "AbsolutePath", "path", "file_path"]
    for k in keys:
        if k in params:
            v = params[k]
            if isinstance(v, str) and v.strip():
                return v.strip()
    for v in params.values():
        if isinstance(v, str) and v.strip().startswith("/"):
            return v.strip()
    return None


def extract_command(params: dict[str, Any]) -> str | None:
    """Araç parametrelerinden komut satırını çıkarır."""
    if not isinstance(params, dict):
        return None
    keys = ["CommandLine", "command", "cmd", "command_line"]
    for k in keys:
        if k in params:
            v = params[k]
            if isinstance(v, str) and v.strip():
                return v.strip()
    return None


def is_write_tool(tool_name: str | None) -> bool:
    """Aracın dosya yazma/değiştirme aracı olup olmadığını belirler."""
    if not tool_name:
        return False
    n = tool_name.lower()
    return any(
        kw in n for kw in ("write", "edit", "replace", "create", "save", "patch")
    )


def is_command_tool(tool_name: str | None) -> bool:
    """Aracın komut çalıştırma aracı olup olmadığını belirler."""
    if not tool_name:
        return False
    n = tool_name.lower()
    return any(kw in n for kw in ("command", "shell", "bash", "exec"))


def is_pid_alive(pid: int | None) -> bool:
    """Verilen PID'nin canlı olup olmadığını kontrol eder."""
    if not pid or pid <= 0:
        return False
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except OSError:
        return False


def gorev_canli(meta: dict[str, Any] | None) -> bool:
    """Görev süreci canlı mı: bekçi (agy'yi başlatan) VEYA agy canlıysa.

    baslat meta'yı yazıp bekçiyi başlatır; agy pid'i bekçi tarafından birkaç an sonra yazılır.
    Yalnız pid'e bakmak bu arada görevi "kayip" sayıyordu (yarış).
    """
    if not meta:
        return False
    return is_pid_alive(meta.get("bekci_pid")) or is_pid_alive(meta.get("pid"))


def save_meta(task_dir: str, meta: dict[str, Any]) -> None:
    """meta.json dosyasını atomik olarak yazar."""
    os.makedirs(task_dir, exist_ok=True)
    meta_path = os.path.join(task_dir, "meta.json")
    tmp_path = os.path.join(task_dir, "meta.json.tmp")
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp_path, meta_path)


def load_meta(task_dir: str) -> dict[str, Any] | None:
    """meta.json dosyasını yükler."""
    meta_path = os.path.join(task_dir, "meta.json")
    if not os.path.isfile(meta_path):
        return None
    try:
        with open(meta_path, "r", encoding="utf-8") as f:
            data = json.load(f)
            return data if isinstance(data, dict) else None
    except Exception:
        return None


def build_gemini_md(
    yaz_list: list[str], komut_list: list[str], gorev_metni: str
) -> str:
    """GEMINI.md için sabit ajan kurallarını ve görev metnini birleştirir."""
    if get_lang() == "en":
        yaz_str = (
            ", ".join(yaz_list)
            if yaz_list
            else "none (write permission not granted)"
        )
        komut_str = (
            ", ".join(komut_list)
            if komut_list
            else "none (terminal disabled)"
        )
        return (
            "# Fixed Agent Rules\n"
            f"- You may only write to the following directories: {yaz_str}. Any attempt to write elsewhere will be rejected and the task will fail.\n"
            f"- Allowed commands: {komut_str}. Commands must prefix-match.\n"
            "- Do not modify or delete files not requested by the task. Do not relax existing tests.\n"
            '- When finished, append a JSON block at the END of your response (```json ... ```): {"ozet": "...", '
            '"degisen_dosyalar": ["..."], "calistirilan_testler": ["command: result"], "acik_konular": ["..."], '
            '"basarili": true|false}. ajan extracts this block into rapor.json (if not found, derives from written files and response with "basarili": null).\n\n'
            "---\n"
            f"## Task\n{gorev_metni}\n"
        )

    yaz_str = ", ".join(yaz_list) if yaz_list else "yok (yazma izni verilmedi)"
    komut_str = ", ".join(komut_list) if komut_list else "yok (terminal kapalı)"

    return (
        "# Sabit Ajan Kuralları\n"
        f"- Yalnız şu dizinlere yazabilirsin: {yaz_str}. Başka yere yazma girişimi reddedilir ve görev başarısız olur.\n"
        f"- Çalıştırabileceğin komutlar: {komut_str}. Komutun önek eşleşmesi gerekir.\n"
        "- Görevde istenmeyen dosyaları değiştirme, silme. Var olan testleri gevşetme.\n"
        '- Bitirince yanıtının SONUNA şu biçimde bir JSON bloğu koy (```json ... ```): {"ozet": "...", '
        '"degisen_dosyalar": ["..."], "calistirilan_testler": ["komut: sonuç"], "acik_konular": ["..."], '
        '"basarili": true|false}. ajan bu bloğu rapor.json\'a çıkarır (bulamazsa yazilan_dosyalar\'dan ve yanıttan '
        'rapor üretir, "basarili": null).\n\n'
        "---\n"
        f"## Görev\n{gorev_metni}\n"
    )


def extract_rapor(
    response_text: str, yazilan_dosyalar: list[str], calistirilan_komutlar: list[str]
) -> dict[str, Any]:
    """
    Yanıttaki JSON bloğunu ayrıştırarak rapor sözlüğü üretir.
    Bulamazsa yazilan_dosyalar ve yanıttan türetir ('basarili': None).
    """
    matches = list(
        re.finditer(r"```(?:json)?\s*(\{.*?\})\s*```", response_text, re.DOTALL)
    )
    parsed: dict[str, Any] | None = None
    if matches:
        for m in reversed(matches):
            try:
                candidate = json.loads(m.group(1))
                if isinstance(candidate, dict):
                    parsed = candidate
                    break
            except Exception:
                continue

    if parsed is None:
        # Kod bloğu dışında çıplak JSON dene
        raw_match = re.search(
            r'(\{\s*"(?:ozet|summary)"\s*:.*?\})\s*$', response_text, re.DOTALL
        )
        if raw_match:
            try:
                candidate = json.loads(raw_match.group(1))
                if isinstance(candidate, dict):
                    parsed = candidate
            except Exception:
                pass

    if parsed is not None:
        ozet_val = parsed.get("ozet")
        if ozet_val is None:
            ozet_val = parsed.get("summary", "")

        degisen_val = parsed.get("degisen_dosyalar")
        if degisen_val is None:
            degisen_val = parsed.get("changed_files", list(yazilan_dosyalar))

        testler_val = parsed.get("calistirilan_testler")
        if testler_val is None:
            testler_val = parsed.get("calistirilan_komutlar") or parsed.get(
                "executed_tests", list(calistirilan_komutlar)
            )

        acik_val = parsed.get("acik_konular")
        if acik_val is None:
            acik_val = parsed.get("open_issues", [])

        basarili_val = parsed.get("basarili")
        if basarili_val is None and "success" in parsed:
            basarili_val = parsed["success"]
        if basarili_val is None and "basarili" not in parsed:
            basarili_val = True

        return {
            "ozet": ozet_val,
            "degisen_dosyalar": degisen_val,
            "calistirilan_testler": testler_val,
            "acik_konular": acik_val,
            "basarili": basarili_val,
        }

    return {
        "ozet": response_text.strip()[:300] if response_text else "",
        "degisen_dosyalar": list(yazilan_dosyalar),
        "calistirilan_testler": list(calistirilan_komutlar),
        "acik_konular": [],
        "basarili": None,
    }


def get_last_error_from_log(hata_path: str) -> str | None:
    """hata.log içindeki son ERROR veya hata satırını bulur."""
    if not os.path.isfile(hata_path):
        return None
    try:
        with open(hata_path, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f if line.strip()]
        for line in reversed(lines):
            if "ERROR" in line.upper() or "HATA" in line.upper():
                return line
        return lines[-1] if lines else None
    except Exception:
        return None


def extract_dogrulama_ozet(cikti: str, limit: int = 5) -> str:
    """
    Doğrulama komutu çıktısından ilk FAILED/Error satırlarını özet olarak çıkarır.
    """
    if not cikti:
        return ""
    eslesenler: list[str] = []
    for line in cikti.splitlines():
        line_clean = line.strip()
        if not line_clean:
            continue
        l_upper = line_clean.upper()
        if "FAILED" in l_upper or "ERROR" in l_upper or "FAIL" in l_upper:
            eslesenler.append(line_clean)
            if len(eslesenler) >= limit:
                break
    if eslesenler:
        return "\n".join(eslesenler)
    lines = [l.strip() for l in cikti.splitlines() if l.strip()]
    return lines[-1] if lines else ""


def run_dogrulama_komut(
    komut: str,
    cwd: str | None = None,
    zaman_asimi: float | None = None,
) -> tuple[int, str]:
    """
    Doğrulama komutunu çalıştırır ve (çıkış_kodu, birleşik_çıktı) döner.
    """
    try:
        proc = subprocess.run(
            komut,
            shell=True,
            cwd=cwd,
            timeout=zaman_asimi,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        return proc.returncode, proc.stdout or ""
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or "") if isinstance(e.stdout, str) else ""
        if e.stderr:
            out += "\n" + (e.stderr if isinstance(e.stderr, str) else "")
        out += f"\nZaman aşımı ({zaman_asimi}s) aşıldı."
        return 124, out
    except Exception as e:
        return 1, f"Komut çalıştırılamadı: {e}"


# ---------------------------------------------------------------------------
# Bekçi Süreci
# ---------------------------------------------------------------------------
def run_bekci(etiket: str) -> None:
    """
    Arka planda çalışan bekçi süreç (`ajan _bekci ETIKET`).
    agy sürecini yönetir, stream-json çıktısını okur ve meta.json'u günceller.
    """
    state_root = get_state_root()
    task_dir = os.path.join(state_root, etiket)
    meta = load_meta(task_dir)
    if not meta:
        sys.exit(1)

    calisma_dir = os.path.join(task_dir, "calisma")
    os.makedirs(calisma_dir, exist_ok=True)

    olaylar_path = os.path.join(task_dir, "olaylar.jsonl")
    hata_path = os.path.join(task_dir, "hata.log")
    yanit_path = os.path.join(task_dir, "yanit.md")
    rapor_path = os.path.join(task_dir, "rapor.json")

    start_time = time.time()
    meta["baslangic_zamani"] = datetime.now().isoformat()
    meta["bekci_pid"] = os.getpid()
    save_meta(task_dir, meta)

    agy_bin = get_agy_binary()

    tur = meta.get("tur", 1)
    if tur == 1:
        pointer_prompt = "Complete the TASK described in your GEMINI.md."
    else:
        followup = meta.get("takip_mesaji", "")
        pointer_prompt = (
            followup or "Complete the follow-up task described in your GEMINI.md."
        )

    cmd = [agy_bin, "-p", pointer_prompt, "--output-format", "stream-json"]

    model_id = resolve_model(meta.get("model", "flash"))
    cmd.extend(["--model", model_id])

    if meta.get("effort"):
        cmd.extend(["--effort", str(meta["effort"])])

    # --add-dir listesi
    add_dirs: list[str] = []
    for d in meta.get("yaz", []) + meta.get("oku", []):
        abs_d = os.path.abspath(os.path.expanduser(d))
        if abs_d not in add_dirs:
            add_dirs.append(abs_d)
    for d in add_dirs:
        cmd.extend(["--add-dir", d])

    if meta.get("conversation_id"):
        cmd.extend(["--conversation", str(meta["conversation_id"])])

    if meta.get("sure"):
        cmd.extend(["--print-timeout", str(meta["sure"])])

    if meta.get("sandbox"):
        cmd.append("--sandbox")

    agy_proc: subprocess.Popen[str] | None = None
    try:
        agy_proc = subprocess.Popen(
            cmd,
            cwd=calisma_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
    except Exception as e:
        with open(hata_path, "a", encoding="utf-8") as hf:
            hf.write(f"ERROR: agy başlatılamadı: {e}\n")
        meta["durum"] = "hata"
        meta["hata"] = f"agy başlatılamadı: {e}"
        meta["sure_s"] = round(time.time() - start_time, 2)
        prev_total = sum(
            float(t.get("sure_s", 0.0)) for t in meta.get("turler", [])
        )
        meta["toplam_sure_s"] = round(prev_total + meta["sure_s"], 2)
        save_meta(task_dir, meta)

        default_rapor = {
            "ozet": "",
            "degisen_dosyalar": [],
            "calistirilan_testler": [],
            "acik_konular": [],
            "basarili": False,
        }
        with open(rapor_path, "w", encoding="utf-8") as rf:
            json.dump(default_rapor, rf, indent=2, ensure_ascii=False)
        with open(yanit_path, "w", encoding="utf-8") as yf:
            yf.write("")
        sys.exit(1)

    meta["pid"] = agy_proc.pid
    save_meta(task_dir, meta)

    hata_file = open(hata_path, "a", encoding="utf-8")
    olaylar_file = open(olaylar_path, "a", encoding="utf-8")

    def read_stderr():
        assert agy_proc is not None
        assert agy_proc.stderr is not None
        for err_line in iter(agy_proc.stderr.readline, ""):
            hata_file.write(err_line)
            hata_file.flush()

    stderr_thread = threading.Thread(target=read_stderr, daemon=True)
    stderr_thread.start()

    final_response = ""
    result_status = None
    result_duration = None

    yazilan_dosyalar: list[str] = list(meta.get("yazilan_dosyalar", []))
    calistirilan_komutlar: list[str] = list(meta.get("calistirilan_komutlar", []))

    assert agy_proc.stdout is not None
    for line in iter(agy_proc.stdout.readline, ""):
        olaylar_file.write(line)
        olaylar_file.flush()

        line_str = line.strip()
        if not line_str:
            continue

        try:
            ev = json.loads(line_str)
        except Exception:
            continue

        event_type = ev.get("event")

        if event_type == "init":
            cid = ev.get("conversation_id")
            if cid:
                meta["conversation_id"] = cid
                save_meta(task_dir, meta)

        elif event_type == "step_update":
            su = ev.get("step_update", {})
            cid = su.get("conversation_id")
            if cid and not meta.get("conversation_id"):
                meta["conversation_id"] = cid

            step_index = su.get("step_index", 0)
            tool_name = su.get("tool_name")
            tool_info = su.get("tool_info", {})
            params = (
                tool_info.get("parameters", {})
                if isinstance(tool_info, dict)
                else {}
            )

            if tool_name:
                if is_write_tool(tool_name):
                    p = extract_file_path(params)
                    if p and p not in yazilan_dosyalar:
                        yazilan_dosyalar.append(p)
                        meta["yazilan_dosyalar"] = yazilan_dosyalar
                    detail = os.path.basename(p) if p else ""
                    meta["son_adim"] = f"{tool_name} {detail}".strip()
                elif is_command_tool(tool_name):
                    cmd_str = extract_command(params)
                    if cmd_str and cmd_str not in calistirilan_komutlar:
                        calistirilan_komutlar.append(cmd_str)
                        meta["calistirilan_komutlar"] = calistirilan_komutlar
                    detail = cmd_str.split()[0] if cmd_str else ""
                    meta["son_adim"] = f"{tool_name} {detail}".strip()
                else:
                    p = extract_file_path(params)
                    detail = os.path.basename(p) if p else ""
                    meta["son_adim"] = f"{tool_name} {detail}".strip()
            elif su.get("step_type"):
                meta["son_adim"] = str(su.get("step_type"))
            else:
                meta["son_adim"] = f"adım {step_index}"

            usage = su.get("usage")
            if isinstance(usage, dict):
                for k in ["input_tokens", "output_tokens", "total_tokens"]:
                    if k in usage and usage[k] is not None:
                        meta["token_toplami"][k] = max(
                            meta["token_toplami"].get(k, 0), int(usage[k])
                        )

            save_meta(task_dir, meta)

        elif event_type == "result":
            res = ev.get("result", {})
            cid = res.get("conversation_id")
            if cid:
                meta["conversation_id"] = cid
            result_status = res.get("status")
            final_response = res.get("response", "") or ""
            result_duration = res.get("duration_seconds")
            usage = res.get("usage")
            if isinstance(usage, dict):
                for k in ["input_tokens", "output_tokens", "total_tokens"]:
                    if k in usage and usage[k] is not None:
                        meta["token_toplami"][k] = max(
                            meta["token_toplami"].get(k, 0), int(usage[k])
                        )

    agy_proc.wait()
    stderr_thread.join(timeout=2.0)
    hata_file.close()
    olaylar_file.close()

    # İptal kontrolü (dışarıdan iptal edilmiş olabilir)
    latest_meta = load_meta(task_dir)
    if latest_meta and latest_meta.get("durum") == "iptal":
        meta["durum"] = "iptal"
    else:
        if result_status == "SUCCESS" and final_response.strip():
            meta["durum"] = "bitti"
        else:
            meta["durum"] = "hata"
            if result_status and result_status != "SUCCESS":
                meta["hata"] = str(result_status)
            elif not final_response.strip():
                meta["hata"] = "Boş yanıt döndü"
            else:
                last_err = get_last_error_from_log(hata_path)
                meta["hata"] = (
                    last_err or f"agy çıkış kodu: {agy_proc.returncode}"
                )

    elapsed = round(time.time() - start_time, 2)
    tur = meta.get("tur", 1)
    if tur > 1:
        meta["sure_s"] = elapsed
    else:
        meta["sure_s"] = (
            float(result_duration) if result_duration is not None else elapsed
        )

    prev_total = sum(
        float(t.get("sure_s", 0.0)) for t in meta.get("turler", [])
    )
    meta["toplam_sure_s"] = round(prev_total + meta["sure_s"], 2)

    with open(yanit_path, "w", encoding="utf-8") as yf:
        yf.write(final_response)

    rapor = extract_rapor(
        final_response, yazilan_dosyalar, calistirilan_komutlar
    )
    if meta["durum"] == "hata" and rapor.get("basarili") is None:
        rapor["basarili"] = False

    with open(rapor_path, "w", encoding="utf-8") as rf:
        json.dump(rapor, rf, indent=2, ensure_ascii=False)
        rf.write("\n")

    save_meta(task_dir, meta)


# ---------------------------------------------------------------------------
# Komut Yöneticileri
# ---------------------------------------------------------------------------
def cmd_baslat(args: argparse.Namespace) -> None:
    etiket = args.etiket
    state_root = get_state_root()
    task_dir = os.path.join(state_root, etiket)
    settings_path = get_settings_path()

    yaz_list = [
        os.path.abspath(os.path.expanduser(p)) for p in (args.yaz or [])
    ]
    oku_list = [
        os.path.abspath(os.path.expanduser(p)) for p in (args.oku or [])
    ]
    komut_list = [c.strip() for c in (args.komut or []) if c.strip()]

    # 1. Görev metnini belirle
    if args.gorev_dosyalari:
        contents: list[str] = []
        for fpath in args.gorev_dosyalari:
            p = os.path.abspath(os.path.expanduser(fpath))
            if not os.path.isfile(p):
                print(
                    tr("err_task_file_not_found", fpath=fpath), file=sys.stderr
                )
                sys.exit(1)
            with open(p, "r", encoding="utf-8") as f:
                contents.append(f.read())
        gorev_metni = "\n\n".join(contents)
    elif args.mesaj:
        gorev_metni = args.mesaj
    else:
        print(
            tr("err_task_input_required"),
            file=sys.stderr,
        )
        sys.exit(1)

    # 2. Canlı görev kontrolü
    if os.path.exists(task_dir):
        meta = load_meta(task_dir)
        if meta and meta.get("durum") == "calisiyor":
            pid = meta.get("pid")
            if gorev_canli(meta):
                print(
                    tr("err_task_running", etiket=etiket, pid=pid),
                    file=sys.stderr,
                )
                sys.exit(3)
        # Bitmiş veya ölü görev varsa arşivle
        ts = datetime.now().strftime("%Y%m%d-%H%M%S")
        archive_dir = f"{task_dir}.{ts}"
        i = 1
        while os.path.exists(archive_dir):
            archive_dir = f"{task_dir}.{ts}-{i}"
            i += 1
        shutil.move(task_dir, archive_dir)

    # 3. İzin ön kontrolü
    missing_permissions = check_permissions(
        yaz_list, komut_list, settings_path
    )
    if missing_permissions:
        if args.izin_ekle:
            bak = add_permissions(missing_permissions, settings_path)
            print(tr("permissions_added", bak=bak))
            for r in missing_permissions:
                print(f"  + {r}")
        else:
            print(
                tr("err_missing_permissions"),
                file=sys.stderr,
            )
            for r in missing_permissions:
                print(f"  {r}", file=sys.stderr)
            print(
                tr("hint_add_permissions"),
                file=sys.stderr,
            )
            sys.exit(2)

    # 4. Dizin ve GEMINI.md hazırlığı
    os.makedirs(task_dir, exist_ok=True)
    calisma_dir = os.path.join(task_dir, "calisma")
    os.makedirs(calisma_dir, exist_ok=True)

    gemini_content = build_gemini_md(yaz_list, komut_list, gorev_metni)
    gemini_path = os.path.join(task_dir, "GEMINI.md")
    with open(gemini_path, "w", encoding="utf-8") as gf:
        gf.write(gemini_content)

    calisma_gemini = os.path.join(calisma_dir, "GEMINI.md")
    try:
        if os.path.islink(calisma_gemini) or os.path.exists(calisma_gemini):
            os.remove(calisma_gemini)
        os.symlink("../GEMINI.md", calisma_gemini)
    except Exception:
        shutil.copy2(gemini_path, calisma_gemini)

    # 5. Sandbox belirleme: --sandbox yoksa ve komut yoksa daima sandbox
    use_sandbox = bool(args.sandbox or (len(komut_list) == 0))

    initial_meta: dict[str, Any] = {
        "etiket": etiket,
        "model": args.model,
        "effort": args.effort,
        "yaz": yaz_list,
        "oku": oku_list,
        "komut": komut_list,
        "sure": args.sure,
        "sandbox": use_sandbox,
        "durum": "calisiyor",
        "baslangic_zamani": datetime.now().isoformat(),
        "pid": None,
        "bekci_pid": None,
        "tur": 1,
        "turler": [],
        "dogrulama": [],
        "son_adim": None,
        "yazilan_dosyalar": [],
        "calistirilan_komutlar": [],
        "token_toplami": {
            "input_tokens": 0,
            "output_tokens": 0,
            "total_tokens": 0,
        },
        "sure_s": 0.0,
        "conversation_id": None,
        "hata": None,
    }
    save_meta(task_dir, initial_meta)

    # 6. Bekçi sürecini başlat
    script_path = os.path.abspath(__file__)
    bekci = subprocess.Popen(
        [sys.executable, script_path, "_bekci", etiket],
        start_new_session=True,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        close_fds=True,
    )
    # bekçi kendi meta yazımından önce okunursa da canlı görünsün (yarış)
    initial_meta["bekci_pid"] = bekci.pid
    save_meta(task_dir, initial_meta)

    if args.bekle:
        code = wait_for_tasks([etiket], zaman_asimi=None)
        sys.exit(code)
    else:
        print(tr("task_started_bg", etiket=etiket))
        sys.exit(0)


def calistir_devam(
    etiket: str,
    mesaj: str,
    bekle: bool = False,
    zaman_asimi: float | None = None,
) -> int:
    state_root = get_state_root()
    task_dir = os.path.join(state_root, etiket)
    if not os.path.exists(task_dir):
        print(tr("err_task_not_found", etiket=etiket), file=sys.stderr)
        return 1

    meta = load_meta(task_dir)
    if not meta:
        print(
            tr("err_meta_read", etiket=etiket), file=sys.stderr
        )
        return 1

    if meta.get("durum") == "calisiyor" and gorev_canli(meta):
        print(
            tr("err_task_not_finished", etiket=etiket),
            file=sys.stderr,
        )
        return 1

    conversation_id = meta.get("conversation_id")
    if not conversation_id:
        print(
            tr("err_no_conv_id", etiket=etiket),
            file=sys.stderr,
        )
        return 1

    current_tur = meta.get("tur", 1)
    turler = meta.get("turler", [])
    turler.append(
        {
            "tur": current_tur,
            "baslangic": meta.get("baslangic_zamani")
            or datetime.now().isoformat(),
            "sure_s": meta.get("sure_s", 0),
            "durum": meta.get("durum"),
            "son_adim": meta.get("son_adim"),
            "yazilan_dosyalar": list(meta.get("yazilan_dosyalar", [])),
            "calistirilan_komutlar": list(
                meta.get("calistirilan_komutlar", [])
            ),
        }
    )
    meta["turler"] = turler
    meta["tur"] = current_tur + 1
    meta["durum"] = "calisiyor"
    meta["baslangic_zamani"] = datetime.now().isoformat()
    meta["sure_s"] = 0.0
    meta["takip_mesaji"] = mesaj
    meta["hata"] = None
    save_meta(task_dir, meta)

    gemini_path = os.path.join(task_dir, "GEMINI.md")
    if os.path.exists(gemini_path):
        with open(gemini_path, "a", encoding="utf-8") as f:
            f.write(f"\n\n---\n## Takip Turu {meta['tur']}\n{mesaj}\n")

    script_path = os.path.abspath(__file__)
    bekci = subprocess.Popen(
        [sys.executable, script_path, "_bekci", etiket],
        start_new_session=True,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        close_fds=True,
    )
    meta["bekci_pid"] = bekci.pid
    meta["pid"] = None  # önceki turun agy pid'i ölü; yenisini bekçi yazar
    save_meta(task_dir, meta)

    if bekle:
        return wait_for_tasks([etiket], zaman_asimi=zaman_asimi)
    else:
        print(tr("task_started_round", etiket=etiket, tur=meta["tur"]))
        return 0


def cmd_devam(args: argparse.Namespace) -> None:
    code = calistir_devam(args.etiket, args.mesaj, bekle=args.bekle)
    sys.exit(code)


def cmd_dogrula(args: argparse.Namespace) -> None:
    etiket = args.etiket
    komut = args.test
    state_root = get_state_root()
    task_dir = os.path.join(state_root, etiket)
    if not os.path.exists(task_dir):
        print(tr("err_task_not_found", etiket=etiket), file=sys.stderr)
        sys.exit(1)

    meta = load_meta(task_dir)
    if not meta:
        print(
            tr("err_meta_read", etiket=etiket), file=sys.stderr
        )
        sys.exit(1)

    # 1. ETIKET "bitti" ya da "hata" durumunda olmalı (çalışıyorsa önce bekle)
    if meta.get("durum") == "calisiyor" and gorev_canli(meta):
        print(
            tr("err_task_not_finished_verify", etiket=etiket),
            file=sys.stderr,
        )
        sys.exit(1)

    durum = meta.get("durum")
    if durum not in ("bitti", "hata"):
        print(
            tr("err_task_not_verifiable", etiket=etiket, durum=durum),
            file=sys.stderr,
        )
        sys.exit(1)

    cwd = None
    if args.cwd:
        cwd = os.path.abspath(os.path.expanduser(args.cwd))
        if not os.path.isdir(cwd):
            print(
                tr("err_cwd_not_found", cwd=args.cwd), file=sys.stderr
            )
            sys.exit(1)

    max_tur = max(1, args.tur)
    tur = 1
    while tur <= max_tur:
        cikis, cikti = run_dogrulama_komut(
            komut, cwd=cwd, zaman_asimi=args.zaman_asimi
        )
        if cikis == 0:
            record = {
                "tur": tur,
                "komut": komut,
                "cikis": 0,
                "ozet": "",
                "zaman": datetime.now().isoformat(),
            }
            dogrulama_list = meta.get("dogrulama")
            if not isinstance(dogrulama_list, list):
                dogrulama_list = []
            dogrulama_list.append(record)
            meta["dogrulama"] = dogrulama_list
            save_meta(task_dir, meta)

            print("DOGRULANDI")
            sys.exit(0)

        # Başarısızlık kaydı
        ozet = extract_dogrulama_ozet(cikti)
        record = {
            "tur": tur,
            "komut": komut,
            "cikis": cikis,
            "ozet": ozet,
            "zaman": datetime.now().isoformat(),
        }
        dogrulama_list = meta.get("dogrulama")
        if not isinstance(dogrulama_list, list):
            dogrulama_list = []
        dogrulama_list.append(record)
        meta["dogrulama"] = dogrulama_list
        save_meta(task_dir, meta)

        if tur >= max_tur:
            break

        lines = cikti.splitlines()
        son_60 = (
            "\n".join(lines[-60:]) if len(lines) > 60 else "\n".join(lines)
        )
        if get_lang() == "en":
            mesaj = (
                f"Verification command failed: {komut}\n"
                f"```\n{son_60}\n```\n"
                "Find and fix the root cause. Do not relax tests."
            )
        else:
            mesaj = (
                f"Doğrulama komutu başarısız: {komut}\n"
                f"```\n{son_60}\n```\n"
                "Kök nedeni bul ve düzelt. Testleri gevşetme."
            )

        code = calistir_devam(etiket, mesaj, bekle=True)
        meta_after = load_meta(task_dir)
        if meta_after:
            meta = meta_after
        else:
            break

        durum = meta.get("durum")
        if durum not in ("bitti", "hata"):
            break

        tur += 1

    print(f"DOGRULANAMADI ({tur} tur)")
    sys.exit(1)


def cmd_durum(args: argparse.Namespace) -> None:
    state_root = get_state_root()
    etiketler: list[str] = []
    if args.etiket:
        etiketler = [args.etiket]
    else:
        if os.path.exists(state_root):
            for entry in sorted(os.listdir(state_root)):
                tdir = os.path.join(state_root, entry)
                if os.path.isdir(tdir) and os.path.exists(
                    os.path.join(tdir, "meta.json")
                ):
                    etiketler.append(entry)

    items: list[dict[str, Any]] = []
    for etiket in etiketler:
        task_dir = os.path.join(state_root, etiket)
        meta = load_meta(task_dir)
        if not meta:
            continue

        durum = meta.get("durum", "bilinmiyor")
        pid = meta.get("pid")
        if durum == "calisiyor" and not gorev_canli(meta):
            durum = "kayip"

        sure = meta.get("sure_s", 0)
        tokens = (
            meta.get("token_toplami", {}).get("total_tokens", 0)
            if isinstance(meta.get("token_toplami"), dict)
            else 0
        )
        yazilan_sayisi = len(meta.get("yazilan_dosyalar", []))
        son_adim = meta.get("son_adim") or "-"
        model = meta.get("model") or "flash"
        tur = meta.get("tur", 1)

        dogrulama_sonucu = "-"
        dogrulama_list = meta.get("dogrulama")
        if isinstance(dogrulama_list, list) and dogrulama_list:
            son_dog = dogrulama_list[-1]
            if isinstance(son_dog, dict):
                cikis = son_dog.get("cikis")
                if cikis == 0:
                    dogrulama_sonucu = "✓"
                else:
                    dogrulama_sonucu = "✗"

        items.append(
            {
                "etiket": etiket,
                "model": model,
                "durum": durum,
                "dogrulama": dogrulama_sonucu,
                "tur": tur,
                "sure": sure,
                "toplam_sure": meta.get("toplam_sure_s", sure),
                "son_adim": son_adim,
                "yazilan_dosya_sayisi": yazilan_sayisi,
                "token": tokens,
            }
        )

    if args.json:
        if args.etiket:
            print(
                json.dumps(
                    items[0] if items else {}, indent=2, ensure_ascii=False
                )
            )
        else:
            print(json.dumps(items, indent=2, ensure_ascii=False))
        return

    if not items:
        print(tr("no_registered_tasks"))
        return

    headers = list(MESSAGES.get(get_lang(), MESSAGES["tr"])["headers"])
    rows: list[list[str]] = []
    for it in items:
        sure_val = it["sure"]
        sure_str = (
            f"{sure_val:.1f}s"
            if isinstance(sure_val, (int, float))
            else str(sure_val)
        )
        rows.append(
            [
                str(it["etiket"]),
                str(it["model"]),
                str(it["durum"]),
                str(it["dogrulama"]),
                str(it["tur"]),
                sure_str,
                str(it["son_adim"]),
                str(it["yazilan_dosya_sayisi"]),
                str(it["token"]),
            ]
        )

    col_widths = [len(h) for h in headers]
    for row in rows:
        for idx, cell in enumerate(row):
            col_widths[idx] = max(col_widths[idx], len(cell))

    header_line = "  ".join(
        h.ljust(col_widths[i]) for i, h in enumerate(headers)
    )
    sep_line = "  ".join("-" * col_widths[i] for i in range(len(headers)))
    print(header_line)
    print(sep_line)
    for row in rows:
        print("  ".join(cell.ljust(col_widths[i]) for i, cell in enumerate(row)))


def wait_for_tasks(
    target_etikets: list[str] | None, zaman_asimi: float | None = None
) -> int:
    """
    Belirtilen görevler bitene kadar bekler.
    Her biri için tek satır özet yazar. Hepsi 'bitti' ise 0, aksi 1 döner.
    """
    state_root = get_state_root()
    if not target_etikets:
        target_etikets = []
        if os.path.exists(state_root):
            for entry in sorted(os.listdir(state_root)):
                tdir = os.path.join(state_root, entry)
                meta = load_meta(tdir)
                if meta and meta.get("durum") == "calisiyor":
                    target_etikets.append(entry)

    if not target_etikets:
        return 0

    start_wait = time.time()
    finished: dict[str, str] = {}

    while True:
        all_done = True
        for etiket in target_etikets:
            if etiket in finished:
                continue
            task_dir = os.path.join(state_root, etiket)
            meta = load_meta(task_dir)
            if not meta:
                finished[etiket] = "kayip"
                continue

            durum = meta.get("durum")
            pid = meta.get("pid")
            if durum == "calisiyor":
                if not gorev_canli(meta):
                    durum = "kayip"
                    finished[etiket] = durum
                else:
                    all_done = False
            else:
                finished[etiket] = str(durum)

        if all_done:
            break

        if zaman_asimi is not None and (time.time() - start_wait) >= zaman_asimi:
            for etiket in target_etikets:
                if etiket not in finished:
                    finished[etiket] = "zaman_asimi"
            break

        time.sleep(0.2)

    all_bitti = True
    for etiket in target_etikets:
        task_dir = os.path.join(state_root, etiket)
        meta = load_meta(task_dir) or {}
        st = finished.get(etiket, "bilinmiyor")
        sure = meta.get("sure_s", 0.0)
        tur = meta.get("tur", 1)
        yazilan = len(meta.get("yazilan_dosyalar", []))
        hata = meta.get("hata")

        if st == "bitti":
            print(
                tr(
                    "wait_completed",
                    etiket=etiket,
                    tur=tur,
                    sure=sure,
                    yazilan=yazilan,
                )
            )
        elif st == "hata":
            all_bitti = False
            msg = f": {hata}" if hata else ""
            print(
                tr(
                    "wait_error",
                    etiket=etiket,
                    msg=msg,
                    tur=tur,
                    sure=sure,
                )
            )
        elif st == "kayip":
            all_bitti = False
            print(tr("wait_lost", etiket=etiket))
        elif st == "iptal":
            all_bitti = False
            print(tr("wait_cancelled", etiket=etiket))
        else:
            all_bitti = False
            print(tr("wait_other", etiket=etiket, st=st, tur=tur))

    return 0 if all_bitti else 1


def cmd_bekle(args: argparse.Namespace) -> None:
    code = wait_for_tasks(
        args.etiketler if args.etiketler else None,
        zaman_asimi=args.zaman_asimi,
    )
    sys.exit(code)


def cmd_rapor(args: argparse.Namespace) -> None:
    etiket = args.etiket
    state_root = get_state_root()
    task_dir = os.path.join(state_root, etiket)
    if not os.path.exists(task_dir):
        print(tr("err_task_not_found", etiket=etiket), file=sys.stderr)
        sys.exit(1)

    if args.ham:
        yanit_path = os.path.join(task_dir, "yanit.md")
        if os.path.exists(yanit_path):
            with open(yanit_path, "r", encoding="utf-8") as f:
                print(f.read())
        else:
            print(tr("no_response_yet"))
        return

    rapor_path = os.path.join(task_dir, "rapor.json")
    if not os.path.exists(rapor_path):
        print(tr("no_report_yet"))
        return

    try:
        with open(rapor_path, "r", encoding="utf-8") as f:
            rapor = json.load(f)
    except Exception as e:
        print(tr("err_report_read", err=e), file=sys.stderr)
        sys.exit(1)

    basarili_val = rapor.get("basarili")
    if basarili_val is True:
        basarili_str = tr("yes")
    elif basarili_val is False:
        basarili_str = tr("no")
    else:
        basarili_str = tr("not_specified")

    print(tr("report_title", etiket=etiket))
    print(tr("report_success", val=basarili_str))
    print(tr("report_summary", ozet=rapor.get("ozet", "-")))

    degisen = rapor.get("degisen_dosyalar", [])
    print(tr("report_changed_files"))
    if degisen:
        for d in degisen:
            print(f"  - {d}")
    else:
        print(tr("none_item"))

    testler = rapor.get("calistirilan_testler", [])
    print(tr("report_tests"))
    if testler:
        for t in testler:
            print(f"  - {t}")
    else:
        print(tr("none_item"))

    acik = rapor.get("acik_konular", [])
    print(tr("report_open_issues"))
    if acik:
        for a in acik:
            print(f"  - {a}")
    else:
        print(tr("none_item"))


def cmd_izle(args: argparse.Namespace) -> None:
    etiket = args.etiket
    state_root = get_state_root()
    task_dir = os.path.join(state_root, etiket)
    if not os.path.exists(task_dir):
        print(tr("err_task_not_found", etiket=etiket), file=sys.stderr)
        sys.exit(1)

    olaylar_path = os.path.join(task_dir, "olaylar.jsonl")
    print(tr("watching_events", etiket=etiket))

    waited = 0
    while not os.path.exists(olaylar_path) and waited < 50:
        time.sleep(0.1)
        waited += 1

    if not os.path.exists(olaylar_path):
        print(tr("event_file_not_found"))
        return

    with open(olaylar_path, "r", encoding="utf-8") as f:
        try:
            while True:
                line = f.readline()
                if line:
                    line_str = line.strip()
                    if not line_str:
                        continue
                    try:
                        ev = json.loads(line_str)
                        ev_type = ev.get("event")
                        ts = datetime.now().strftime("%H:%M:%S")
                        if ev_type == "init":
                            print(
                                f"[{ts}] BAŞLANGIÇ: Model={ev.get('init', {}).get('model')}"
                            )
                        elif ev_type == "step_update":
                            su = ev.get("step_update", {})
                            s_idx = su.get("step_index", 0)
                            t_name = su.get("tool_name")
                            if t_name:
                                t_info = su.get("tool_info", {})
                                params = (
                                    t_info.get("parameters", {})
                                    if isinstance(t_info, dict)
                                    else {}
                                )
                                detail = (
                                    extract_file_path(params)
                                    or extract_command(params)
                                    or ""
                                )
                                print(
                                    f"[{ts}] ADIM {s_idx}: {t_name} {detail}".strip()
                                )
                            elif su.get("step_type"):
                                print(
                                    f"[{ts}] ADIM {s_idx}: {su.get('step_type')}"
                                )
                        elif ev_type == "result":
                            res = ev.get("result", {})
                            print(
                                f"[{ts}] SONUÇ: {res.get('status')} ({res.get('duration_seconds')}s)"
                            )
                    except Exception:
                        pass
                else:
                    meta = load_meta(task_dir)
                    if meta and meta.get("durum") != "calisiyor":
                        break
                    time.sleep(0.2)
        except KeyboardInterrupt:
            print(tr("watching_stopped"))


def cmd_iptal(args: argparse.Namespace) -> None:
    etiket = args.etiket
    state_root = get_state_root()
    task_dir = os.path.join(state_root, etiket)
    if not os.path.exists(task_dir):
        print(tr("err_task_not_found", etiket=etiket), file=sys.stderr)
        sys.exit(1)

    meta = load_meta(task_dir)
    if not meta:
        print(tr("err_meta_read", etiket=etiket), file=sys.stderr)
        sys.exit(1)

    pid = meta.get("pid")
    bekci_pid = meta.get("bekci_pid")

    # SIGTERM gönder (önce agy süreç grubu, sonra bekçi)
    if pid and is_pid_alive(pid):
        try:
            pgid = os.getpgid(pid)
            os.killpg(pgid, signal.SIGTERM)
        except Exception:
            try:
                os.kill(pid, signal.SIGTERM)
            except Exception:
                pass

    if bekci_pid and is_pid_alive(bekci_pid):
        try:
            os.kill(bekci_pid, signal.SIGTERM)
        except Exception:
            pass

    # 5 saniye boyunca sonlanmayı bekle
    start_t = time.time()
    while time.time() - start_t < 5.0:
        if not (pid and is_pid_alive(pid)):
            break
        time.sleep(0.2)
    else:
        # Hala canlıysa SIGKILL
        if pid and is_pid_alive(pid):
            try:
                pgid = os.getpgid(pid)
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                try:
                    os.kill(pid, signal.SIGKILL)
                except Exception:
                    pass
        if bekci_pid and is_pid_alive(bekci_pid):
            try:
                os.kill(bekci_pid, signal.SIGKILL)
            except Exception:
                pass

    meta["durum"] = "iptal"
    save_meta(task_dir, meta)
    print(tr("task_cancelled", etiket=etiket))


def cmd_temizle(args: argparse.Namespace) -> None:
    gun = args.gun
    state_root = get_state_root()
    if not os.path.exists(state_root):
        print(tr("no_tasks_to_clean"))
        return

    now = time.time()
    cutoff = now - (gun * 86400)
    cleaned = 0
    for entry in os.listdir(state_root):
        tdir = os.path.join(state_root, entry)
        if not os.path.isdir(tdir):
            continue
        meta = load_meta(tdir)
        if (
            meta
            and meta.get("durum") == "calisiyor"
            and gorev_canli(meta)
        ):
            continue

        try:
            mtime = os.path.getmtime(tdir)
            if mtime < cutoff:
                shutil.rmtree(tdir)
                print(tr("task_deleted", entry=entry))
                cleaned += 1
        except Exception:
            pass
    print(tr("tasks_cleaned", cleaned=cleaned))


def cmd_izin_kontrol(args: argparse.Namespace) -> None:
    settings_path = get_settings_path()
    yaz_list = [
        os.path.abspath(os.path.expanduser(p)) for p in (args.yaz or [])
    ]
    komut_list = [c.strip() for c in (args.komut or []) if c.strip()]

    missing = check_permissions(yaz_list, komut_list, settings_path)
    if missing:
        if getattr(args, "izin_ekle", False):
            bak = add_permissions(missing, settings_path)
            print(tr("permissions_added", bak=bak))
            for r in missing:
                print(f"  + {r}")
            sys.exit(0)
        else:
            print(
                tr("missing_permissions_detected"),
                file=sys.stderr,
            )
            for r in missing:
                print(f"  {r}", file=sys.stderr)
            sys.exit(2)
    else:
        print(tr("all_permissions_present"))
        sys.exit(0)


# ---------------------------------------------------------------------------
# Ana Giriş Noktası
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(
        description="ajan - Gemini (agy CLI) alt ajan yöneticisi", prog="ajan"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # 1. baslat
    p_baslat = subparsers.add_parser(
        "baslat", help="Yeni bir ajan görevi başlat"
    )
    p_baslat.add_argument("etiket", help="Görev etiketi")
    g_input = p_baslat.add_mutually_exclusive_group(required=True)
    g_input.add_argument(
        "-g", "--gorev-dosyalari", nargs="+", help="Görev metnini içeren dosyalar"
    )
    g_input.add_argument("-m", "--mesaj", help="Görev metni")
    p_baslat.add_argument(
        "--model",
        default="flash",
        help="Model (flash, argon, flash-orta, argon-orta veya tam kimlik)",
    )
    p_baslat.add_argument(
        "--effort",
        choices=["low", "medium", "high", "xhigh", "max"],
        help="Düşünme eforu",
    )
    p_baslat.add_argument(
        "--yaz",
        action="append",
        default=[],
        help="Yazma izni verilecek dizin (--add-dir eklenir)",
    )
    p_baslat.add_argument(
        "--oku",
        action="append",
        default=[],
        help="Okuma izni verilecek dizin (--add-dir eklenir)",
    )
    p_baslat.add_argument(
        "--komut",
        action="append",
        default=[],
        help="Çalıştırılabilecek komut öneki",
    )
    p_baslat.add_argument(
        "--sure", help="Maksimum süre (ör. 30m, 1h, 1800s)"
    )
    p_baslat.add_argument(
        "--sandbox", action="store_true", help="Sandbox modunu etkinleştir"
    )
    p_baslat.add_argument(
        "--izin-ekle",
        action="store_true",
        help="Eksik izinleri settings.json'a otomatik ekle",
    )
    p_baslat.add_argument(
        "--bekle", action="store_true", help="Görev bitene kadar bekle"
    )

    # 2. devam
    p_devam = subparsers.add_parser(
        "devam", help="Mevcut göreve yeni turla devam et"
    )
    p_devam.add_argument("etiket", help="Görev etiketi")
    p_devam.add_argument("-m", "--mesaj", required=True, help="Takip mesajı")
    p_devam.add_argument(
        "--bekle", action="store_true", help="Görev bitene kadar bekle"
    )

    # 3. durum
    p_durum = subparsers.add_parser(
        "durum", help="Görev durumlarını görüntüle"
    )
    p_durum.add_argument("etiket", nargs="?", help="Görev etiketi")
    p_durum.add_argument(
        "--json", action="store_true", help="JSON formatında çıktı ver"
    )

    # 4. bekle
    p_bekle = subparsers.add_parser(
        "bekle", help="Görevlerin bitmesini bekle"
    )
    p_bekle.add_argument(
        "etiketler", nargs="*", help="Beklenecek görev etiketleri"
    )
    p_bekle.add_argument(
        "--zaman-asimi", type=float, help="Zaman aşımı süresi (saniye)"
    )

    # 5. rapor
    p_rapor = subparsers.add_parser(
        "rapor", help="Görev raporunu veya ham yanıtı göster"
    )
    p_rapor.add_argument("etiket", help="Görev etiketi")
    p_rapor.add_argument(
        "--ham", action="store_true", help="Ham yanıtı (yanit.md) göster"
    )

    # 6. izle
    p_izle = subparsers.add_parser(
        "izle", help="Görev olay akışını canlı takip et"
    )
    p_izle.add_argument("etiket", help="Görev etiketi")

    # 7. iptal
    p_iptal = subparsers.add_parser("iptal", help="Çalışan bir görevi iptal et")
    p_iptal.add_argument("etiket", help="Görev etiketi")

    # 8. temizle
    p_temizle = subparsers.add_parser("temizle", help="Eski görevleri temizle")
    p_temizle.add_argument(
        "--gun",
        type=int,
        default=7,
        help="Kaç günden eski görevlerin temizleneceği (varsayılan: 7)",
    )

    # 9. izin
    p_izin = subparsers.add_parser("izin", help="İzin yönetimi")
    izin_sub = p_izin.add_subparsers(dest="izin_command", required=True)
    p_izin_kontrol = izin_sub.add_parser("kontrol", help="İzinleri denetle")
    p_izin_kontrol.add_argument(
        "--yaz", action="append", default=[], help="Yazma izni dizini"
    )
    p_izin_kontrol.add_argument(
        "--komut", action="append", default=[], help="Komut öneki"
    )
    p_izin_kontrol.add_argument(
        "--izin-ekle",
        action="store_true",
        help="Eksik izinleri otomatik ekle",
    )

    # 10. dogrula
    p_dogrula = subparsers.add_parser(
        "dogrula",
        help="Otomatik test ve düzeltme döngüsü ile görevi doğrula",
    )
    p_dogrula.add_argument("etiket", help="Görev etiketi")
    p_dogrula.add_argument(
        "--test", required=True, help="Çalıştırılacak doğrulama komutu"
    )
    p_dogrula.add_argument(
        "--tur",
        type=int,
        default=3,
        help="Maksimum deneme/düzeltme tur sayısı (varsayılan: 3)",
    )
    p_dogrula.add_argument(
        "--cwd", help="Komutun çalıştırılacağı dizin"
    )
    p_dogrula.add_argument(
        "--zaman-asimi",
        type=float,
        help="Komut zaman aşımı süresi (saniye)",
    )

    # Dahili bekçi
    p_bekci = subparsers.add_parser("_bekci", help=argparse.SUPPRESS)
    p_bekci.add_argument("etiket", help="Görev etiketi")

    args = parser.parse_args()

    if args.command == "baslat":
        cmd_baslat(args)
    elif args.command == "devam":
        cmd_devam(args)
    elif args.command == "dogrula":
        cmd_dogrula(args)
    elif args.command == "durum":
        cmd_durum(args)
    elif args.command == "bekle":
        cmd_bekle(args)
    elif args.command == "rapor":
        cmd_rapor(args)
    elif args.command == "izle":
        cmd_izle(args)
    elif args.command == "iptal":
        cmd_iptal(args)
    elif args.command == "temizle":
        cmd_temizle(args)
    elif args.command == "izin":
        if args.izin_command == "kontrol":
            cmd_izin_kontrol(args)
    elif args.command == "_bekci":
        run_bekci(args.etiket)


if __name__ == "__main__":
    main()
