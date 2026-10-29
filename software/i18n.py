"""
Interface language.

All user-visible text lives in lang/<code>.json (en, uk, ...). The language is
chosen by "language" in config.json. A missing string falls back to English.
To add a translation, copy lang/en.json to lang/<code>.json, translate the
values (keep the {placeholders}), and set "language": "<code>" in config.json.
"""
import json
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_EN = {}
_CUR = {}
lang = "en"


def _read(code):
    path = os.path.join(_HERE, "lang", f"{code}.json")
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def set_language(code):
    global _EN, _CUR, lang
    _EN = _read("en")
    lang = code or "en"
    _CUR = _EN if lang == "en" else _read(lang)


def init_from_config(path="config.json"):
    code = "en"
    try:
        with open(path, encoding="utf-8") as f:
            code = json.load(f).get("language", "en")
    except (OSError, ValueError):
        pass
    set_language(code)


def tr(key, **kw):
    if not _EN:
        set_language(lang)
    s = _CUR.get(key, _EN.get(key, key))
    return s.format(**kw) if kw and isinstance(s, str) else s


def available():
    d = os.path.join(_HERE, "lang")
    return sorted(f[:-5] for f in os.listdir(d) if f.endswith(".json"))
