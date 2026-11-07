"""Fusion's current UI language and FitMe's UTF-8 string catalog."""

from __future__ import annotations

import json
from pathlib import Path

import adsk.core as core


_CATALOG = None
_LANGUAGES = {
    "GermanLanguage": "de",
    "EnglishLanguage": "en",
    "SpanishLanguage": "es",
    "FrenchLanguage": "fr",
    "ItalianLanguage": "it",
    "JapaneseLanguage": "ja",
    "KoreanLanguage": "ko",
    "PolishLanguage": "pl",
    "PortugueseBrazilianLanguage": "pt-BR",
    "ChinesePRCLanguage": "zh-CN",
    "TurkishLanguage": "tr",
}


def current_language() -> str:
    """Read Fusion's preference, falling back for unknown API language values."""
    try:
        selected = core.Application.get().preferences.generalPreferences.userLanguage
        for enum_name, code in _LANGUAGES.items():
            if selected == getattr(core.UserLanguages, enum_name):
                return code
    except (AttributeError, RuntimeError):
        pass
    return "en"


def catalog() -> dict:
    global _CATALOG
    if _CATALOG is None:
        with (Path(__file__).parent / "messages.json").open(encoding="utf-8") as source:
            _CATALOG = json.load(source)
    return _CATALOG


def tr(key: str, language: str | None = None, **values) -> str:
    """Translate by stable key; format only after selecting a complete string."""
    messages = catalog()
    language = language or current_language()
    template = messages.get(language, {}).get(key) or messages["en"][key]
    return template.format(**values) if values else template
