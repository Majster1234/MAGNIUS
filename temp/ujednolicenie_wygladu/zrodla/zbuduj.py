#!/usr/bin/env python3
"""Składa samodzielną makietę C2 „Drzewo” (styl C „Sterownia”, gęsty, ekrany dopasowane do okna).

Użycie: python3 temp/ujednolicenie_wygladu/zrodla/zbuduj.py
"""
from pathlib import Path

SRC = Path(__file__).resolve().parent
OUT = SRC.parent
SHEETS = ["baza.css", "wariant_c_sterownia.css", "gesty.css", "wariant_c2_drzewo.css"]


def read(name: str) -> str:
    return (SRC / name).read_text(encoding="utf-8")


def main() -> None:
    diag = read("diagnostyka/c2_drzewo.html").replace("{{TOOLBAR}}", read("diagnostyka/pasek_przebiegow.html"))
    html = (read("szkielet_gesty.html").replace("{{DIAG}}", diag)
            .replace("{{CSS}}", "\n".join(read(s) for s in SHEETS)).replace("{{JS}}", read("wspolne_gesty.js"))
            .replace("{{TITLE}}", "Ujednolicenie wyglądu · C2").replace("{{ID}}", "c2_drzewo")
            .replace("{{VARIANT_NAME}}", "C2 — Drzewo (mapa systemu + zakładki, statusy w menu)"))
    assert "{{" not in html, "niewypełniony znacznik"
    target = OUT / "wariant_c2_drzewo.html"
    target.write_text(html, encoding="utf-8")
    print(f"{target.relative_to(OUT.parent.parent)}  {len(html) // 1024} KiB")


if __name__ == "__main__":
    main()
