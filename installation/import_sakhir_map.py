#!/usr/bin/env python3
"""Importa únicamente el mapa Sakhir original del ZIP proporcionado por el curso.

No descarga ni redistribuye los mapas. Los SHA-256 fijados corresponden al par
que generó las evidencias finales y sus manifests. No usa ZipFile.extract().
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat
import sys
import tempfile
import zipfile


PROJECT = Path(__file__).resolve().parents[1]
EXPECTED_SHA256 = {
    "Sakhir_map.png": "f3746a4d48a72ccc3cad020bc383e370dd3a915009c87d4d6041af06e7de85eb",
    "Sakhir_map.yaml": "f24f04998becd72e4fa4c8dea7ff2177a9035a691d944804cd0e6278c201cf44",
}
MAX_MEMBER_BYTES = 2 * 1024 * 1024


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def archive_pair(path: Path) -> dict[str, bytes]:
    """Validate names, uniqueness, size and contents before writing anything."""
    selected: dict[str, zipfile.ZipInfo] = {}
    with zipfile.ZipFile(path) as archive:
        for entry in archive.infolist():
            name = entry.filename
            parts = PurePosixPath(name).parts
            if (not parts or name.startswith("/") or "\\" in name
                    or ".." in parts or ":" in parts[0] or "\x00" in name):
                raise ValueError(f"Nombre de archivo inseguro en el ZIP: {name!r}")
            if entry.is_dir() or parts[-1] not in EXPECTED_SHA256:
                continue
            base = parts[-1]
            if base in selected:
                raise ValueError(f"El ZIP contiene más de una entrada para {base}")
            mode = entry.external_attr >> 16
            if stat.S_ISLNK(mode):
                raise ValueError(f"No se admiten enlaces simbólicos: {name}")
            if entry.flag_bits & 1:
                raise ValueError(f"No se admiten mapas cifrados: {name}")
            if not 0 < entry.file_size <= MAX_MEMBER_BYTES:
                raise ValueError(f"Tamaño de mapa no permitido: {name}")
            selected[base] = entry
        missing = sorted(set(EXPECTED_SHA256) - set(selected))
        if missing:
            raise ValueError("Falta el mapa en el ZIP: " + ", ".join(missing))
        contents = {name: archive.read(entry) for name, entry in selected.items()}
    for name, data in contents.items():
        actual = sha256(data)
        if actual != EXPECTED_SHA256[name]:
            raise ValueError(f"SHA-256 incorrecto para {name}: {actual}; "
                             f"se esperaba {EXPECTED_SHA256[name]}")
    # Preserve the exact original YAML: it already uses a portable image name.
    image_lines = [line.strip() for line in contents["Sakhir_map.yaml"].decode("utf-8").splitlines()
                   if line.strip().startswith("image:")]
    if image_lines != ["image: Sakhir_map.png"]:
        raise ValueError("El YAML debe referenciar image: Sakhir_map.png")
    return contents


def import_pair(contents: dict[str, bytes], destination: Path) -> dict[str, str]:
    if destination.is_symlink():
        raise ValueError(f"El directorio de destino es un enlace simbólico: {destination}")
    for name, data in contents.items():
        path = destination / name
        if path.is_symlink():
            raise ValueError(f"No se reemplaza un enlace simbólico: {path}")
        if path.exists() and (not path.is_file() or path.read_bytes() != data):
            raise ValueError(f"El destino contiene un mapa diferente; no se sobrescribe: {path}")
    destination.mkdir(parents=True, exist_ok=True)
    status: dict[str, str] = {}
    created: list[Path] = []
    try:
        for name, data in contents.items():
            path = destination / name
            if path.exists():
                status[name] = "ya presente, idéntico"
                continue
            # A hard link publishes a fully written file atomically without
            # replacing any destination created concurrently.
            with tempfile.NamedTemporaryFile(dir=destination, prefix=".sakhir-", delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            try:
                os.link(temporary, path)
                created.append(path)
            finally:
                temporary.unlink(missing_ok=True)
            status[name] = "importado"
    except Exception:
        for path in created:
            path.unlink(missing_ok=True)
        raise
    return status


def write_report(path: Path, report: dict) -> None:
    """Exclusive creation also rejects existing files and symlinks."""
    with path.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
        stream.write("\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zip", required=True, type=Path, help="ZIP original Mapas-F1Tenth-v2.zip del curso")
    parser.add_argument("--destination", type=Path, default=PROJECT / "results", help="directorio de destino (por defecto: results/ del proyecto)")
    parser.add_argument("--output", type=Path, help="informe JSON nuevo; no reemplaza archivos existentes")
    args = parser.parse_args()
    try:
        if args.output is not None and (args.output.exists() or args.output.is_symlink()):
            raise ValueError(f"El informe ya existe; elija otra ruta: {args.output}")
        if args.output is not None and not args.output.parent.is_dir():
            raise ValueError(f"No existe el directorio del informe: {args.output.parent}")
        destination = args.destination.absolute()
        contents = archive_pair(args.zip)
        status = import_pair(contents, destination)
        report = {"status": "PASS", "zip": str(args.zip.resolve()),
                  "destination": str(destination), "files": {
                      name: {"sha256": EXPECTED_SHA256[name], "status": status[name]}
                      for name in sorted(contents)}}
        if args.output is not None:
            write_report(args.output, report)
        for name in sorted(status):
            print(f"PASS {name}: {status[name]}; SHA-256 {EXPECTED_SHA256[name]}")
        print(f"Mapa listo: {destination / 'Sakhir_map.yaml'}")
        return 0
    except (OSError, ValueError, zipfile.BadZipFile, RuntimeError, UnicodeError) as error:
        print(f"FAIL importación de Sakhir: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
