"""Validate release versions and package only the intended distribution files."""

import argparse
import hashlib
import os
from pathlib import Path
import re
import shutil
import tarfile
import tomllib
import zipfile

ROOT = Path(__file__).resolve().parents[2]
TARGETS = (
    "x86_64-pc-windows-msvc",
    "i686-pc-windows-msvc",
    "aarch64-pc-windows-msvc",
    "x86_64-unknown-linux-musl",
    "i686-unknown-linux-musl",
    "aarch64-unknown-linux-musl",
    "armv7-unknown-linux-musleabihf",
    "x86_64-apple-darwin",
    "aarch64-apple-darwin",
)


def validate_tag(version, tag):
    number = r"(?:0|[1-9][0-9]*)"
    identifier = r"[0-9A-Za-z-]+"
    pattern = rf"{number}\.{number}\.{number}(?:-{identifier}(?:\.{identifier})*)?(?:\+{identifier}(?:\.{identifier})*)?"
    if not re.fullmatch(pattern, version):
        raise ValueError(f"Invalid SemVer: {version}")
    prerelease = version.split("+", 1)[0].partition("-")[2]
    if any(part.isdigit() and len(part) > 1 and part.startswith("0") for part in prerelease.split(".")):
        raise ValueError(f"Invalid prerelease: {version}")
    if tag and tag not in (version, f"v{version}"):
        raise ValueError(f"Tag {tag!r} must match Cargo version {version!r} (optional v prefix)")
    return bool(prerelease)


def version_info(root=ROOT):
    with (root / "Cargo.toml").open("rb") as file:
        manifest = tomllib.load(file)
    version = manifest["workspace"]["package"]["version"]
    validate_tag(version, "")
    # A tagged release must also build successfully with --locked.
    with (root / "Cargo.lock").open("rb") as file:
        locked = {p["name"]: p["version"] for p in tomllib.load(file)["package"] if "source" not in p}
    for member in manifest["workspace"]["members"]:
        with (root / member / "Cargo.toml").open("rb") as file:
            package = tomllib.load(file)["package"]
        if package["version"] != {"workspace": True} or locked.get(package["name"]) != version:
            raise ValueError(f"{member}: workspace / lockfile version must be {version}")
    return version


def archive_name(version, target):
    if target not in TARGETS:
        raise ValueError(f"Unsupported release target: {target}")
    suffix = ".zip" if "windows" in target else ".tar.gz"
    return f"dth10-{version}-{target}{suffix}"


def package(root, target, output):
    version = version_info(root)
    name = archive_name(version, target)
    windows = "windows" in target
    binary_dir = root / "target" / target / "release"
    extension = ".exe" if windows else ""
    files = [(binary_dir / (name + extension), name + extension) for name in ("dth10", "dth10-diag")]
    files += [(root / name, name) for name in ("README.md", "LICENSE")]
    if windows:
        files += [
            (binary_dir / "BonDriver_DTH10.dll", "BonDriver_DTH10.dll"),
            (root / "crates/dth10-bondriver/BonDriver_DTH10.ini", "BonDriver_DTH10.ini"),
        ]
    for source, _ in files:
        if not source.is_file():
            raise FileNotFoundError(source)
    output.mkdir(parents=True, exist_ok=True)
    archive = output / name
    folder = name.removesuffix(".zip").removesuffix(".tar.gz")
    if windows:
        with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as bundle:
            for source, destination in files:
                bundle.write(source, f"{folder}/{destination}")
    else:
        with tarfile.open(archive, "w:gz") as bundle:
            for source, destination in files:
                info = bundle.gettarinfo(str(source), arcname=f"{folder}/{destination}")
                info.mode = 0o755 if destination in ("dth10", "dth10-diag") else 0o644
                info.uid = info.gid = 0
                info.uname = info.gname = ""
                with source.open("rb") as file:
                    bundle.addfile(info, file)
    return archive


def collect(source, output, version):
    # Require every target before publishing; avoid silently incomplete releases.
    archives = []
    for target in TARGETS:
        name = archive_name(version, target)
        matches = list(source.rglob(name))
        if len(matches) != 1 or not matches[0].is_file():
            raise ValueError(f"Expected exactly one artifact: {name}")
        archives.append(matches[0])
    output.mkdir(parents=True, exist_ok=True)
    checksums = []
    for archive in sorted(archives):
        destination = output / archive.name
        shutil.copyfile(archive, destination)
        with destination.open("rb") as file:
            digest = hashlib.file_digest(file, "sha256").hexdigest()
        checksums.append(f"{digest}  {destination.name}\n")
    (output / "SHA256SUMS").write_text("".join(checksums), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("version")
    pack = commands.add_parser("package")
    pack.add_argument("--target", required=True, choices=TARGETS)
    pack.add_argument("--output", type=Path, default=Path("dist"))
    gather = commands.add_parser("collect")
    gather.add_argument("--source", type=Path, default=Path("artifacts"))
    gather.add_argument("--output", type=Path, default=Path("dist"))
    args = parser.parse_args()
    if args.command == "version":
        version = version_info()
        prerelease = validate_tag(version, os.environ.get("RELEASE_TAG", ""))
        result = f"version={version}\nprerelease={str(prerelease).lower()}\n"
        print(result, end="")
        if os.environ.get("GITHUB_OUTPUT"):
            with open(os.environ["GITHUB_OUTPUT"], "a", encoding="utf-8") as file:
                file.write(result)
    elif args.command == "package":
        print(package(ROOT, args.target, args.output))
    else:
        collect(args.source, args.output, version_info())


if __name__ == "__main__":
    main()
