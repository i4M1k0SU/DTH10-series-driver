import hashlib
from pathlib import Path
import tarfile
import tempfile
import unittest
import zipfile

import release


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "Cargo.toml").write_text(
            '[workspace]\nmembers = ["crates/example"]\n[workspace.package]\nversion = "1.2.3"\n',
            encoding="utf-8",
        )
        (self.root / "Cargo.lock").write_text(
            '[[package]]\nname = "example"\nversion = "1.2.3"\n', encoding="utf-8"
        )
        member = self.root / "crates/example"
        member.mkdir(parents=True)
        (member / "Cargo.toml").write_text(
            '[package]\nname = "example"\nversion.workspace = true\n', encoding="utf-8"
        )
        for name in ("README.md", "LICENSE"):
            (self.root / name).write_text(name, encoding="utf-8")
        ini = self.root / "crates/dth10-bondriver/BonDriver_DTH10.ini"
        ini.parent.mkdir(parents=True)
        ini.write_text("[Device]\n", encoding="utf-8")

    def binaries(self, target):
        directory = self.root / "target" / target / "release"
        directory.mkdir(parents=True)
        names = ("dth10", "dth10-diag", "dth30-diag")
        if "windows" in target:
            names = ("dth10.exe", "dth10-diag.exe", "dth30-diag.exe", "BonDriver_DTH10.dll")
        for name in (*names, "CoBDACom.dll", "unwanted.pdb"):
            (directory / name).write_bytes(b"binary")
        return directory

    def test_tag_matches_workspace_version(self):
        for tag in ("v1.2.3", "1.2.3", ""):
            self.assertFalse(release.validate_tag("1.2.3", tag))
        self.assertTrue(release.validate_tag("1.2.3-rc.1", "v1.2.3-rc.1"))
        self.assertFalse(release.validate_tag("1.2.3+build.4", "v1.2.3+build.4"))
        for version, tag in (("1.2.3", "v1.2.4"), ("01.2.3", ""), ("1.2.3-01", "")):
            with self.subTest(version=version, tag=tag), self.assertRaises(ValueError):
                release.validate_tag(version, tag)

    def test_lockfile_must_match_workspace_version(self):
        self.assertEqual(release.version_info(self.root), "1.2.3")
        (self.root / "Cargo.lock").write_text(
            '[[package]]\nname = "example"\nversion = "1.2.2"\n', encoding="utf-8"
        )
        with self.assertRaises(ValueError):
            release.version_info(self.root)

    def test_windows_zip_contains_only_distribution_files(self):
        target = "aarch64-pc-windows-msvc"
        self.binaries(target)
        archive = release.package(self.root, target, self.root / "dist")
        with zipfile.ZipFile(archive) as bundle:
            names = {Path(name).name for name in bundle.namelist()}
        self.assertEqual(
            names,
            {
                "dth10.exe",
                "dth10-diag.exe",
                "dth30-diag.exe",
                "BonDriver_DTH10.dll",
                "BonDriver_DTH10.ini",
                "README.md",
                "LICENSE",
            },
        )

    def test_unix_tar_preserves_executable_modes(self):
        for target in ("i686-unknown-linux-musl", "x86_64-apple-darwin"):
            self.binaries(target)
            archive = release.package(self.root, target, self.root / "dist")
            with tarfile.open(archive) as bundle:
                modes = {Path(info.name).name: info.mode for info in bundle.getmembers()}
            self.assertEqual(
                modes,
                {"dth10": 0o755, "dth10-diag": 0o755, "dth30-diag": 0o755, "README.md": 0o644, "LICENSE": 0o644},
            )

    def test_missing_binary_prevents_archive(self):
        target = "x86_64-pc-windows-msvc"
        directory = self.binaries(target)
        (directory / "BonDriver_DTH10.dll").unlink()
        with self.assertRaises(FileNotFoundError):
            release.package(self.root, target, self.root / "dist")
        self.assertFalse((self.root / "dist").exists())

    def test_collect_requires_all_targets_and_generates_checksums(self):
        source = self.root / "artifacts"
        source.mkdir()
        for target in release.TARGETS:
            (source / release.archive_name("1.2.3", target)).write_bytes(target.encode())
        output = self.root / "dist"
        release.collect(source, output, "1.2.3")
        lines = (output / "SHA256SUMS").read_text(encoding="utf-8").splitlines()
        self.assertEqual(len(lines), len(release.TARGETS))
        for line in lines:
            digest, name = line.split("  ")
            self.assertEqual(digest, hashlib.sha256((output / name).read_bytes()).hexdigest())
        (source / release.archive_name("1.2.3", release.TARGETS[0])).unlink()
        with self.assertRaises(ValueError):
            release.collect(source, output, "1.2.3")

    def test_collect_rejects_duplicate_artifacts(self):
        source = self.root / "artifacts"
        source.mkdir()
        duplicate = source / "duplicate"
        duplicate.mkdir()
        name = release.archive_name("1.2.3", release.TARGETS[0])
        (source / name).write_bytes(b"first")
        (duplicate / name).write_bytes(b"second")
        with self.assertRaises(ValueError):
            release.collect(source, self.root / "dist", "1.2.3")


if __name__ == "__main__":
    unittest.main()
