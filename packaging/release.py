#!/usr/bin/env python3
"""Automate the MRPT 3.x release procedure described in
doc/source/make_a_mrpt_release.rst.

Steps performed, in order:
  1. Sanity checks: on `develop`, clean tree, up to date with remote.
  2. `catkin_prepare_release` to bump every module/app `package.xml`
     version, regenerate `CHANGELOG.rst` files, commit and tag (unsigned).
  3. Re-create the tag as GPG-signed.
  4. [CONFIRM] Push the `develop` commit and the signed tag.
  5. Merge `develop` into `master`.
  6. [CONFIRM] Push `master`.
  7. Run `packaging/make_release.sh` to build tar.gz/zip + .asc signature.
  8. Aggregate the new per-module CHANGELOG.rst entries into release notes.
  9. [CONFIRM] Create the GitHub release (`gh release create`) and upload
     the tarball, zip, and signature.

Any step marked [CONFIRM] is public and/or hard to reverse: the script
prints the exact command(s) and asks for an explicit "yes" before running
them. Everything else runs automatically. Use --dry-run to only print the
commands that would be run, without executing anything (confirmations are
skipped in dry-run mode).

This script intentionally has no fully-unattended mode for the confirmed
steps: merging to a public branch and publishing a GitHub release should
always have a human in the loop.
"""

from __future__ import annotations

import argparse
import glob
import re
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def run(cmd: list[str], dry_run: bool, cwd: Path = REPO_ROOT, capture: bool = False) -> str:
    print(f"$ {' '.join(cmd)}")
    if dry_run:
        return ""
    result = subprocess.run(
        cmd, cwd=cwd, check=True, text=True, capture_output=capture
    )
    return result.stdout if capture else ""


def confirm(question: str, dry_run: bool) -> None:
    if dry_run:
        print(f"[dry-run] would ask: {question}")
        return
    answer = input(f"{question} [type 'yes' to continue] ").strip().lower()
    if answer != "yes":
        print("Aborted by user.")
        sys.exit(1)


def check_clean_tree(dry_run: bool) -> None:
    status = run(["git", "status", "--porcelain"], dry_run, capture=True)
    if not dry_run and status.strip():
        print("Error: working tree is not clean. Commit or stash changes first.")
        sys.exit(1)


def check_branch(expected: str, dry_run: bool) -> None:
    branch = run(["git", "rev-parse", "--abbrev-ref", "HEAD"], dry_run, capture=True).strip()
    if not dry_run and branch != expected:
        print(f"Error: expected to be on branch '{expected}', currently on '{branch}'.")
        sys.exit(1)


def read_version() -> str:
    pkg_xml = REPO_ROOT / "modules" / "mrpt_common" / "package.xml"
    text = pkg_xml.read_text()
    m = re.search(r"<version>([0-9.]+)</version>", text)
    if not m:
        raise RuntimeError(f"Could not find <version> in {pkg_xml}")
    return m.group(1)


def aggregate_changelog_notes(version: str) -> str:
    """Collect the new dated section just created by catkin_prepare_release
    in every module/app CHANGELOG.rst into a single Markdown blob, grouped
    by module, skipping modules with no entries for this version."""
    sections: list[str] = []
    pattern = re.compile(
        rf"^{re.escape(version)} \([0-9-]+\)\n-+\n(.*?)(?=\n\S.*\n-+\n|\Z)",
        re.DOTALL | re.MULTILINE,
    )
    for changelog in sorted(glob.glob(str(REPO_ROOT / "modules" / "*" / "CHANGELOG.rst"))) + sorted(
        glob.glob(str(REPO_ROOT / "apps" / "*" / "CHANGELOG.rst"))
    ):
        path = Path(changelog)
        module_name = path.parent.name
        text = path.read_text()
        m = pattern.search(text)
        if not m:
            continue
        body = m.group(1).strip()
        if not body:
            continue
        sections.append(f"### {module_name}\n\n{body}\n")
    header = f"# Release {version}\n\n"
    if not sections:
        return header + "(No per-module changes recorded.)\n"
    return header + "\n".join(sections)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    bump_group = parser.add_mutually_exclusive_group()
    bump_group.add_argument("--bump", choices=["major", "minor", "patch"], default="patch")
    bump_group.add_argument("--version", help="Explicit version X.Y.Z instead of bumping")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without running them")
    parser.add_argument("--skip-debian-reminder", action="store_true", help="Suppress the final Debian/gbp reminder")
    args = parser.parse_args()

    dry_run = args.dry_run

    check_branch("develop", dry_run)
    check_clean_tree(dry_run)
    run(["git", "fetch", "origin"], dry_run)
    run(["git", "pull", "--ff-only", "origin", "develop"], dry_run)

    # 1. Bump versions + changelogs, commit, unsigned tag.
    prep_cmd = ["catkin_prepare_release", "--no-push", "-y"]
    if args.version:
        prep_cmd += ["--version", args.version]
    else:
        prep_cmd += ["--bump", args.bump]
    run(prep_cmd, dry_run)

    version = args.version or read_version()
    print(f"\n>>> New version: {version}\n")

    # 2. Re-sign the tag.
    run(["git", "tag", "-d", version], dry_run)
    run(["git", "tag", "--sign", version, "-m", f"MRPT {version}"], dry_run)

    print(f"\nReview the changes now in another terminal (git show, git log -p) before continuing.\n")

    # 3. Push develop + tag (CONFIRM).
    confirm(f"About to push 'develop' and tag '{version}' to origin. This is public. Continue?", dry_run)
    run(["git", "push", "origin", "develop"], dry_run)
    run(["git", "push", "origin", version], dry_run)

    # 4. Merge into master.
    run(["git", "checkout", "master"], dry_run)
    run(["git", "pull", "--ff-only", "origin", "master"], dry_run)
    run(["git", "merge", "--no-edit", "develop"], dry_run)

    confirm(f"About to push 'master' (merged with develop @ {version}) to origin. Continue?", dry_run)
    run(["git", "push", "origin", "master"], dry_run)

    # 5. Build packages.
    run(["bash", "packaging/make_release.sh"], dry_run)

    out_dir = Path.home() / "mrpt_release"
    tarball = out_dir / f"mrpt-{version}.tar.gz"
    sig = out_dir / f"mrpt-{version}.tar.gz.asc"
    zipfile = out_dir / f"mrpt-{version}.zip"

    if not dry_run:
        for f in (tarball, sig, zipfile):
            if not f.exists():
                print(f"Error: expected release artifact not found: {f}")
                sys.exit(1)

    # 6. Aggregate changelog notes and create GitHub release.
    notes = aggregate_changelog_notes(version)
    print("\n----- Proposed release notes -----")
    print(notes)
    print("-----------------------------------\n")

    confirm(f"About to create and publish GitHub release '{version}' with the notes above and attach "
            f"{tarball.name}, {sig.name}, {zipfile.name}. Continue?", dry_run)

    with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False) as fh:
        fh.write(notes)
        notes_path = fh.name

    run(
        [
            "gh", "release", "create", version,
            "--title", f"Release of v{version}",
            "--notes-file", notes_path,
            str(tarball), str(sig), str(zipfile),
        ],
        dry_run,
    )

    run(["git", "clean", "-fd"], dry_run)

    print(f"\nRelease {version} done.")
    if not args.skip_debian_reminder:
        print(
            "Remember to update the Debian/gbp packaging repo next "
            "(see step 5, 'Create a new Debian package', in "
            "doc/source/make_a_mrpt_release.rst)."
        )


if __name__ == "__main__":
    main()
