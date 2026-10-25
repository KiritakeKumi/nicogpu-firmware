#!/usr/bin/env python3
"""Build and run the CP firmware host tests.

Firmware core sources are freestanding C99 (no libc, no platform headers);
only the host test harness (tests/) may use libc.  Two compilers are driven
when available so the core stays warning-clean under both MSVC and GCC
(-Wall -Wextra / /W4), matching the usual dual-compiler discipline:

  python tools/run_host_tests.py            # build + test, every compiler found
  python tools/run_host_tests.py --build-only
  python tools/run_host_tests.py --compiler msvc|gcc
  python tools/run_host_tests.py --cross    # syntax-only riscv probe (see --help)

Windows notes: MSVC is located via vswhere or the common BuildTools path
(vcvars64); GCC is used from
C:/msys64/mingw64/bin when present (add it to PATH if cc1 complains about
missing DLLs).  CI (ubuntu) is authoritative; this script is a local mirror.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]  # repo root
FW = ROOT

CORE_SOURCES = [
    FW / "src" / "ngfw_core.c",
    FW / "src" / "ngfw_pkt.c",
    FW / "src" / "ngfw_util.c",
    FW / "src" / "ngfw_plat_ibex.c",  # compiled on the host on purpose:
    # keeps the cross build warning-clean without a riscv toolchain here
    FW / "src" / "ngfw_ibex_main.c",  # entry point (boot + event loop)
]
TEST_SOURCES = [
    FW / "tests" / "ngfw_hostplat.c",
    FW / "tests" / "test_main.c",
]
INCLUDES = ["-I" + str(FW / "include"), "-I" + str(FW / "tests")]

# Freestanding core: warn about everything, allow nothing from libc.
GCC_CORE_FLAGS = ["-std=c99", "-Wall", "-Wextra", "-ffreestanding"]
GCC_TEST_FLAGS = ["-std=c99", "-Wall", "-Wextra"]

MSVC_CORE_FLAGS = ["/nologo", "/W4", "/c", "/D_CRT_SECURE_NO_WARNINGS"]
MSVC_TEST_FLAGS = ["/nologo", "/W4", "/D_CRT_SECURE_NO_WARNINGS"]


def run(cmd, cwd=None, env=None):
    print("+", " ".join(str(c) for c in cmd))
    return subprocess.run([str(c) for c in cmd], cwd=cwd, env=env).returncode


def run_quiet(cmd, env=None):
    # MSVC-built binaries sometimes fail CreateProcess when handed forward-
    # slash paths; pass Windows separators and swallow the compile chatter.
    args = [str(c).replace("/", "\\") if str(c).startswith("C:/") else str(c)
            for c in cmd]
    r = subprocess.run(args, capture_output=True, text=True, env=env)
    if r.stdout:
        print(r.stdout.rstrip())
    if r.stderr:
        print(r.stderr.rstrip())
    return r.returncode


def find_msvc():
    """Return (env, cl_path) for the newest MSVC, or None."""
    vswhere = Path(
        r"C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
    )
    candidates = []
    if vswhere.exists():
        try:
            out = subprocess.run(
                [str(vswhere), "-latest", "-products", "*", "-requires",
                 "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
                 "-property", "installationPath"],
                capture_output=True, text=True,
            ).stdout.strip()
            if out:
                candidates.append(Path(out))
        except OSError:
            pass
    for common in (r"C:\Program Files\Microsoft Visual Studio\2022",
                   r"C:\Program Files (x86)\Microsoft Visual Studio\2022"):
        p = Path(common)
        if p.exists():
            for edition in ("BuildTools", "Community", "Professional", "Enterprise"):
                if (p / edition).exists():
                    candidates.append(p / edition)
    for base in candidates:
        vcvars = base / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
        if not vcvars.exists():
            continue
        # Ask the environment for cl.exe via a throwaway cmd invocation.
        probe = (
            "@echo off\r\ncall \"%s\" >nul 2>&1\r\nwhere cl\r\n" % vcvars
        )
        fd, name = tempfile.mkstemp(suffix=".bat")
        os.close(fd)
        tmp = Path(name)
        try:
            tmp.write_bytes(probe.encode("ascii"))
            out = subprocess.run(
                ["cmd", "/c", str(tmp)], capture_output=True, text=True
            ).stdout.strip()
        finally:
            tmp.unlink(missing_ok=True)
        for line in out.splitlines():
            line = line.strip()
            if line.lower().endswith("cl.exe"):
                return vcvars, Path(line)
    return None


def msvc_env(vcvars):
    probe = (
        "@echo off\r\ncall \"%s\" >nul 2>&1\r\n"
        "echo LIB=%%LIB%%\r\necho INCLUDE=%%INCLUDE%%\r\n"
        "echo PATH=%%PATH%%\r\n" % vcvars
    )
    fd, name = tempfile.mkstemp(suffix=".bat")
    os.close(fd)
    tmp = Path(name)
    try:
        tmp.write_bytes(probe.encode("ascii"))
        out = subprocess.run(
            ["cmd", "/c", str(tmp)], capture_output=True, text=True
        ).stdout
    finally:
        tmp.unlink(missing_ok=True)
    env = dict(os.environ)
    for line in out.splitlines():
        if "=" in line and (line.startswith(("LIB=", "INCLUDE=", "PATH="))):
            k, v = line.split("=", 1)
            env[k] = v
    return env


def build_gcc(out_dir, compiler):
    out_dir.mkdir(parents=True, exist_ok=True)
    # msys2's cc1 needs its DLLs from the compiler's own bin directory,
    # which is not on PATH when gcc.exe is addressed by full path.
    bin_dir = str(Path(compiler).resolve().parent)
    env = dict(os.environ)
    env["PATH"] = bin_dir + os.pathsep + env.get("PATH", "")
    objs = []
    for src in CORE_SOURCES:
        obj = out_dir / (src.stem + ".o")
        cmd = [compiler] + GCC_CORE_FLAGS + INCLUDES + ["-c", src, "-o", obj]
        if run(cmd, env=env) != 0:
            return None
        objs.append(obj)
    for src in TEST_SOURCES:
        obj = out_dir / (src.stem + ".o")
        cmd = [compiler] + GCC_TEST_FLAGS + INCLUDES + ["-c", src, "-o", obj]
        if run(cmd, env=env) != 0:
            return None
        objs.append(obj)
    exe = out_dir / "ngfw_test_gcc.exe"
    if run([compiler] + objs + ["-o", exe], env=env) != 0:
        return None
    return exe


def build_msvc(out_dir, tool):
    vcvars, cl = tool
    env = msvc_env(vcvars)
    out_dir.mkdir(parents=True, exist_ok=True)
    objs = []
    for src in CORE_SOURCES:
        obj = out_dir / (src.stem + ".obj")
        cmd = [cl] + MSVC_CORE_FLAGS + INCLUDES + ["-Fo" + str(obj), str(src)]
        if run(cmd, env=env) != 0:
            return None
        objs.append(obj)
    for src in TEST_SOURCES:
        obj = out_dir / (src.stem + ".obj")
        cmd = [cl, "/nologo", "/W4", "/D_CRT_SECURE_NO_WARNINGS"] + INCLUDES + [
            "/c", "/Fo" + str(obj), str(src)]
        if run(cmd, env=env) != 0:
            return None
        objs.append(obj)
    exe = out_dir / "ngfw_test_msvc.exe"
    if run([cl] + MSVC_TEST_FLAGS + [str(o) for o in objs] +
           ["/link", "/OUT:" + str(exe), "/SUBSYSTEM:CONSOLE"], env=env) != 0:
        return None
    return exe


def riscv_probe(out_dir):
    """Syntax-only compile of the core with the CI cross compiler if present.

    The authoritative riscv64-unknown-elf build runs in CI (see
    .github/workflows/ci-riscv.yml).  Locally this just checks that the toolchain, when
    installed, still accepts the core with -Wall -Wextra.
    """
    for cc in ("riscv64-unknown-elf-gcc", "riscv32-unknown-elf-gcc"):
        if subprocess.run(["where", cc], capture_output=True).returncode == 0:
            out_dir.mkdir(parents=True, exist_ok=True)
            ok = True
            for src in CORE_SOURCES:
                obj = out_dir / (src.stem + ".rv.o")
                cmd = [cc, "-march=rv32imc_zicsr", "-mabi=ilp32", "-std=c99",
                       "-Wall", "-Wextra", "-ffreestanding"] + INCLUDES + [
                    "-c", src, "-o", obj]
                if run(cmd) != 0:
                    ok = False
                    break
            return 0 if ok else 1
    print("riscv cross compiler not found locally; skipping (CI covers it)")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--compiler", choices=["msvc", "gcc"], default=None)
    ap.add_argument("--build-only", action="store_true")
    ap.add_argument("--cross", action="store_true",
                    help="also syntax-check the core with a local riscv gcc")
    args = ap.parse_args()

    build_root = FW / "tests" / "build"
    failures = 0
    built = []

    if args.compiler in (None, "gcc"):
        gcc = "C:/msys64/mingw64/bin/gcc.exe"
        gcc = gcc if Path(gcc).exists() else "gcc"
        exe = build_gcc(build_root / "gcc", gcc)
        if exe is None:
            failures += 1
        else:
            built.append(("gcc", exe))

    if args.compiler in (None, "msvc"):
        tool = find_msvc()
        if tool is None:
            print("MSVC not found; skipping")
        else:
            exe = build_msvc(build_root / "msvc", tool)
            if exe is None:
                failures += 1
            else:
                built.append(("msvc", exe))

    if args.cross:
        failures += riscv_probe(build_root / "riscv")

    if not args.build_only:
        for name, exe in built:
            print("== running", name, "tests ==")
            if run_quiet([exe, FW / "tests" / "vectors" / "manifest.txt"]) != 0:
                failures += 1

    print("DONE", "FAILED" if failures else "OK")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
