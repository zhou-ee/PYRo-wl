"""Compile old/new attitude code with identical Cortex-M7 flags and dump ARM assembly.

This is a static audit, not a proof of numerical equivalence or a cycle benchmark.
The reference is the frozen migration baseline (its inverse sqrt uses safe memcpy).
Usage: python audit_imu_assembly.py <compile_commands.json> <output-directory>
"""
import collections
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys


def run(arguments):
    result = subprocess.run(arguments, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f"{Path(arguments[0]).name} failed:\n{result.stderr}")
    return result.stdout


def functions(disassembly):
    result = {}
    current = None
    for line in disassembly.splitlines():
        header = re.match(r"^[0-9a-f]+ <(.+)>:$", line)
        if header:
            current = {"instructions": [], "calls": []}
            result[header[1]] = current
        elif current is not None:
            instruction = re.match(r"\s*[0-9a-f]+:\s+(?:[0-9a-f]{4,8}\s+)+\s*([a-z][\w.]*)\s*(.*)", line)
            if instruction:
                current["instructions"].append([instruction[1], instruction[2]])
            relocation = re.search(r"R_ARM_(?:THM_CALL|THM_JUMP24)\s+(.+)", line)
            if relocation:
                current["calls"].append(relocation[1])
    for value in result.values():
        value["instruction_count"] = len(value["instructions"])
        value["float_opcodes"] = dict(collections.Counter(
            op for op, _ in value["instructions"]
            if op.startswith(("vadd", "vsub", "vmul", "vdiv", "vfma", "vfms", "vmla", "vmls", "vnm", "vcmp", "vsqrt"))))
    return result


def main():
    database, output_arg = sys.argv[1:]
    repo = Path(__file__).resolve().parents[2]
    output = Path(output_arg).resolve()
    output.mkdir(parents=True, exist_ok=True)
    entry = next(item for item in json.loads(Path(database).read_text())
                 if item["file"].endswith("/pyro_ins_attitude.cpp"))
    tokens = [word.strip('"') for word in shlex.split(entry["command"], posix=False)]
    compiler = Path(tokens[0])
    objdump = compiler.with_name("arm-none-eabi-objdump.exe")
    size = compiler.with_name("arm-none-eabi-size.exe")
    flags = []
    index = 1
    while index < len(tokens):
        word = tokens[index]
        if word in ("-o", "-c"):
            index += 2
            continue
        if not re.fullmatch(r"-O[0-3sg]|-g\d*", word) and word != "-fdiagnostics-color=always":
            flags.append(word)
        index += 1
    sources = {
        "current_attitude": repo / "PYRo/Component/INS/pyro_ins_attitude.cpp",
        "current_matrix": repo / "PYRo/Algorithm/Matrix/pyro_algo_matrix.cpp",
        "legacy_attitude": repo / "tests/forward_kf/reference/quaternion_ekf_reference.cpp",
        "legacy_kf": repo / "PYRo/Algorithm/KF/kalman_filter.cpp",
    }
    report = {"compiler": run([str(compiler), "--version"]).splitlines()[0],
              "flags": flags, "profiles": {},
              "limitations": ["Unlinked object files; size includes unreachable functions and both solver paths.",
                              "Instruction counts are static, not CPU cycle counts.",
                              "No ARM execution or formal equivalence proof is performed."]}
    for optimization in ("O0", "O3"):
        profile = {}
        for label, source in sources.items():
            obj = output / f"{label}-{optimization}.o"
            command = [str(compiler), *flags, f"-{optimization}", "-c", str(source), "-o", str(obj)]
            run(command)
            dump = run([str(objdump), "-drC", str(obj)])
            (output / f"{label}-{optimization}.asm.txt").write_text(dump, encoding="utf-8")
            profile[label] = {"command": command,
                              "size": run([str(size), str(obj)]).strip(),
                              "functions": functions(dump)}
        report["profiles"][optimization] = profile
    current = next(value for name, value in report["profiles"]["O0"]["current_attitude"]["functions"].items()
                   if "inverse_sqrt" in name)
    legacy = next(value for name, value in report["profiles"]["O0"]["legacy_attitude"]["functions"].items()
                  if "invSqrt" in name)
    report["comparisons"] = {
        "O0_inverse_sqrt_full_instruction_equal": current["instructions"] == legacy["instructions"],
        "O0_inverse_sqrt_float_arithmetic_equal":
            [item for item in current["instructions"] if item[0] in current["float_opcodes"]] ==
            [item for item in legacy["instructions"] if item[0] in legacy["float_opcodes"]],
        "unchecked_core_numeric_scan_calls": [
            {"optimization": optimization, "function": name, "call": call}
            for optimization, profile in report["profiles"].items()
            for name, value in profile["current_attitude"]["functions"].items()
            if "pyro::kf_t<6u, 3u, false>" in name
            for call in value["calls"] if "is_finite" in call or "_covariance_valid" in call],
    }
    (output / "audit.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"ARM assembly and audit metadata: {output}")
    for optimization, profile in report["profiles"].items():
        print(optimization)
        for label, item in profile.items():
            print(label, item["size"].splitlines()[-1])


if __name__ == "__main__":
    main()
