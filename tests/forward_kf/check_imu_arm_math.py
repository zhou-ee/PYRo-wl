"""Execute the compiled Cortex-M7 old/new algorithms; compare matrix dataflow.

Requires unicorn and pyelftools (can live in <output>/python). Does not measure
physical STM32 timing or constitute a formal proof for every possible input.
Usage: python check_imu_arm_math.py <compile_commands.json> <output-directory> [O0 O3 O3-strict]
"""
import importlib.util
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
sys.dont_write_bytecode = True


def build(database, output, profile):
    audit_path = Path(__file__).with_name("audit_imu_assembly.py")
    spec = importlib.util.spec_from_file_location("static_audit", audit_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    import shlex
    repo = Path(__file__).resolve().parents[2]
    entry = next(e for e in json.loads(Path(database).read_text()) if e["file"].endswith("/pyro_ins_attitude.cpp"))
    tokens = [s.strip('"') for s in shlex.split(entry["command"], posix=False)]
    compiler = Path(tokens[0])
    flags = []
    i = 1
    while i < len(tokens):
        token = tokens[i]
        if token in ("-c", "-o"):
            i += 2
            continue
        if not token.startswith(("-O", "-g", "-fdiagnostics-color")):
            flags.append(token)
        i += 1
    optimization = "-O0" if profile == "O0" else "-O3"
    flags += [optimization, "-I" + str(repo / "third_party/CMSIS-DSP/PrivateInclude")]
    if profile == "O3-strict":
        flags = [f for f in flags if f != "-funsafe-math-optimizations"]
        flags += ["-fno-unsafe-math-optimizations", "-ffp-contract=off", "-fno-strict-aliasing"]
    sources = [repo / "tests/forward_kf/imu_arm_audit_harness.cpp",
               repo / "PYRo/Component/INS/pyro_ins_attitude.cpp",
               repo / "PYRo/Algorithm/Matrix/pyro_algo_matrix.cpp",
               repo / "tests/forward_kf/reference/quaternion_ekf_reference.cpp",
               repo / "tests/forward_kf/reference/kalman_filter.cpp"]
    dsp = repo / "third_party/CMSIS-DSP/Source/MatrixFunctions"
    sources += [dsp / (name + ".c") for name in (
        "arm_mat_init_f32", "arm_mat_add_f32", "arm_mat_sub_f32", "arm_mat_mult_f32",
        "arm_mat_trans_f32", "arm_mat_inverse_f32", "arm_mat_cholesky_f32",
        "arm_mat_solve_lower_triangular_f32", "arm_mat_solve_upper_triangular_f32")]
    objects = []
    for source in sources:
        obj = output / (source.stem + "-exec-" + profile + ".o")
        if source.suffix == ".c":
            cflags = [f for f in flags if f not in ("-fno-rtti", "-fno-exceptions", "-fno-threadsafe-statics", "-std=gnu++17")]
            command = [str(compiler.with_name("arm-none-eabi-gcc.exe")), *cflags, "-std=gnu17"]
        else:
            command = [str(compiler), *flags]
        module.run([*command, "-c", str(source), "-o", str(obj)])
        objects.append(str(obj))
    elf = output / ("imu-math-" + profile + ".elf")
    module.run([str(compiler), "-mcpu=cortex-m7", "-mfpu=fpv5-d16", "-mfloat-abi=hard", "-mthumb",
                "-nostartfiles", "--specs=nosys.specs", *objects, "-lm",
                "-Wl,-Ttext=0x10000,-Tdata=0x20000000,-e,audit_init,--wrap=malloc,--gc-sections",
                "-Wl,-u,audit_tick_old,-u,audit_tick_new,-u,audit_enable_fpu,-u,audit_seed_gate",
                "-o", str(elf)])
    return elf


def emulate(elf_path, profile):
    from elftools.elf.elffile import ELFFile
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
    from unicorn.arm_const import (UC_CPU_ARM_CORTEX_M7, UC_ARM_REG_SP, UC_ARM_REG_LR,
                                   UC_ARM_REG_PC, UC_ARM_REG_FPEXC, UC_ARM_REG_R0,
                                   UC_ARM_REG_R1, UC_ARM_REG_R2)
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
    uc.mem_map(0x10000, 0x100000)
    uc.mem_map(0x20000000, 0x200000)
    uc.mem_map(0xE0000000, 0x100000)
    symbols = {}
    with elf_path.open("rb") as file:
        elf = ELFFile(file)
        for segment in elf.iter_segments():
            if segment["p_type"] == "PT_LOAD":
                uc.mem_write(segment["p_vaddr"], segment.data())
        for symbol in elf.get_section_by_name(".symtab").iter_symbols():
            if symbol["st_value"]:
                symbols[symbol.name] = symbol["st_value"]
    uc.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
    stop = 0x100000
    def call(name):
        uc.reg_write(UC_ARM_REG_SP, 0x201F0000)
        uc.reg_write(UC_ARM_REG_LR, stop | 1)
        uc.emu_start(symbols[name] | 1, stop, count=10000000)
        if uc.reg_read(UC_ARM_REG_PC) != stop:
            raise RuntimeError("Instruction limit reached in " + name)
    call("audit_enable_fpu")
    trace = []
    active = False
    def descriptor(address):
        rows, cols, pointer = struct.unpack("<HHI", uc.mem_read(address, 8))
        return (rows, cols), pointer
    def matrix_hook(name):
        def hook(cpu, address, size, data):
            if not active:
                return
            a_shape, a_ptr = descriptor(cpu.reg_read(UC_ARM_REG_R0))
            binary = name in ("mult", "add", "sub")
            operands = [bytes(cpu.mem_read(a_ptr, 4 * a_shape[0] * a_shape[1]))]
            shapes = [a_shape]
            if binary:
                b_shape, b_ptr = descriptor(cpu.reg_read(UC_ARM_REG_R1))
                shapes.append(b_shape)
                operands.append(bytes(cpu.mem_read(b_ptr, 4 * b_shape[0] * b_shape[1])))
            out_shape, out_ptr = descriptor(cpu.reg_read(UC_ARM_REG_R2 if binary else UC_ARM_REG_R1))
            entry = {"op": name, "shapes": shapes, "inputs": operands}
            trace.append(entry)
            return_pc = cpu.reg_read(UC_ARM_REG_LR) & ~1
            holder = []
            def returned(cpu, address, size, data):
                entry["output"] = bytes(cpu.mem_read(out_ptr, out_shape[0] * out_shape[1] * 4))
                entry["status"] = cpu.reg_read(UC_ARM_REG_R0)
                cpu.hook_del(holder[0])
                if entry["status"] != 0:
                    raise RuntimeError("CMSIS matrix failure during audit: " + name)
            holder.append(cpu.hook_add(UC_HOOK_CODE, returned, begin=return_pc, end=return_pc))
        return hook
    for name in ("mult", "add", "sub", "trans", "inverse"):
        address = symbols["arm_mat_" + name + "_f32"] & ~1
        uc.hook_add(UC_HOOK_CODE, matrix_hook(name), begin=address, end=address)
    reports = []
    cases = [("normal", 1.0, 512), ("fading", 0.9996, 512),
             ("reject", 1.0, 1), ("recover", 1.0, 1), ("adaptive", 1.0, 1)]
    for scenario, fading, samples in cases:
        uc.mem_write(symbols["audit_config"], struct.pack("<5f", 10, 0.001, 1000000, fading, 0.01))
        call("audit_init")
        threshold = 0.0 if scenario in ("reject", "recover") else 1e-8
        uc.mem_write(symbols["audit_threshold"], struct.pack("<f", threshold))
        uc.mem_write(symbols["audit_gate_state"], struct.pack("<2I", 50 if scenario == "recover" else 0,
                                                            int(scenario in ("reject", "recover"))))
        call("audit_seed_gate")
        compared_calls = 0
        mismatches = []
        max_error = 0.0
        first_matrix_mismatch = None
        first_field_mismatch = None
        rejections = [0, 0]
        extra_gain_calls = 0
        max_field_errors = [0.0] * 73
        structure_mismatches = 0
        branch_mismatches = 0
        for sample in range(samples):
            t = sample * 0.001
            theta, phi = 0.35 * math.sin(1.3*t), 0.15 * math.sin(0.7*t)
            td, pd, yd = 0.455 * math.cos(1.3*t), 0.105 * math.cos(0.7*t), 0.2 * math.cos(0.3*t)
            values = [pd-yd*math.sin(theta)+0.002,
                      td*math.cos(phi)+yd*math.cos(theta)*math.sin(phi)-0.001,
                      -td*math.sin(phi)+yd*math.cos(theta)*math.cos(phi),
                      -9.81*math.sin(theta)+0.15*math.sin(4*t),
                      9.81*math.cos(theta)*math.sin(phi), 9.81*math.cos(theta)*math.cos(phi),
                      (0.0011 if sample%2 else 0.0009) if scenario == "fading" else 0.001]
            if scenario == "fading" and sample%80 < 12:
                values[3] += 6.0
            if scenario in ("reject", "recover"):
                values[:6] = [0.001, 0.001, 0.001, 0.1, 0.0, 9.81]
            uc.mem_write(symbols["audit_input"], struct.pack("<7f", *values))
            trace = []
            active = True
            call("audit_tick_old")
            active = False
            old_trace = trace
            old = bytes(uc.mem_read(symbols["audit_snapshot"], 73*4))
            old_flags = bytes(uc.mem_read(symbols["audit_flags"], 16))
            old_aux = list(struct.unpack("<12f", uc.mem_read(symbols["audit_aux"], 48)))
            if scenario == "adaptive":
                nis = struct.unpack_from("<f", old, 52*4)[0]
                threshold = nis * 2
                # Reinitialize both before probing the identical first sample.
                call("audit_init")
                uc.mem_write(symbols["audit_threshold"], struct.pack("<f", threshold))
                uc.mem_write(symbols["audit_gate_state"], struct.pack("<2I", 0, 1))
                call("audit_seed_gate")
                trace = []
                active = True
                call("audit_tick_old")
                active = False
                old_trace = trace
                old = bytes(uc.mem_read(symbols["audit_snapshot"], 73*4))
                old_flags = bytes(uc.mem_read(symbols["audit_flags"], 16))
                old_aux = list(struct.unpack("<12f", uc.mem_read(symbols["audit_aux"], 48)))
            trace = []
            active = True
            call("audit_tick_new")
            active = False
            new = bytes(uc.mem_read(symbols["audit_snapshot"], 73*4))
            new_flags = bytes(uc.mem_read(symbols["audit_flags"], 16))
            new_aux = list(struct.unpack("<12f", uc.mem_read(symbols["audit_aux"], 48)))
            if sample == 0:
                def readable(entries):
                    return [{"op": e["op"], "shapes": e["shapes"],
                             "inputs": [list(struct.unpack("<" + "f" * (len(v)//4), v)) for v in e["inputs"]],
                             "output": list(struct.unpack("<" + "f" * (len(e["output"])//4), e["output"])),
                             "status": e["status"]}
                            for e in entries]
                first_step = {"input": values, "old_aux": old_aux, "new_aux": new_aux,
                              "old": readable(old_trace), "new": readable(trace)}
                (elf_path.parent / f"first-step-{profile}-{scenario}.json").write_text(
                    json.dumps(first_step, indent=2), encoding="utf-8")
            of, nf = struct.unpack("<4I", old_flags), struct.unpack("<4I", new_flags)
            for index, flags in enumerate((of, nf)):
                rejections[index] += bool(flags[3])
            # Old y=z-h(x) uses CMSIS subtraction/transpose, new scalar subtraction
            # and a row view. These structural operations are checked via NIS inputs.
            filtered = [item for item in old_trace if not (
                item["op"] == "sub" and item["shapes"][0] == (3, 1) or
                item["op"] == "trans" and item["shapes"][0] == (3, 1))]
            if of[3] and nf[3]:
                extra_gain_calls += 2
                trace = trace[:-2] # New computes K before rejection; no state commit.
            matrix_equal = len(filtered) == len(trace)
            expected = [("mult", [(6, 6), (6, 1)]), ("trans", [(6, 6)]),
                        ("mult", [(6, 6), (6, 6)]), ("mult", [(6, 6), (6, 6)]),
                        ("add", [(6, 6), (6, 6)]), ("trans", [(3, 6)]),
                        ("mult", [(3, 6), (6, 6)]), ("mult", [(3, 6), (6, 3)]),
                        ("add", [(3, 3), (3, 3)]), ("inverse", [(3, 3)]),
                        ("mult", [(3, 3), (3, 1)]), ("mult", [(1, 3), (3, 1)]),
                        ("mult", [(6, 6), (6, 3)]), ("mult", [(6, 3), (3, 3)]),
                        ("mult", [(6, 3), (3, 1)]), ("add", [(6, 1), (6, 1)]),
                        ("mult", [(6, 3), (3, 6)]), ("mult", [(6, 6), (6, 6)]),
                        ("sub", [(6, 6), (6, 6)])]
            expected = expected[:12] if of[3] and nf[3] else expected
            if any([(e["op"], e["shapes"]) for e in sequence] != expected for sequence in (filtered, trace)):
                structure_mismatches += 1
            if of != nf:
                branch_mismatches += 1
            for i, (a, b) in enumerate(zip(filtered, trace)):
                compared_calls += 1
                equal = a == b
                matrix_equal &= equal
                if not equal and first_matrix_mismatch is None:
                    first_matrix_mismatch = {"sample": sample, "call": i, "old": [a['op'], a['shapes']],
                                             "new": [b['op'], b['shapes']]}
            # Stale old K differs from freshly prepared new K on rejection; inspect
            # x/P/NIS/policy state, and compare K only for accepted corrections.
            field_indices = list(range(53)) + list(range(71, 73))
            if not of[3] and not nf[3]:
                field_indices += list(range(53, 71))
            values_old, values_new = struct.unpack("<73f", old), struct.unpack("<73f", new)
            fields_equal = True
            for i in field_indices:
                if old[4*i:4*i+4] != new[4*i:4*i+4]:
                    fields_equal = False
                    if first_field_mismatch is None:
                        first_field_mismatch = {"sample": sample, "field": i,
                                                "old": values_old[i], "new": values_new[i]}
                error = abs(values_old[i]-values_new[i])
                max_error = max(max_error, error)
                max_field_errors[i] = max(max_field_errors[i], error)
                if not math.isfinite(values_old[i]) or not math.isfinite(values_new[i]):
                    raise RuntimeError("Non-finite output")
            if not matrix_equal or not fields_equal or of != nf:
                mismatches.append(sample)
        reports.append({"scenario": scenario, "samples": samples, "matrix_calls_compared": compared_calls,
                        "mismatch_samples": len(mismatches), "first_matrix_mismatch": first_matrix_mismatch,
                        "first_field_mismatch": first_field_mismatch, "max_absolute_field_error": max_error,
                        "rejected_old_new": rejections, "extra_gain_calls_before_rejection": extra_gain_calls,
                        "structure_mismatch_samples": structure_mismatches,
                        "branch_mismatch_samples": branch_mismatches,
                        "last_flags_old_new": [of, nf], "last_scale_old_new": [values_old[71], values_new[71]],
                        "max_errors_by_group": {"quaternion": max(max_field_errors[:4]),
                                                "bias": max(max_field_errors[4:7]),
                                                "euler_degrees": max(max_field_errors[7:10]),
                                                "covariance": max(max_field_errors[16:52]),
                                                "nis": max_field_errors[52]},
                        "max_errors_by_field": max_field_errors})
    return {"profile": profile, "cases": reports}


def main():
    database, output_arg = sys.argv[1:3]
    output = Path(output_arg).resolve()
    output.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(output / "python"))
    results = []
    for profile in sys.argv[3:] or ("O0", "O3", "O3-strict"):
        elf = build(database, output, profile)
        result = emulate(elf, profile)
        results.append(result)
        print(profile, [(c["scenario"], c["mismatch_samples"], c["max_absolute_field_error"],
                         c["first_field_mismatch"]) for c in result["cases"]], flush=True)
    (output / "arm-math-results.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
    for profile in results:
        for case in profile["cases"]:
            if case["structure_mismatch_samples"] or case["branch_mismatch_samples"]:
                raise SystemExit("FAIL: matrix structure or gate decisions differ")
            if profile["profile"] == "O3-strict" and case["mismatch_samples"]:
                raise SystemExit("FAIL: strict floating-point trace differs")
    print("PASS: checked matrix dataflow and gate cases; inspect report for unsafe-math rounding differences.")


if __name__ == "__main__":
    main()
