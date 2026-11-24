"""Compare every state/covariance sample against the frozen legacy model."""
import math
from pathlib import Path
import struct
import subprocess
import sys

reference, current, directory, scenario = sys.argv[1:]
output = Path(directory)
output.mkdir(parents=True, exist_ok=True)
before = output / "reference.bin"
after = output / "current.bin"
subprocess.run([reference, str(before), scenario], check=True)
subprocess.run([current, str(after), scenario], check=True)
original, migrated = before.read_bytes(), after.read_bytes()
expected_size = 20000 * 52 * 4
if len(original) != expected_size or len(migrated) != expected_size:
    raise SystemExit("FAIL: incomplete quaternion trace")
for data in (original, migrated):
    if any(not math.isfinite(value) for (value,) in struct.iter_unpack("<f", data)):
        raise SystemExit("FAIL: non-finite quaternion trace")
if original != migrated:
    max_error = 0.0
    first = None
    for index, ((a,), (b,)) in enumerate(zip(struct.iter_unpack("<f", original),
                                           struct.iter_unpack("<f", migrated))):
        if not math.isfinite(a) or not math.isfinite(b):
            raise SystemExit(f"FAIL: non-finite trace at float {index}")
        max_error = max(max_error, abs(a - b))
        if a != b and first is None:
            first = (index // 52, index % 52, a, b)
    raise SystemExit(f"FAIL: {scenario}: first mismatch {first}, max error {max_error}")
print(f"PASS: {scenario}, 20000 samples x 52 float32, byte-identical")
