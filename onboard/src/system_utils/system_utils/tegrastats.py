"""Read Jetson GPU telemetry without blocking the ROS executor."""

import re
import shutil
import subprocess
import threading
import time

_MAX_UTILIZATION = 100
_MAX_SAMPLE_AGE = 5.0
_GPU_PATTERN = re.compile(r'\bGR3D_FREQ\s+(\d+)%(?:@(?:\[(\d+(?:,\s*\d+)*)\]|(\d+)))?(?=\s|$)')


def parse_gpu_stats(line: str) -> tuple[float, float] | None:
    """Return utilization (%) and maximum reported GPC clock (GHz), if present."""
    match = _GPU_PATTERN.search(line)
    if match is None:
        return None
    utilization = float(match[1])
    if utilization > _MAX_UTILIZATION:
        return None
    clocks = match[2] or match[3]
    speed = max(float(clock) for clock in clocks.split(',')) / 1000 if clocks else 0.0
    return utilization, speed


class TegrastatsReader:
    """Cache fresh GPU samples from one owned tegrastats process."""

    def __init__(self) -> None:
        """Start sampling once per second; raise OSError if the tool cannot start."""
        executable = shutil.which('tegrastats')
        if executable is None:
            msg = 'tegrastats is not on PATH'
            raise FileNotFoundError(msg)
        self._lock = threading.Lock()
        self._sample: tuple[float, float, float] | None = None
        self._process = subprocess.Popen(  # noqa: S603 - resolved tool path, fixed arguments, no shell
            [executable, '--interval', '1000'],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            errors='replace',
        )
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self) -> None:
        """Consume output continuously so the child process cannot fill its pipe."""
        assert self._process.stdout is not None
        for line in self._process.stdout:
            stats = parse_gpu_stats(line)
            if stats is not None:
                with self._lock:
                    self._sample = (*stats, time.monotonic())

    def get_gpu(self) -> tuple[float, float] | None:
        """Return the latest sample, or None if stopped, absent, or over five seconds old."""
        with self._lock:
            sample = self._sample
        if self._process.poll() is not None or sample is None or time.monotonic() - sample[2] > _MAX_SAMPLE_AGE:
            return None
        return sample[:2]

    def close(self) -> None:
        """Stop only our child process and release its output pipe."""
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
        self._thread.join(timeout=2)
        if self._process.stdout is not None:
            self._process.stdout.close()
