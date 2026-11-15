"""Hardware-free tests for Jetson telemetry parsing and child process lifetime."""

import io
import subprocess
import unittest
from unittest.mock import Mock, patch

import pytest
from system_utils.tegrastats import TegrastatsReader, parse_gpu_stats


class TestTegrastats(unittest.TestCase):
    """Exercise documented output variants and unavailable telemetry."""

    def test_output_variants(self) -> None:
        """Read scalar/array clocks and utilization-only output, ignoring other engines."""
        cases = [
            ('RAM 200/8000MB GR3D_FREQ 42%@918 cpu@40C', (42.0, 0.918)),
            ('GR3D_FREQ 99%@[918,1098] VIC_FREQ 0%@115', (99.0, 1.098)),
            ('GR3D_FREQ 0%', (0.0, 0.0)),
            ('GR3D_FREQ 100%@0', (100.0, 0.0)),
            ('GR3D_FREQ @[918,918]', None),
            ('GR3D_FREQ 101%@918', None),
            ('GR3D_FREQ broken', None),
            ('GR3D_FREQ 42%@[918,]', None),
            ('RAM 200/8000MB', None),
        ]
        for line, expected in cases:
            with self.subTest(line=line):
                assert parse_gpu_stats(line) == expected

    def test_missing_executable(self) -> None:
        """Surface missing tools to the node's nonfatal startup handler."""
        with patch('system_utils.tegrastats.shutil.which', return_value=None), pytest.raises(FileNotFoundError):
            TegrastatsReader()

    def test_reader_and_cleanup(self) -> None:
        """Cache samples, reject stale/dead readings, and kill a child that ignores termination."""
        process = Mock()
        process.stdout = io.StringIO('warning\nGR3D_FREQ 42%@918\n')
        process.poll.return_value = None
        with patch('system_utils.tegrastats.shutil.which', return_value='/usr/bin/tegrastats'), \
                patch('system_utils.tegrastats.subprocess.Popen', return_value=process) as start:
            reader = TegrastatsReader()
            reader._thread.join(timeout=2)  # noqa: SLF001 - wait for the fixture stream to finish
            assert reader.get_gpu() == (42.0, 0.918)
            start.assert_called_once()
            with patch('system_utils.tegrastats.time.monotonic', return_value=float('inf')):
                assert reader.get_gpu() is None
            process.poll.return_value = 1
            assert reader.get_gpu() is None
            process.poll.return_value = None
            process.wait.side_effect = [subprocess.TimeoutExpired('tegrastats', 2), None]
            reader.close()
            process.terminate.assert_called_once()
            process.kill.assert_called_once()
            assert process.stdout.closed

    def test_no_gpu_output(self) -> None:
        """Ignore diagnostic output instead of publishing fabricated GPU measurements."""
        process = Mock()
        process.stdout = io.StringIO('cannot read GPU\n')
        process.poll.return_value = None
        with patch('system_utils.tegrastats.shutil.which', return_value='/usr/bin/tegrastats'), \
                patch('system_utils.tegrastats.subprocess.Popen', return_value=process):
            reader = TegrastatsReader()
            reader._thread.join(timeout=2)  # noqa: SLF001 - wait for the fixture stream to finish
            assert reader.get_gpu() is None
            reader.close()
