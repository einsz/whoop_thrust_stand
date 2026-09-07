"""Host failure paths using the emulator's schema and an in-memory serial peer."""
import contextlib
import csv
import io
import itertools
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import measure
import plot_comparison
from tools.fake_stand import BANNER


class SerialPeer:
    def __init__(self, ending):
        self.ending = ending
        self.queue = []
        self.commands = []
        self.is_open = True

    def write(self, data):
        command = data.decode().strip()
        self.commands.append(command)
        mode = command.split(',')[0]
        if mode == 'ID':
            self.queue.extend(BANNER)
        elif mode in ('SWEEP', 'RESPONSE', 'TRANSIENT', 'COASTDOWN', 'DHOLD', 'HOLD'):
            token = 'HOLD' if mode in ('DHOLD', 'HOLD') else mode
            phase = 2 if mode == 'SWEEP' else 7
            self.queue.append('START_' + token)
            cols = BANNER[-1].split(',')[1:]
            for n in range(1, 4):
                values = dict.fromkeys(cols, 0)
                values.update(t_us=n*4000, phase=phase, throttle_pct=40,
                              dshot=788, rpm=20000, erpm=120000, erpm_raw=500,
                              n_rpm=n, n_thrust=n, n_ina=n, thrust_raw=-17800,
                              bus_mv=4000, bus_ua=1000000, shunt_uv=10000)
                self.queue.append('D,' + ','.join(str(values[c]) for c in cols))
            if self.ending == 'complete':
                self.queue.append('END_' + token)
            elif self.ending == 'teardown':
                # The firmware's endSequence() order: reason, stats, closing
                # ambient, sentinel. The host has to read all four.
                self.queue.extend([
                    '#WARN,sequence_aborted,stop',
                    '#STATS,' + token.lower() + ',telem_ok=900,telem_corrupt=7,'
                    'telem_silent=2,rows_dropped=0',
                    '#AMBIENT,end,present=1,temp_c=23.10,press_pa=100849.0',
                    'END_' + token])
            else:
                self.queue.append(self.ending)

    def readline(self):
        if not self.queue:
            return b''
        line = self.queue.pop(0)
        if isinstance(line, BaseException):
            raise line
        return (line + '\n').encode()

    def reset_input_buffer(self):
        self.queue.clear()

    def close(self):
        self.is_open = False


class CaptureTests(unittest.TestCase):
    def invoke(self, directory, ending, mode='RESPONSE', extra=()):
        peer = SerialPeer(ending)
        output = Path(directory) / 'run.csv'
        args = ['measure.py', '-m', mode, '-o', str(output), '--prop-hand', 'normal',
                '--scale', '-17800', '--no-tare', *extra]
        clock = itertools.count(0, 0.25)
        with patch.object(measure.serial, 'Serial', return_value=peer), \
             patch.object(measure, 'load_stand_config', return_value={}), \
             patch.object(measure.time, 'time', side_effect=lambda: next(clock)), \
             patch.object(measure.time, 'sleep'), \
             patch.object(measure.Link, 'start_keepalive') as keepalive, \
             patch.object(sys, 'argv', args), \
             contextlib.redirect_stdout(io.StringIO()), \
             contextlib.redirect_stderr(io.StringIO()):
            code = measure.main()
        self.assertFalse(peer.is_open)
        self.assertEqual(peer.commands[-2:], ['0', 'STOP'])
        keepalive.assert_called_once()
        text = output.read_text()
        rows = list(csv.DictReader(line for line in text.splitlines() if not line.startswith('#')))
        raw = list(Path(directory).glob('run.csv.raw-*.log'))
        self.assertEqual(len(raw), 1)
        self.assertIn('#COLS,', raw[0].read_text())
        return code, text, rows, raw[0].read_text(), peer

    def test_time_series_failures_preserve_samples_and_reason(self):
        for ending, reason, expected in [
            (KeyboardInterrupt(), 'interrupted', 130),
            (measure.serial.SerialException('disconnected'), 'disconnected', 1),
            ('', 'no output from the board', 1),
            ('SYSTEM_READY', 'firmware restarted', 1),
            ('#WARN,sequence_aborted,watchdog', 'sequence_aborted;watchdog', 1),
            ('#WARN,sequence_aborted,stop', 'sequence_aborted;stop', 1),
            ('#WARN,sequence_aborted,core0_stall', 'sequence_aborted;core0_stall', 1),
            ('#ERROR,idle_hunt_failed', 'idle_hunt_failed', 1),
        ]:
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as directory:
                code, text, rows, raw, _ = self.invoke(directory, ending)
                self.assertEqual(code, expected)
                self.assertIn('status=aborted', text)
                self.assertIn(reason, text)
                self.assertEqual(len(rows), 3)
                self.assertIn('time_s', rows[0])
                self.assertEqual(sum(s.startswith('D,') for s in raw.splitlines()), 3)
                if isinstance(ending, str) and ending.startswith('#'):
                    self.assertIn(ending[1:].split(',')[1], text)

    def test_abort_teardown_is_read_before_giving_up(self):
        """The stats and closing ambient still reach the CSV of a failed run."""
        with tempfile.TemporaryDirectory() as directory:
            code, text, rows, raw, peer = self.invoke(directory, 'teardown')
            self.assertEqual(code, 1)
            self.assertIn('status=aborted', text)
            self.assertIn('sequence_aborted;stop', text)
            self.assertIn('# stats,response,telem_ok=900', text)
            self.assertIn('# ambient,end,', text)
            self.assertEqual(len(rows), 3)
            # Density is the one correction that cannot be applied afterwards,
            # so a partial run has to carry it like any other.
            self.assertIn('# conditions,source=bmp280', text)
            # Read to the sentinel and stopped there, not one line further.
            self.assertIn('END_RESPONSE', raw)

    def test_partial_marker_is_visible_to_consumers(self):
        with tempfile.TemporaryDirectory() as directory:
            _, _, _, _, _ = self.invoke(directory, 'teardown')
            partial = str(Path(directory) / 'run.csv')
            err = io.StringIO()
            with contextlib.redirect_stderr(err):
                self.assertTrue(measure.warn_if_partial(partial))
            self.assertIn('PARTIAL run', err.getvalue())
            meta, _ = plot_comparison.parse_csv(partial)
            self.assertTrue(plot_comparison.is_partial(meta))

    def test_complete_run_is_not_flagged_partial(self):
        with tempfile.TemporaryDirectory() as directory:
            self.invoke(directory, 'complete')
            done = str(Path(directory) / 'run.csv')
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertFalse(measure.warn_if_partial(done))
            meta, _ = plot_comparison.parse_csv(done)
            self.assertFalse(plot_comparison.is_partial(meta))

    def test_success_is_marked_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            code, text, rows, _, _ = self.invoke(directory, 'complete')
            self.assertIn(code, (None, 0))
            self.assertIn('status=complete', text)
            self.assertEqual(len(rows), 3)

    def test_aggregate_modes_stop_and_retain_raw_window(self):
        for mode, extra in [('SWEEP', ()), ('KV', ('--voltages', '4',)),
                            ('FINEWALK', ('--dshot-range', '1000,1020,20')),
                            ('LINKTEST', ('--throttles', '40,50'))]:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory, \
                 patch('builtins.input', return_value=''):
                code, text, rows, raw, peer = self.invoke(
                    directory, '#WARN,sequence_aborted,stop', mode, extra)
                self.assertEqual(code, 1)
                self.assertIn('status=aborted', text)
                self.assertIn('# conditions,source=bmp280', text)
                self.assertEqual(sum(s.startswith('D,') for s in raw.splitlines()), 3)
                if mode != 'LINKTEST':
                    self.assertEqual(len(rows), 1)
                if mode == 'SWEEP':
                    # Density-normalised thrust, on the rows that have a column
                    # for it. Aborting does not make the samples uncomparable.
                    self.assertTrue(rows[0]['thrust_g_iso'])
                holds = [c for c in peer.commands if c.startswith(('HOLD,', 'DHOLD,'))]
                self.assertLessEqual(len(holds), 1)

    def test_interrupted_calibration_never_updates_config(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch('builtins.input', side_effect=['', KeyboardInterrupt()]), \
             patch.object(measure, 'settling_tolerance', return_value=1), \
             patch.object(measure, 'wait_stable', return_value='settled'), \
             patch.object(measure, 'read_thrust_avg', return_value=(0, 0, 10, 0, 0)), \
             patch.object(measure, 'save_stand_config') as save:
            code, text, rows, _, _ = self.invoke(directory, 'complete', 'MASSCHECK')
            self.assertEqual(code, 130)
            self.assertEqual(len(rows), 1)
            self.assertIn('status=aborted', text)
            save.assert_not_called()

    def test_missing_port_returns_failure(self):
        with patch.object(measure.serial, 'Serial', side_effect=OSError('missing port')), \
             patch.object(sys, 'argv', ['measure.py', '--check']), \
             contextlib.redirect_stdout(io.StringIO()), \
             contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(measure.main(), 1)


if __name__ == '__main__':
    unittest.main()
