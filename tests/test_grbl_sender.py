#!/usr/bin/env python
"""
Tests for the GRBL sender of the Levelling plugin, run against a simulated GRBL 1.1 controller: jogging, zeroing,
the Probe Z touch off, homing, the parameter readout, stopping and resetting, the state of the panel buttons, and
a job that is streamed through a tool change at which the operator touches off the new tool.

Run: python tests/test_grbl_sender.py
"""

import logging
import os
import re
import sys
import threading
import time
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from appPlugins.ToolLevelling import ToolLevelling


class FakeGrbl:
    """
    A serial port with a GRBL 1.1 controller behind it. It answers the way the firmware does, but every move is
    done the moment it is received.

    Machine positions are kept in `mpos`; the work position is the machine position minus the work coordinate
    offset `wco`.
    """

    word_re = re.compile(r"([A-Z])([-+]?\d*\.?\d+)")

    def __init__(self, surface_z=None, travel=None, chunk=None):
        self.is_open = True
        self.port = 'COM_TEST'
        self.lock = threading.RLock()

        self.state = 'Idle'
        self.mpos = [0.0, 0.0, 0.0]
        self.wco = [0.0, 0.0, 0.0]
        self.relative = False
        # the machine Z at which the probe touches; None for a probe that touches nothing
        self.surface_z = surface_z
        # the soft limits, as a (min, max) pair for every axis, or None when they are off
        self.travel = travel
        # when set, a read returns at most that many bytes, which splits answers the way a real port does
        self.chunk = chunk
        # when set, nothing is answered, like a controller that is busy with a long homing
        self.mute = False

        self.settings = {0: '10', 1: '25', 10: '1', 11: '0.010', 20: '0', 22: '1', 100: '800.000', 110: '5000.000'}

        self.lines = []
        self.realtime = []
        self._rx = ''
        self._tx = b''

    # ---------------------------------------------------------------------------------------- the serial port
    def write(self, data):
        with self.lock:
            for ch in data.decode('latin-1'):
                if ch in '?!~\x18':
                    self.realtime.append(ch)
                    self._realtime(ch)
                elif ch == '\n':
                    line, self._rx = self._rx.strip(), ''
                    self.lines.append(line)
                    if not self.mute:
                        self._line(line)
                elif ch != '\r':
                    self._rx += ch
        return len(data)

    def readlines(self):
        with self.lock:
            if self.chunk:
                data, self._tx = self._tx[:self.chunk], self._tx[self.chunk:]
            else:
                data, self._tx = self._tx, b''

        # like pyserial: split after every line feed; the last piece is incomplete when it has none
        parts = data.split(b'\n')
        lines = [part + b'\n' for part in parts[:-1]]
        if parts[-1]:
            lines.append(parts[-1])
        return lines

    def reset_input_buffer(self):
        with self.lock:
            self._tx = b''

    def close(self):
        self.is_open = False

    # ------------------------------------------------------------------------------------------- helpers
    @property
    def wpos(self):
        return [m - w for m, w in zip(self.mpos, self.wco)]

    def sent(self, word):
        """The lines that hold the word, 'G10' or 'M0' for instance, and not one that only starts the same."""
        pattern = re.compile(r"(?<![A-Z0-9.])%s(?![0-9.])" % re.escape(word.upper()))
        return [line for line in self.lines if pattern.search(line.upper())]

    def _emit(self, text):
        self._tx += (text + '\r\n').encode('utf-8')

    def _status(self):
        return '<%s|MPos:%.3f,%.3f,%.3f|FS:0,0|WCO:%.3f,%.3f,%.3f>' % (self.state, *self.mpos, *self.wco)

    def _realtime(self, ch):
        if ch == '?':
            self._emit(self._status())
        elif ch == '!':
            if self.state in ('Run', 'Jog'):
                self.state = 'Hold:0'
        elif ch == '~':
            if self.state.startswith('Hold'):
                self.state = 'Idle'
        elif ch == '\x18':
            # a reset in motion loses the position; one in a completed hold does not
            if self.state in ('Run', 'Jog'):
                self.state = 'Alarm'
            elif not self.state.startswith('Alarm'):
                self.state = 'Idle'
            self.relative = False
            self._rx = ''
            self._tx = b''
            self._emit('')
            self._emit("Grbl 1.1h ['$' for help]")

    @staticmethod
    def _words(text):
        """The words of a line, or None when the line is malformed."""
        text = re.sub(r"\(.*?\)", '', text.upper()).replace(' ', '')
        words = FakeGrbl.word_re.findall(text)
        if ''.join(letter + value for letter, value in words) != text:
            return None
        return words

    def _target(self, words, relative, machine=False):
        target = list(self.mpos)
        for letter, value in words:
            if letter in 'XYZ':
                axis = 'XYZ'.index(letter)
                if relative:
                    target[axis] += float(value)
                elif machine:
                    target[axis] = float(value)
                else:
                    target[axis] = float(value) + self.wco[axis]
        return target

    def _within_travel(self, target):
        if self.travel is None:
            return True
        return all(low <= value <= high for value, (low, high) in zip(target, self.travel))

    # ------------------------------------------------------------------------------------------ the lines
    def _line(self, line):
        upper = line.upper()

        if upper == '':
            self._emit('ok')
        elif upper.startswith('$'):
            self._system(upper)
        elif self.state.startswith('Alarm'):
            self._emit('error:9')
        else:
            self._gcode(upper)

    def _system(self, upper):
        if upper == '$H':
            if self.settings[22] == '0':
                self._emit('error:5')
                return
            self.mpos = [0.0, 0.0, 0.0]
            self.state = 'Idle'
            self._emit('ok')
        elif upper == '$X':
            if self.state.startswith('Alarm'):
                self.state = 'Idle'
                self._emit('[MSG:Caution: Unlocked]')
            self._emit('ok')
        elif upper == '$$':
            for key in sorted(self.settings):
                self._emit('$%d=%s' % (key, self.settings[key]))
            self._emit('ok')
        elif upper.startswith('$J='):
            self._jog(upper[3:])
        else:
            found = re.match(r"^\$(\d+)=(.+)$", upper)
            if found:
                self.settings[int(found.group(1))] = found.group(2)
                self._emit('ok')
            else:
                self._emit('error:3')

    def _jog(self, body):
        if self.state != 'Idle':
            self._emit('error:8')
            return

        words = self._words(body)
        if words is None:
            self._emit('error:2')
            return
        if not any(letter == 'F' for letter, _v in words):
            self._emit('error:22')
            return

        codes = ['G%s' % value for letter, value in words if letter == 'G']
        target = self._target(words, relative='G91' in codes, machine='G53' in codes)
        if not self._within_travel(target):
            self._emit('error:15')
            return

        # a jog leaves the modal state of the parser alone
        self.mpos = target
        self._emit('ok')

    def _gcode(self, upper):
        words = self._words(upper)
        if words is None:
            self._emit('error:2')
            return

        codes = {('%s%g' % (letter, float(value))) for letter, value in words if letter in 'GM'}
        values = {letter: value for letter, value in words if letter not in 'GM'}

        if 'M6' in codes:
            self._emit('error:20')
            return

        if 'G90' in codes:
            self.relative = False
        if 'G91' in codes:
            self.relative = True

        if 'G10' in codes:
            if values.get('L') == '20':
                for letter, value in words:
                    if letter in 'XYZ':
                        axis = 'XYZ'.index(letter)
                        self.wco[axis] = self.mpos[axis] - float(value)
            elif values.get('L') == '2':
                for letter, value in words:
                    if letter in 'XYZ':
                        self.wco['XYZ'.index(letter)] = float(value)
            self._emit('ok')
            return

        if 'G38.2' in codes:
            target = self._target(words, relative=self.relative)
            start_z = self.mpos[2]
            if self.surface_z is not None and target[2] <= self.surface_z <= start_z:
                self.mpos[2] = self.surface_z
                self._emit('[PRB:%.3f,%.3f,%.3f:1]' % tuple(self.mpos))
                self._emit('ok')
            else:
                self.mpos = target
                self.state = 'Alarm'
                self._emit('ALARM:5')
            return

        if 'M0' in codes or 'M1' in codes:
            self.state = 'Hold:0'

        if any(letter in 'XYZ' for letter, _v in words):
            self.mpos = self._target(words, relative=self.relative)

        self._emit('ok')


class Recorder:
    """Stands in for a Qt signal and for the inform signals of the application."""

    def __init__(self, on_emit=None):
        self.messages = []
        self.on_emit = on_emit

    def emit(self, *args):
        self.messages.append(' '.join(str(a) for a in args))
        if self.on_emit:
            self.on_emit(*args)

    def __getitem__(self, _signature):
        return self

    def text(self):
        return '\n'.join(self.messages)


class FakeWidget:
    def __init__(self, value=None):
        self.value = value
        self.enabled = True
        self.checked = False
        self.text = ''

    def setDisabled(self, disabled):
        self.enabled = not disabled

    def isEnabled(self):
        return self.enabled

    def setChecked(self, checked):
        self.checked = bool(checked)

    def isChecked(self):
        return self.checked

    def get_value(self):
        return self.value

    def set_value(self, value):
        self.value = value

    def setStyleSheet(self, _style):
        pass

    def setText(self, text):
        self.text = text

    def count(self):
        return 0


class FakeUI:
    """Every widget the sender reaches for exists; the ones with a value are given it."""

    def __init__(self, **values):
        self._values = values

    def __getattr__(self, name):
        widget = FakeWidget(self._values.get(name))
        setattr(self, name, widget)
        return widget


def make_sender(port):
    """A ToolLevelling that talks to the given port and to nothing else of the application."""
    tool = ToolLevelling.__new__(ToolLevelling)

    tool.app = SimpleNamespace(
        inform=Recorder(),
        inform_shell=Recorder(),
        options={"tools_al_grbl_travelz": 15.0},
        log=logging.getLogger('test_grbl_sender'),
        ui=SimpleNamespace(shell_dock=SimpleNamespace(show=lambda: None)),
        shell_message=lambda *args, **kwargs: None,
        collection=SimpleNamespace(get_by_name=lambda name: None),
    )
    tool.ui = FakeUI(jog_step_entry=5.0, jog_fr_entry=1500.0, probe_travel_entry=20.0, probe_fr_entry=50.0,
                     plate_thickness_entry=0.0, object_combo='')
    tool.units = 'MM'

    tool.grbl_ser_port = port
    tool.grbl_job_stop = False
    tool.grbl_job_waiting = False
    tool.grbl_job_lines = []
    tool.grbl_job_last_ack = 0
    tool.grbl_rx_partial = b''
    tool.grbl_job_line_timeout = 5.0
    tool.grbl_job_running = False
    tool.grbl_can_resume = False
    tool.grbl_busy = 0
    tool.grbl_resets = 0

    tool.grbl_job_progress_sig = Recorder()
    tool.grbl_job_paused_sig = Recorder()
    tool.grbl_job_finished_sig = Recorder()
    tool.grbl_busy_done_sig = Recorder(on_emit=lambda *args: tool.on_grbl_busy_done())
    return tool


def wait_for(condition, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.02)
    return False


class TestJogCommands(unittest.TestCase):
    def test_a_step_in_every_direction(self):
        expected = {
            'xplus': '$J=G91 G21 X5.0 F1500.0', 'xminus': '$J=G91 G21 X-5.0 F1500.0',
            'yplus': '$J=G91 G21 Y5.0 F1500.0', 'yminus': '$J=G91 G21 Y-5.0 F1500.0',
            'zplus': '$J=G91 G21 Z5.0 F1500.0', 'zminus': '$J=G91 G21 Z-5.0 F1500.0',
        }
        for direction, command in expected.items():
            self.assertEqual(ToolLevelling.grbl_jog_commands(direction, 'MM', '5.0', '1500.0', '15.0'), [command])

    def test_the_origin_is_reached_after_lifting(self):
        self.assertEqual(ToolLevelling.grbl_jog_commands('origin', 'MM', '5.0', '1500.0', '15.0'),
                         ['$J=G90 G21 Z15.0 F1500.0', '$J=G90 G21 X0.0 Y0.0 F1500.0'])

    def test_inch(self):
        self.assertEqual(ToolLevelling.grbl_jog_commands('xplus', 'IN', '0.1', '50.0', '0.6'),
                         ['$J=G91 G20 X0.1 F50.0'])

    def test_an_unknown_direction_sends_nothing(self):
        self.assertEqual(ToolLevelling.grbl_jog_commands(None, 'MM', '5.0', '1500.0', '15.0'), [])


class TestManualControl(unittest.TestCase):
    def setUp(self):
        self.grbl = FakeGrbl()
        self.sender = make_sender(self.grbl)

    def test_every_arrow_moves_the_machine(self):
        moves = {'xplus': (5, 0, 0), 'xminus': (0, 0, 0), 'yplus': (0, 5, 0), 'yminus': (0, 0, 0),
                 'zminus': (0, 0, -5), 'zplus': (0, 0, 0)}
        for direction, position in moves.items():
            self.sender.on_grbl_jog(direction)
            self.assertEqual(self.grbl.mpos, [float(value) for value in position], direction)
        self.assertNotIn('rejected', self.sender.app.inform.text())

    def test_the_jog_step_is_sent_as_a_number(self):
        # the step was once a tuple, which turned every jog into "$J=G91 G21 X(5.0,) F1500.0"
        self.sender.on_grbl_jog('xplus')
        self.assertEqual(self.grbl.sent('$J=')[-1], '$J=G91 G21 X5.0 F1500.0')

    def test_go_to_origin_returns_to_the_work_origin(self):
        self.grbl.mpos = [-120.0, -80.0, -30.0]
        self.sender.on_grbl_zero('all')
        self.grbl.mpos = [-50.0, -10.0, -25.0]

        self.sender.on_grbl_jog('origin')
        self.assertEqual(self.grbl.wpos, [0.0, 0.0, 15.0])

    def test_a_refused_jog_is_reported_with_the_state(self):
        self.grbl.state = 'Alarm'
        self.sender.on_grbl_jog('xplus')

        self.assertEqual(self.grbl.mpos, [0.0, 0.0, 0.0])
        self.assertIn('rejected', self.sender.app.inform.text())
        self.assertIn('alarm', self.sender.app.inform.text())
        # the answer of the controller is in the shell even though the jogs are not echoed
        self.assertIn('ERROR:8', self.sender.app.inform_shell.text())

    def test_a_jog_past_the_soft_limits_is_reported(self):
        self.grbl.travel = ((-400.0, 0.0), (-400.0, 0.0), (-95.0, 0.0))
        self.sender.on_grbl_jog('xplus')

        self.assertEqual(self.grbl.mpos, [0.0, 0.0, 0.0])
        self.assertIn('ERROR:15', self.sender.app.inform_shell.text())
        self.assertIn('idle', self.sender.app.inform.text())

    def test_zero_z_keeps_the_machine_position(self):
        self.grbl.mpos = [-100.0, -100.0, -20.0]
        self.sender.on_grbl_zero('z')

        self.assertEqual(self.grbl.mpos, [-100.0, -100.0, -20.0])
        self.assertEqual(self.grbl.wpos[2], 0.0)
        # X and Y keep their origin
        self.assertEqual(self.grbl.wpos[:2], [-100.0, -100.0])

    def test_zero_all(self):
        self.grbl.mpos = [-100.0, -150.0, -20.0]
        self.sender.on_grbl_zero('all')
        self.assertEqual(self.grbl.wpos, [0.0, 0.0, 0.0])

    def test_zero_does_not_write_the_settings(self):
        self.sender.on_grbl_zero('z')
        self.assertEqual([line for line in self.grbl.lines if line.startswith('$')], [])

    def test_get_parameter_is_an_exact_match(self):
        self.assertEqual(self.sender.on_grbl_get_parameter('1'), 25.0)
        self.assertEqual(self.sender.on_grbl_get_parameter('$10'), 1.0)
        self.assertEqual(self.sender.on_grbl_get_parameter('100'), 800.0)
        self.assertIsNone(self.sender.on_grbl_get_parameter('999'))

    def test_get_parameter_with_answers_split_over_reads(self):
        self.grbl.chunk = 7
        self.assertEqual(self.sender.on_grbl_get_parameter('110'), 5000.0)

    def test_homing(self):
        self.grbl.state = 'Alarm'
        self.grbl.mpos = [-37.0, -12.0, -4.0]

        self.assertTrue(self.sender.grbl_homing())
        self.assertEqual(self.grbl.state, 'Idle')
        self.assertEqual(self.grbl.mpos, [0.0, 0.0, 0.0])

    def test_homing_that_is_disabled(self):
        self.grbl.settings[22] = '0'
        self.assertFalse(self.sender.grbl_homing())
        self.assertIn('$22=0', self.sender.app.inform.text())


class TestProbeZ(unittest.TestCase):
    def setUp(self):
        self.grbl = FakeGrbl(surface_z=-32.5)
        self.grbl.mpos = [-100.0, -100.0, -20.0]
        self.sender = make_sender(self.grbl)

    def commands(self, thickness='0.0'):
        return ToolLevelling.grbl_probe_z_commands(units='MM', travel='20.0', feedrate='50.0',
                                                   thickness=thickness, lift='15.0')

    def test_the_touch_point_becomes_z_zero(self):
        self.assertTrue(self.sender.grbl_probe_z(self.commands()))

        # the tool was lifted to the travel height above the surface it touched
        self.assertAlmostEqual(self.grbl.wpos[2], 15.0)
        self.assertAlmostEqual(self.grbl.mpos[2], -32.5 + 15.0)
        self.assertFalse(self.grbl.relative)
        self.assertEqual(self.grbl.state, 'Idle')

    def test_the_plate_thickness(self):
        self.assertTrue(self.sender.grbl_probe_z(self.commands(thickness='1.6')))
        # the plate top is at Z 1.6, therefore the material under it is at Z 0
        self.assertAlmostEqual(self.grbl.mpos[2] - self.grbl.wco[2], 15.0)
        self.assertAlmostEqual(-32.5 - self.grbl.wco[2], 1.6)

    def test_a_probe_that_touches_nothing_leaves_the_origin_alone(self):
        self.grbl.surface_z = None
        wco_before = list(self.grbl.wco)

        self.assertFalse(self.sender.grbl_probe_z(self.commands()))

        self.assertEqual(self.grbl.sent('G10'), [])
        self.assertEqual(self.grbl.wco, wco_before)
        self.assertEqual(self.grbl.state, 'Alarm')
        self.assertIn('did not touch', self.sender.app.inform.text())


class TestStopAndReset(unittest.TestCase):
    def setUp(self):
        self.grbl = FakeGrbl()
        self.sender = make_sender(self.grbl)

    def test_stop_empties_the_controller_and_keeps_the_position(self):
        self.grbl.mpos = [-10.0, -20.0, -3.0]
        self.grbl.state = 'Run'
        self.grbl.write(b'!')

        self.sender.grbl_finish_stop(timeout=2.0)

        self.assertIn('\x18', self.grbl.realtime)
        self.assertEqual(self.grbl.state, 'Idle')
        self.assertEqual(self.grbl.mpos, [-10.0, -20.0, -3.0])

    def test_the_machine_can_be_jogged_after_a_stop(self):
        self.grbl.state = 'Run'
        self.grbl.write(b'!')
        self.sender.grbl_finish_stop(timeout=2.0)

        self.sender.on_grbl_jog('zplus')
        self.assertEqual(self.grbl.mpos[2], 5.0)

    def test_a_reset_ends_the_wait_for_an_answer(self):
        self.grbl.mute = True
        result = {}

        def wait():
            started = time.monotonic()
            result['answer'] = self.sender.send_grbl_command('$H', timeout=30.0)
            result['took'] = time.monotonic() - started

        thread = threading.Thread(target=wait)
        thread.start()
        time.sleep(0.3)
        self.sender.grbl_resets += 1
        thread.join(5.0)

        self.assertEqual(result['answer'], '')
        self.assertLess(result['took'], 2.0)


class TestPanelState(unittest.TestCase):
    def setUp(self):
        self.sender = make_sender(FakeGrbl())
        self.ui = self.sender.ui

    def manual_controls(self):
        return [self.ui.jog_wdg, self.ui.zero_axs_wdg, self.ui.probe_z_button, self.ui.grbl_command_entry,
                self.ui.grbl_send_button, self.ui.grbl_report_button, self.ui.grbl_get_param_button]

    def test_idle(self):
        self.sender.grbl_refresh_controls()
        self.assertTrue(all(widget.isEnabled() for widget in self.manual_controls()))
        self.assertTrue(self.ui.grbl_send_job_button.isEnabled())
        self.assertFalse(self.ui.grbl_stop_job_button.isEnabled())
        self.assertFalse(self.ui.grbl_continue_button.isEnabled())

    def test_streaming(self):
        self.sender.grbl_job_running = True
        self.sender.grbl_refresh_controls()
        self.assertFalse(any(widget.isEnabled() for widget in self.manual_controls()))
        self.assertFalse(self.ui.grbl_send_job_button.isEnabled())
        self.assertFalse(self.ui.grbl_get_heightmap_button.isEnabled())
        self.assertTrue(self.ui.grbl_stop_job_button.isEnabled())

    def test_waiting_at_a_tool_change(self):
        self.sender.grbl_job_running = True
        self.sender.grbl_job_waiting = True
        self.sender.grbl_refresh_controls()
        self.assertTrue(all(widget.isEnabled() for widget in self.manual_controls()))
        self.assertTrue(self.ui.grbl_continue_button.isEnabled())
        self.assertTrue(self.ui.grbl_stop_job_button.isEnabled())
        self.assertFalse(self.ui.grbl_send_job_button.isEnabled())
        self.assertFalse(self.ui.grbl_get_heightmap_button.isEnabled())

    def test_a_probe_at_a_tool_change_holds_back_continue(self):
        self.sender.grbl_job_running = True
        self.sender.grbl_job_waiting = True
        self.sender.grbl_busy = 1
        self.sender.grbl_refresh_controls()
        self.assertFalse(self.ui.grbl_continue_button.isEnabled())
        self.assertFalse(any(widget.isEnabled() for widget in self.manual_controls()))

        self.sender.on_grbl_busy_done()
        self.assertTrue(self.ui.grbl_continue_button.isEnabled())
        self.assertTrue(all(widget.isEnabled() for widget in self.manual_controls()))

    def test_a_job_that_ended_early_can_be_resumed(self):
        self.sender.grbl_job_running = True
        self.sender.grbl_job_lines = [('G00 Z1', None, False)] * 10
        self.sender.grbl_job_last_ack = 4
        self.sender.on_grbl_job_finished()

        self.assertTrue(self.ui.grbl_resume_job_button.isEnabled())
        self.assertEqual(self.ui.grbl_resume_line_entry.get_value(), 5)
        self.assertTrue(self.ui.grbl_send_job_button.isEnabled())

    def test_disconnect_closes_the_port(self):
        port = self.sender.grbl_ser_port
        self.sender.on_grbl_connect()

        self.assertFalse(port.is_open)
        self.assertEqual(self.ui.com_connect_button.text, 'Disconnected')
        self.assertFalse(self.ui.controller_reset_button.isEnabled())


class TestToolChangeJob(unittest.TestCase):
    """
    A two tool job streamed through its tool change. At the pause the operator jogs, a first probe misses, the
    controller is unlocked and the second probe sets the length of the new tool.
    """

    job = '\n'.join([
        'G21', 'G90', 'G94',
        'G00 Z15.0', 'G00 X10.0 Y10.0', 'M03 S1000', 'G01 Z-1.0 F100.0', 'G00 Z15.0', 'M05',
        'G00 Z15.0', 'T2',
        '(MSG, Change to Tool Dia = 0.8)',
        'M0',
        'G00 Z15.0', 'G00 X20.0 Y20.0', 'M03 S1000', 'G01 Z-1.0 F100.0', 'G00 Z15.0', 'M05',
    ]) + '\n'

    def setUp(self):
        self.grbl = FakeGrbl(surface_z=-40.0)
        self.grbl.mpos = [-200.0, -200.0, -30.0]
        self.sender = make_sender(self.grbl)

        # the job origin: the surface is at machine Z -40
        self.grbl.wco = [-200.0, -200.0, -40.0]

        self.paused = threading.Event()
        self.sender.grbl_job_paused_sig = Recorder(on_emit=lambda *args: self.paused.set())

    def tearDown(self):
        # a test that failed half way leaves the job waiting at the pause
        self.sender.grbl_job_stop = True
        self.sender.grbl_job_waiting = False

    def stream(self, chunk=None):
        self.grbl.chunk = chunk
        thread = threading.Thread(target=self.sender.grbl_stream_gcode, args=(self.job,), daemon=True)
        thread.start()
        return thread

    def probe(self):
        commands = ToolLevelling.grbl_probe_z_commands(units='MM', travel='20.0', feedrate='50.0',
                                                       thickness='0.0', lift='15.0')
        return self.sender.grbl_probe_z(commands)

    def press_continue(self):
        self.paused.clear()
        self.sender.grbl_job_waiting = False

    def test_touch_off_at_the_tool_change(self):
        thread = self.stream()
        self.assertTrue(self.paused.wait(5.0), "the job did not stop at the tool change")

        # the pause is held by the sender: the controller never saw it and is idle, so it can be jogged
        self.assertEqual(self.grbl.sent('M0'), [])
        self.assertEqual(self.grbl.state, 'Idle')
        self.assertTrue(self.sender.grbl_job_waiting)
        self.assertIn('Change to Tool Dia = 0.8', self.sender.grbl_job_paused_sig.text())

        # the job left the tool at Z 15 above the surface at machine Z -40
        self.assertAlmostEqual(self.grbl.mpos[2], -25.0)
        self.sender.on_grbl_jog('zminus')
        self.assertAlmostEqual(self.grbl.mpos[2], -30.0)

        # the new tool is 3 mm longer; the first probe misses because the clip fell off
        self.grbl.surface_z = None
        self.assertFalse(self.probe())
        self.assertEqual(self.grbl.state, 'Alarm')
        self.assertTrue(self.grbl.relative)

        # continuing from the alarm is refused and the job waits again
        self.press_continue()
        self.assertTrue(self.paused.wait(5.0), "the job went on from an alarm")
        self.assertIn('alarm state', self.sender.app.inform.text())
        self.assertTrue(thread.is_alive())

        self.assertTrue(self.sender.grbl_send_manual('$X'))
        self.grbl.mpos[2] = -30.0
        self.grbl.surface_z = -43.0
        self.assertTrue(self.probe())

        # the operator typed a relative move in the command box and left the controller in relative mode
        self.assertTrue(self.sender.grbl_send_manual('G91'))
        self.assertTrue(self.grbl.relative)
        lines_before_continue = len(self.grbl.lines)

        self.press_continue()
        thread.join(5.0)
        self.assertFalse(thread.is_alive())

        # the modal state was sent again before the job went on
        self.assertEqual(self.grbl.lines[lines_before_continue:lines_before_continue + 3], ['G21', 'G90', 'G94'])
        self.assertFalse(self.grbl.relative)

        # the second tool cut 1 mm below the surface it was touched off on, and ended at the travel height
        self.assertAlmostEqual(self.grbl.wco[2], -43.0)
        self.assertAlmostEqual(self.grbl.wpos[2], 15.0)
        self.assertEqual(self.grbl.wpos[:2], [20.0, 20.0])

        self.assertEqual(len(self.sender.grbl_job_finished_sig.messages), 1)
        self.assertIn('The job was sent', self.sender.app.inform.text())
        self.assertNotIn('rejected the line', self.sender.app.inform.text())

    def test_answers_split_over_reads(self):
        thread = self.stream(chunk=16)
        self.assertTrue(self.paused.wait(5.0))
        self.press_continue()
        thread.join(10.0)

        self.assertFalse(thread.is_alive())
        self.assertIn('The job was sent', self.sender.app.inform.text())
        self.assertEqual(self.sender.grbl_job_last_ack, len(self.sender.grbl_job_lines))

    def test_stop_at_the_tool_change(self):
        thread = self.stream()
        self.assertTrue(self.paused.wait(5.0))

        self.sender.grbl_job_stop = True
        self.sender.grbl_job_waiting = False
        thread.join(5.0)

        self.assertFalse(thread.is_alive())
        # the job can be resumed after the tool change
        self.assertEqual(self.sender.grbl_job_last_ack, 12)
        self.assertNotIn('did not answer', self.sender.app.inform.text())


if __name__ == '__main__':
    unittest.main(verbosity=2)
