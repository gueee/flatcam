#!/usr/bin/env python
"""
Tests for the GRBL sender of the Levelling plugin, run against a simulated GRBL 1.1 controller: jogging, zeroing,
the Probe Z touch off, homing, the parameter readout, stopping and resetting, the state of the panel buttons, and
a job that is streamed through a tool change at which the operator touches off the new tool.

Run: python tests/test_grbl_sender.py
"""

import contextlib
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
        # when set, a move past machine 0 runs into the limit switch the machine homes against
        self.hard_limits = False
        # after a limit was hit the controller takes nothing but a reset and status queries
        self.critical = False

        self.settings = {0: '10', 1: '25', 10: '1', 11: '0.010', 20: '0', 21: '0', 22: '1', 23: '0', 27: '1.000',
                         100: '800.000', 110: '5000.000'}

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
                    if not self.mute and not self.critical:
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

    @property
    def in_waiting(self):
        with self.lock:
            return min(len(self._tx), self.chunk) if self.chunk else len(self._tx)

    def read(self, size=1):
        with self.lock:
            data, self._tx = self._tx[:size], self._tx[size:]
        return data

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
        pins = ''.join(axis for axis, value in zip('XYZ', self.mpos) if value >= -0.001)
        return '<%s|MPos:%.3f,%.3f,%.3f|FS:0,0|WCO:%.3f,%.3f,%.3f%s>' % (
            self.state, *self.mpos, *self.wco, '|Pn:' + pins if self.hard_limits and pins else '')

    def _alarm_limit(self):
        self.state = 'Alarm'
        self.critical = True
        self._emit('ALARM:1')
        self._emit('[MSG:Reset to continue]')

    def _move(self, target):
        """
        Move to the target; past machine 0 the machine runs into a limit switch and stops there. With the hard
        limits on ($21=1) the switch raises the alarm whenever it changes, also when the machine moves off it.
        """
        limits_on = self.hard_limits and self.settings[21] == '1'
        pressed = self.hard_limits and any(value >= -0.001 for value in self.mpos)
        if limits_on and pressed and target != self.mpos:
            self._alarm_limit()
            return
        if self.hard_limits and any(value > 0.0 for value in target):
            self.mpos = [min(value, 0.0) for value in target]
            if limits_on:
                self._alarm_limit()
            return
        self.mpos = target

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
            self.critical = False
            self.relative = False
            self._rx = ''
            self._tx = b''
            self._emit('')
            self._emit("Grbl 1.1h ['$' for help]")
            if self.state == 'Alarm':
                self._emit("[MSG:'$H'|'$X' to unlock]")

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
            # every axis homes in the positive direction and is pulled off the switch by $27
            self.mpos = [-float(self.settings[27])] * 3
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

        # a jog leaves the modal state of the parser alone; it is acknowledged before it moves
        self._emit('ok')
        self._move(target)

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

        self._emit('ok')
        if any(letter in 'XYZ' for letter, _v in words):
            self._move(self._target(words, relative=self.relative))


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
    tool.grbl_status = {'state': None, 'mpos': None, 'wpos': None, 'wco': None, 'pins': ''}
    tool.grbl_status_line = ''
    tool.grbl_homed = False
    tool.grbl_settings = {}
    tool.grbl_limits_suspended = False
    tool.grbl_poll_partial = b''
    tool.grbl_port_readers = 0
    tool.grbl_port_readers_lock = threading.Lock()
    tool.grbl_status_time = 0.0
    tool.grbl_status_timer = SimpleNamespace(start=lambda: None, stop=lambda: None)
    tool.ui.grbl_wpos_labels = [FakeWidget() for _i in range(3)]
    tool.ui.grbl_mpos_labels = [FakeWidget() for _i in range(3)]
    tool.ui.grbl_wco_labels = [FakeWidget() for _i in range(3)]

    tool.build_al_table_sig = Recorder()
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

    def test_the_lift_stops_below_the_top_of_the_machine(self):
        # no work origin was set, therefore the travel height is 15 mm above the top of the machine
        commands = ToolLevelling.grbl_jog_commands('origin', 'MM', '5.0', '1500.0', '15.0',
                                                   ceilings=[None, None, -1.0], wco=[-50.0, -60.0, 0.0])
        self.assertEqual(commands, ['$J=G53 G21 Z-1.000 F1500.0', '$J=G90 G21 X0.0 Y0.0 F1500.0'])

    def test_a_lift_that_is_low_enough_is_left_alone(self):
        commands = ToolLevelling.grbl_jog_commands('origin', 'MM', '5.0', '1500.0', '15.0',
                                                   ceilings=[-1.0, -1.0, -1.0], wco=[-50.0, -60.0, -40.0])
        self.assertEqual(commands, ['$J=G90 G21 Z15.0 F1500.0', '$J=G90 G21 X0.0 Y0.0 F1500.0'])

    def test_an_origin_at_the_switches_is_not_reached(self):
        commands = ToolLevelling.grbl_jog_commands('origin', 'MM', '5.0', '1500.0', '15.0',
                                                   ceilings=[-1.0, -1.0, -1.0], wco=[0.0, -60.0, -40.0])
        self.assertEqual(commands[1], '$J=G53 G21 X-1.000 Y-60.000 F1500.0')

    def test_a_step_up_stops_below_the_top(self):
        ceilings = [None, None, -1.0]
        self.assertEqual(ToolLevelling.grbl_jog_commands('zplus', 'MM', '5.0', '1500.0', '15.0',
                                                         ceilings=ceilings, mpos=[0.0, 0.0, -3.0]),
                         ['$J=G53 G21 Z-1.000 F1500.0'])
        self.assertEqual(ToolLevelling.grbl_jog_commands('zplus', 'MM', '5.0', '1500.0', '15.0',
                                                         ceilings=ceilings, mpos=[0.0, 0.0, -1.0]), [])
        self.assertEqual(ToolLevelling.grbl_jog_commands('zminus', 'MM', '5.0', '1500.0', '15.0',
                                                         ceilings=ceilings, mpos=[0.0, 0.0, -1.0]),
                         ['$J=G91 G21 Z-5.0 F1500.0'])

    def test_inch_moves_are_not_limited(self):
        # the controller reports in mm, which an inch step is not compared with
        self.assertEqual(ToolLevelling.grbl_jog_commands('zplus', 'IN', '0.5', '50.0', '0.6',
                                                         ceilings=[None, None, -1.0], mpos=[0.0, 0.0, -1.0]),
                         ['$J=G91 G20 Z0.5 F50.0'])


class TestAutolevelHeights(unittest.TestCase):
    """The work Z zero is on the copper, 35 mm below the machine zero; the board sags by 0.05 mm in the middle."""

    def setUp(self):
        self.grbl = FakeGrbl(surface_z=-35.05)
        self.grbl.mpos = [-100.0, -100.0, -30.0]
        self.grbl.wco = [-120.0, -120.0, -35.0]
        self.sender = make_sender(self.grbl)

    def level(self, heights):
        job = SimpleNamespace(is_levelled_gcode=False, coords_decimals=4, multitool=False, tools={},
                              is_segmented_gcode=True, obj_options={'type': 'Geometry'},
                              gcode='G00 Z5.0\nG01 X1.0 Y1.0\nG01 Z-0.1 F100\nG01 X9.0 Y9.0\n',
                              prepend_snippet='', append_snippet='', gc_start='')
        self.sender.app.collection = SimpleNamespace(get_by_name=lambda name: job)
        self.sender.ui.al_method_radio.set_value('b')
        self.sender.ui.al_rows_entry.set_value(2)
        self.sender.ui.al_columns_entry.set_value(2)
        self.sender.al_bilinear_geo_storage = [(0.0, 0.0, heights[0]), (10.0, 0.0, heights[1]),
                                               (0.0, 10.0, heights[2]), (10.0, 10.0, heights[3])]
        return self.sender.autolevell_gcode(), job

    def test_the_probed_height_is_in_work_coordinates(self):
        wco = self.sender.grbl_read_work_offset(timeout=2.0)
        answer = self.sender.send_grbl_command('G38.2 Z-10 F50', echo=False)
        machine_z = self.sender.parse_probe_answer(answer)

        self.assertAlmostEqual(machine_z, -35.05)
        self.assertAlmostEqual(ToolLevelling.grbl_work_height(machine_z, wco[2], 'MM'), -0.05)

    def test_inch(self):
        self.assertAlmostEqual(ToolLevelling.grbl_work_height(-35.254, -35.0, 'IN'), -0.01)

    def test_the_work_offset_is_read_fresh(self):
        self.sender.grbl_status['wco'] = [0.0, 0.0, 0.0]
        self.assertEqual(self.sender.grbl_read_work_offset(timeout=2.0), [-120.0, -120.0, -35.0])

    def test_heights_in_machine_coordinates_are_not_applied(self):
        # what the probing stored before: the Z of the [PRB:...] answers, 35 mm below the work Z zero
        result, job = self.level([-35.0, -35.02, -35.05, -35.01])

        self.assertEqual(result, 'fail')
        self.assertIn('Z-0.1 ', job.gcode + ' ')
        self.assertFalse(job.is_levelled_gcode)
        self.assertIn('not applied', self.sender.app.inform.text())

    def test_the_whole_probing_run(self):
        result, job = self.level([0.0, 0.0, 0.0, 0.0])
        job.is_levelled_gcode = False
        job.gcode = 'G00 Z5.0\nG01 X1.0 Y1.0\nG01 Z-0.1 F100\nG01 X9.0 Y9.0\n'
        self.sender.app.inform.messages.clear()

        self.sender.app.worker_task = Recorder(on_emit=lambda task: task['fcn']())
        self.sender.app.proc_container = SimpleNamespace(new=lambda text: contextlib.nullcontext())
        self.sender.probe_points = lambda: [(0.0, 0.0), (10.0, 0.0), (0.0, 10.0), (10.0, 10.0)]
        self.sender.al_voronoi_geo_storage = {}
        self.sender.build_al_table = lambda: None
        self.sender.ui.ptravelz_entry.set_value(5.0)
        self.sender.ui.feedrate_probe_entry.set_value(50.0)
        self.sender.ui.pdepth_entry.set_value(-10.0)

        self.sender.on_grbl_autolevel()

        self.assertEqual([round(h, 3) for _x, _y, h in self.sender.al_bilinear_geo_storage], [-0.05] * 4)
        self.assertTrue(job.is_levelled_gcode)
        # the cut follows the surface 0.05 mm below the work Z zero, and the travel height stays where it was
        self.assertIn('G01 Z-0.1500', job.gcode)
        self.assertIn('Z4.9500', job.gcode)
        self.assertNotIn('Z-35', job.gcode)

    def test_the_warp_of_a_board_is_applied(self):
        result, job = self.level([0.0, -0.02, -0.05, -0.01])

        self.assertIsNone(result)
        self.assertTrue(job.is_levelled_gcode)
        self.assertNotIn('Z-0.1 ', job.gcode)


class TestJobTopZ(unittest.TestCase):
    def test_the_highest_z(self):
        job = '(Z Start: 40 mm)\nG21\nG90\nG94\nG00 Z5.0\nG01 Z-1.5 F100\nG00 Z15.0 ; end\nM05\n'
        self.assertEqual(ToolLevelling.grbl_job_top_z(job), 15.0)

    def test_arc_modes_do_not_count_as_relative(self):
        self.assertEqual(ToolLevelling.grbl_job_top_z('G91.1\nG01 F300.00\nG00 Z2\n'), 2.0)

    def test_heights_that_are_not_known_before_the_job_runs(self):
        for job in ('G91\nG00 Z5\n', 'G20\nG00 Z0.5\n', 'G92 Z0\nG00 Z5\n', 'G55\nG00 Z5\n', 'G10 L20 P0 Z0\nG0 Z5\n'):
            self.assertIsNone(ToolLevelling.grbl_job_top_z(job), job)
        self.assertIsNone(ToolLevelling.grbl_job_top_z('G00 X1 Y1\n'))


class TestStatus(unittest.TestCase):
    def setUp(self):
        self.sender = make_sender(FakeGrbl())

    def test_machine_position_and_offset(self):
        self.sender.grbl_note_status('<Idle|MPos:-10.000,-20.000,-5.000|FS:0,0|WCO:-100.000,-50.000,-30.000>')
        status = self.sender.grbl_status
        self.assertEqual(status['state'], 'Idle')
        self.assertEqual(status['mpos'], [-10.0, -20.0, -5.0])
        self.assertEqual(status['wpos'], [90.0, 30.0, 25.0])
        self.assertEqual(status['pins'], '')

    def test_the_offset_is_kept_between_reports(self):
        self.sender.grbl_note_status('<Idle|MPos:0.000,0.000,0.000|FS:0,0|WCO:-1.000,-2.000,-3.000>')
        self.sender.grbl_note_status('<Jog|MPos:5.000,0.000,0.000|FS:500,0|Pn:Z>')
        status = self.sender.grbl_status
        self.assertEqual(status['wpos'], [6.0, 2.0, 3.0])
        self.assertEqual(status['pins'], 'Z')

    def test_work_position_reports(self):
        # $10=0 reports the work position
        self.sender.grbl_note_status('<Idle|WPos:1.000,2.000,3.000|FS:0,0|WCO:-10.000,-10.000,-10.000>')
        self.assertEqual(self.sender.grbl_status['mpos'], [-9.0, -8.0, -7.0])

    def test_the_newest_report_counts(self):
        grbl = self.sender.grbl_ser_port
        grbl.state = 'Alarm'
        # a report the position display asked for and that was not read yet
        grbl.write(b'?')
        grbl.state = 'Idle'
        self.assertEqual(self.sender.grbl_read_status(), 'idle')

    def test_a_limit_alarm_forgets_the_homing(self):
        self.sender.grbl_homed = True
        self.sender.grbl_note_lines(['ok', 'ALARM:1', '[MSG:Reset to continue]'])
        self.assertFalse(self.sender.grbl_homed)

    def test_the_display_polls_without_waiting(self):
        grbl = self.sender.grbl_ser_port
        grbl.chunk = 9
        for _i in range(20):
            self.sender.on_grbl_status_tick()
        self.assertEqual(self.sender.grbl_status['mpos'], [0.0, 0.0, 0.0])
        self.assertEqual(self.sender.ui.grbl_mpos_labels[2].get_value(), '0.000')

    def test_a_piece_of_a_report_is_not_used(self):
        self.sender.grbl_note_status('<Idle|MPos:0.000,0.000,162.5')
        self.assertIsNone(self.sender.grbl_status['mpos'])

    def test_the_display_does_not_read_in_the_middle_of_an_answer(self):
        # app.inform runs processEvents(), so the poll can run while an answer is being read on the GUI thread
        grbl = self.sender.grbl_ser_port
        grbl.write(b'?')
        with self.sender.grbl_port_reader():
            self.sender.on_grbl_status_tick()
            self.assertGreater(grbl.in_waiting, 0)
        self.assertEqual(self.sender.grbl_read_status(), 'idle')

    def test_a_display_that_is_not_updated_says_so(self):
        self.sender.grbl_note_status('<Idle|MPos:0.000,0.000,0.000|FS:0,0>')
        self.sender.grbl_status_time -= 5
        self.sender.grbl_update_position_ui()
        self.assertIn('NOT UPDATING', self.sender.ui.grbl_state_label.get_value())

    def test_the_display_shows_where_go_to_origin_goes(self):
        grbl = self.sender.grbl_ser_port
        self.sender.grbl_read_settings()
        self.sender.grbl_homing()
        grbl.wco = [-120.0, -80.0, 0.0]
        self.sender.grbl_read_status()
        self.sender.grbl_update_position_ui()

        text = self.sender.ui.grbl_origin_label.get_value()
        self.assertIn('machine X-120.000 Y-80.000 Z-1.000', text)
        self.assertIn('limit switch on: Z', text)

    def test_the_settings(self):
        self.sender.grbl_read_settings()
        self.assertEqual(self.sender.grbl_settings[27], '1.000')
        self.assertEqual(self.sender.grbl_ceilings(), [None, None, None])
        self.sender.grbl_homed = True
        self.assertEqual(self.sender.grbl_ceilings(), [-1.0, -1.0, -1.0])
        self.sender.grbl_settings[23] = '4'
        self.assertEqual(self.sender.grbl_ceilings(), [-1.0, -1.0, None])


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

    def test_a_jog_in_an_alarm_is_not_sent(self):
        self.grbl.state = 'Alarm'
        self.sender.on_grbl_jog('xplus')

        self.assertEqual(self.grbl.sent('$J='), [])
        self.assertIn('alarm state', self.sender.app.inform.text())
        self.assertIn('Do Home', self.sender.app.inform.text())

    def test_a_zero_in_a_hold_is_not_sent(self):
        self.grbl.state = 'Hold:0'
        self.assertFalse(self.sender.grbl_send_manual('G10 L20 P0 Z0'))
        self.assertEqual(self.grbl.sent('G10'), [])
        self.assertIn('Pause/Resume', self.sender.app.inform.text())

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
        self.assertEqual(self.grbl.mpos, [-1.0, -1.0, -1.0])
        self.assertTrue(self.sender.grbl_homed)

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


class TestLimitSwitches(unittest.TestCase):
    """A machine with hard limits, homed against its switches at the top, the back and the right."""

    def setUp(self):
        self.grbl = FakeGrbl()
        self.grbl.hard_limits = True
        self.grbl.settings[21] = '1'
        self.sender = make_sender(self.grbl)
        self.sender.grbl_read_settings()
        self.assertTrue(self.sender.grbl_homing())

    def crash(self):
        # a move typed in the command box is not limited, it runs into the switch
        self.sender.send_grbl_command('G0 X5', echo=False, timeout=3.0)
        self.assertTrue(self.grbl.critical)
        self.sender.app.inform.messages.clear()

    def test_go_to_origin_without_a_work_origin_stays_off_the_switches(self):
        # the work origin was never set, it is the machine origin, where the switches are
        self.sender.on_grbl_jog('origin')

        self.assertFalse(self.grbl.critical)
        self.assertEqual(self.grbl.state, 'Idle')
        self.assertEqual(self.grbl.mpos, [-1.0, -1.0, -1.0])

    def test_go_to_origin_with_a_work_origin(self):
        self.grbl.mpos = [-120.0, -80.0, -40.0]
        self.sender.on_grbl_zero('all')
        self.grbl.mpos = [-10.0, -10.0, -30.0]

        self.sender.on_grbl_jog('origin')

        self.assertFalse(self.grbl.critical)
        self.assertEqual(self.grbl.mpos, [-120.0, -80.0, -25.0])

    def test_steps_towards_the_switches_stop_short_of_them(self):
        self.grbl.mpos = [-3.0, -3.0, -3.0]
        for direction in ('xplus', 'yplus', 'zplus'):
            self.sender.on_grbl_jog(direction)
        self.assertFalse(self.grbl.critical)
        self.assertEqual(self.grbl.mpos, [-1.0, -1.0, -1.0])

        self.sender.on_grbl_jog('zplus')
        self.assertEqual(self.grbl.mpos[2], -1.0)
        self.assertIn('limit switch already', self.sender.app.inform.text())

    def test_a_jog_after_a_limit_is_not_sent_and_does_not_hang(self):
        self.crash()
        jogs_before = len(self.grbl.sent('$J='))

        started = time.monotonic()
        self.sender.on_grbl_jog('xminus')

        self.assertLess(time.monotonic() - started, 3.0)
        self.assertEqual(len(self.grbl.sent('$J=')), jogs_before)
        self.assertIn('limit switch is pressed: X', self.sender.app.inform.text())

    def test_do_home_after_a_limit(self):
        self.crash()
        self.assertFalse(self.sender.grbl_homed)

        self.assertTrue(self.sender.grbl_homing())

        self.assertIn('\x18', self.grbl.realtime)
        self.assertEqual(self.grbl.state, 'Idle')
        self.assertEqual(self.grbl.mpos, [-1.0, -1.0, -1.0])
        self.assertTrue(self.sender.grbl_homed)

    def test_unlock_after_a_limit(self):
        self.crash()

        self.assertTrue(self.sender.grbl_unlock())

        self.assertIn('\x18', self.grbl.realtime)
        self.assertEqual(self.grbl.state, 'Idle')
        self.assertIn('pressed: X', self.sender.app.inform.text())
        # releasing the switch would raise the alarm again
        self.assertEqual(self.grbl.settings[21], '0')

        # the hard limits stay off while the switch is pressed
        self.sender.on_grbl_status_tick()
        time.sleep(0.3)
        self.sender.on_grbl_status_tick()
        self.assertTrue(self.sender.grbl_limits_suspended)

        # the machine can be jogged off the switch
        self.sender.on_grbl_jog('xminus')
        self.assertEqual(self.grbl.mpos[0], -5.0)
        self.assertFalse(self.grbl.critical)

        # and once it is off, the hard limits are switched on again
        self.assertTrue(wait_for(lambda: self.tick() and self.grbl.settings[21] == '1'))
        self.assertFalse(self.sender.grbl_limits_suspended)

    def tick(self):
        self.sender.on_grbl_status_tick()
        return True

    def test_jogging_off_a_switch_with_the_hard_limits_on_raises_the_alarm_again(self):
        # what the controller does when it is only unlocked with $X; the sender switches the limits off for it
        self.crash()
        self.sender.grbl_soft_reset()
        self.assertTrue(self.sender.send_grbl_command('$X', echo=False, timeout=1.0))
        self.sender.on_grbl_jog('xminus')
        self.assertTrue(self.grbl.critical)

    def test_a_job_that_goes_above_the_top_is_not_sent(self):
        # the work Z zero was set 3 mm below the top, the job lifts to 15 mm
        self.grbl.mpos[2] = -4.0
        self.sender.on_grbl_zero('z')
        self.sender.app.collection = SimpleNamespace(
            get_by_name=lambda name: SimpleNamespace(source_file='G21\nG90\nG00 Z15.0\nG01 Z-1.5 F100\n'))
        lines_before = len(self.grbl.lines)

        self.sender.on_grbl_send_job()

        self.assertFalse(self.sender.grbl_job_running)
        self.assertEqual(len(self.grbl.lines), lines_before)
        self.assertIn('12.000 mm above the top', self.sender.app.inform.text())

    def test_a_job_is_not_sent_in_an_alarm(self):
        self.crash()
        self.sender.app.collection = SimpleNamespace(get_by_name=lambda name: SimpleNamespace(source_file='G0 X1\n'))
        lines_before = len(self.grbl.lines)

        self.sender.on_grbl_send_job()

        self.assertFalse(self.sender.grbl_job_running)
        self.assertEqual(len(self.grbl.lines), lines_before)
        self.assertIn('Do Home', self.sender.app.inform.text())


class TestPanelState(unittest.TestCase):
    def setUp(self):
        self.sender = make_sender(FakeGrbl())
        self.ui = self.sender.ui

    def manual_controls(self):
        return [self.ui.jog_wdg, self.ui.zero_axs_wdg, self.ui.probe_z_button, self.ui.grbl_unlock_button,
                self.ui.grbl_command_entry,
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

        self.assertTrue(self.sender.grbl_unlock())
        # a probe alarm is cleared without a reset, which would throw away the job
        self.assertNotIn('\x18', self.grbl.realtime)
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
