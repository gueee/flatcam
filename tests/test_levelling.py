#!/usr/bin/env python
"""
Tests for the autolevelling height interpolation, for the GCode Z compensation and for the helpers of the GRBL
sender.

Run: python tests/test_levelling.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from shapely import Point, box

from appPlugins.ToolLevelling import ToolLevelling


class MockRadio:
    def __init__(self, value):
        self.value = value

    def get_value(self):
        return self.value


class MockUI:
    """Only the widgets that the interpolation and the GCode rewrite read."""

    def __init__(self, method='v', rows=2, columns=2):
        self.al_method_radio = MockRadio(method)
        self.al_rows_entry = MockRadio(rows)
        self.al_columns_entry = MockRadio(columns)


def get_levelling_tool(method='v', rows=2, columns=2):
    """
    A ToolLevelling that is not connected to the application. Creating a real one needs the GUI, the canvas and a
    project, none of which the height math uses.
    """
    tool = ToolLevelling.__new__(ToolLevelling)
    tool.ui = MockUI(method=method, rows=rows, columns=columns)
    tool.al_voronoi_geo_storage = {}
    tool.al_bilinear_geo_storage = []
    return tool


class TestBilinearInterpolation(unittest.TestCase):
    """A 2x2 grid over the unit square with a different height in each corner."""

    def setUp(self):
        self.tool = get_levelling_tool(method='b', rows=2, columns=2)
        # stored row by row, starting with the lower left corner
        self.tool.al_bilinear_geo_storage = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 1.0),
            (0.0, 1.0, 2.0),
            (1.0, 1.0, 3.0),
        ]

    def test_corners(self):
        self.assertAlmostEqual(self.tool.autolevell_bilinear(0.0, 0.0), 0.0)
        self.assertAlmostEqual(self.tool.autolevell_bilinear(1.0, 0.0), 1.0)
        self.assertAlmostEqual(self.tool.autolevell_bilinear(0.0, 1.0), 2.0)
        self.assertAlmostEqual(self.tool.autolevell_bilinear(1.0, 1.0), 3.0)

    def test_center(self):
        self.assertAlmostEqual(self.tool.autolevell_bilinear(0.5, 0.5), 1.5)

    def test_edge_midpoints(self):
        self.assertAlmostEqual(self.tool.autolevell_bilinear(0.5, 0.0), 0.5)
        self.assertAlmostEqual(self.tool.autolevell_bilinear(0.0, 0.5), 1.0)

    def test_quarter_point(self):
        # 0.25 of the way in X, 0.75 in Y
        self.assertAlmostEqual(self.tool.autolevell_bilinear(0.25, 0.75), 1.75)

    def test_three_by_three_grid(self):
        tool = get_levelling_tool(method='b', rows=3, columns=3)
        # a surface that rises only along X, in two steps
        tool.al_bilinear_geo_storage = [
            (0.0, 0.0, 0.0), (1.0, 0.0, 1.0), (2.0, 0.0, 3.0),
            (0.0, 1.0, 0.0), (1.0, 1.0, 1.0), (2.0, 1.0, 3.0),
            (0.0, 2.0, 0.0), (1.0, 2.0, 1.0), (2.0, 2.0, 3.0),
        ]
        self.assertAlmostEqual(tool.autolevell_bilinear(0.5, 0.5), 0.5)
        self.assertAlmostEqual(tool.autolevell_bilinear(1.5, 1.5), 2.0)

    def test_incomplete_grid_uses_closest_point(self):
        # the grid is not complete, therefore the closest probe point is used
        tool = get_levelling_tool(method='b', rows=2, columns=2)
        tool.al_bilinear_geo_storage = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 1.0),
            (0.0, 1.0, 2.0),
        ]
        self.assertAlmostEqual(tool.autolevell_bilinear(0.9, 0.1), 1.0)


class TestVoronoiInterpolation(unittest.TestCase):
    """Two probe points, each one owning half of a 2 by 1 rectangle."""

    def setUp(self):
        self.tool = get_levelling_tool(method='v')
        self.tool.al_voronoi_geo_storage = {
            1: {'point': Point((0.5, 0.5)), 'geo': box(0.0, 0.0, 1.0, 1.0), 'height': -0.1},
            2: {'point': Point((1.5, 0.5)), 'geo': box(1.0, 0.0, 2.0, 1.0), 'height': 0.2},
        }

    def test_inside_first_cell(self):
        self.assertAlmostEqual(self.tool.autolevell_voronoi(0.25, 0.75), -0.1)

    def test_inside_second_cell(self):
        self.assertAlmostEqual(self.tool.autolevell_voronoi(1.75, 0.25), 0.2)

    def test_outside_all_cells_uses_closest_point(self):
        self.assertAlmostEqual(self.tool.autolevell_voronoi(-5.0, 0.5), -0.1)
        self.assertAlmostEqual(self.tool.autolevell_voronoi(7.0, 0.5), 0.2)

    def test_without_voronoi_diagram_uses_closest_point(self):
        for val in self.tool.al_voronoi_geo_storage.values():
            val['geo'] = None
        self.assertAlmostEqual(self.tool.autolevell_voronoi(1.4, 0.5), 0.2)

    def test_no_probe_points(self):
        self.tool.al_voronoi_geo_storage = {}
        self.assertAlmostEqual(self.tool.autolevell_voronoi(1.0, 1.0), 0.0)


class TestGcodeCompensation(unittest.TestCase):
    """A flat surface raised by 0.1 shifts every Z by 0.1."""

    def setUp(self):
        self.tool = get_levelling_tool(method='v')
        self.tool.al_voronoi_geo_storage = {
            1: {'point': Point((0.0, 0.0)), 'geo': box(-100.0, -100.0, 100.0, 100.0), 'height': 0.1},
        }

    def levell(self, gcode):
        return self.tool.autolevell_gcode_block(gcode, dec=4)

    def test_existing_z_is_offset(self):
        gcode = 'G00 Z2.0000\nG00 X1.0000 Y2.0000\nG01 Z-0.0500'
        levelled = self.levell(gcode).splitlines()
        self.assertEqual(levelled[1], 'G00 X1.0000 Y2.0000 Z2.1000')
        self.assertEqual(levelled[2], 'G01 Z0.0500')

    def test_z_is_added_to_a_cutting_move(self):
        gcode = 'G00 X0.0000 Y0.0000\nG01 Z-1.0000\nG01 X5.0000 Y0.0000'
        levelled = self.levell(gcode).splitlines()
        self.assertEqual(levelled[2], 'G01 X5.0000 Y0.0000 Z-0.9000')

    def test_feedrate_line_is_untouched(self):
        gcode = 'G00 X1.0000 Y1.0000\nG01 Z-0.5000\nG01 F120.00\nG01 X2.0000 Y1.0000'
        levelled = self.levell(gcode).splitlines()
        self.assertEqual(levelled[2], 'G01 F120.00')

    def test_non_motion_lines_are_untouched(self):
        gcode = '(GCODE GENERATED BY FLATCAM)\nG21\nG90\nM03 S1000\nG4 P0.5\nM05\nM02'
        self.assertEqual(self.levell(gcode), gcode)

    def test_probing_command_is_untouched(self):
        gcode = 'G00 X1.0000 Y1.0000\nG01 Z-1.0000\nG38.2 Z-2.0000 F60.00'
        levelled = self.levell(gcode).splitlines()
        self.assertEqual(levelled[2], 'G38.2 Z-2.0000 F60.00')

    def test_move_before_the_first_z_is_untouched(self):
        # the programmed depth is not known yet
        gcode = 'G00 X1.0000 Y1.0000'
        self.assertEqual(self.levell(gcode), gcode)

    def test_lift_before_the_first_xy_is_untouched(self):
        # the position is not known yet
        gcode = 'G00 Z2.0000\nG00 X1.0000 Y1.0000'
        levelled = self.levell(gcode).splitlines()
        self.assertEqual(levelled[0], 'G00 Z2.0000')
        self.assertEqual(levelled[1], 'G00 X1.0000 Y1.0000 Z2.1000')

    def test_arc_keeps_its_own_z_and_gets_none_added(self):
        gcode = 'G00 X0.0000 Y0.0000\nG01 Z-1.0000\nG02 X1.0000 Y1.0000 I1.0000 J0.0000\nG02 X2.0000 Z-1.0000'
        levelled = self.levell(gcode).splitlines()
        self.assertEqual(levelled[2], 'G02 X1.0000 Y1.0000 I1.0000 J0.0000')
        self.assertEqual(levelled[3], 'G02 X2.0000 Z-0.9000')

    def test_comment_at_the_end_of_a_line_is_kept(self):
        gcode = 'G00 X1.0000 Y1.0000\nG01 Z-0.2000 (plunge)'
        levelled = self.levell(gcode).splitlines()
        self.assertEqual(levelled[1], 'G01 Z-0.1000 (plunge)')

    def test_decimals_are_honoured(self):
        gcode = 'G00 X1.0 Y1.0\nG01 Z-0.5'
        levelled = self.tool.autolevell_gcode_block(gcode, dec=2).splitlines()
        self.assertEqual(levelled[1], 'G01 Z-0.40')

    def test_trailing_newline_is_kept(self):
        self.assertTrue(self.levell('G21\nG90\n').endswith('\n'))

    def test_output_of_a_non_gcode_preprocessor_is_untouched(self):
        # a Roland job holds no G words, therefore there is nothing that the levelling can change
        roland = '^DF;!MC1;\nV15.0;Z10.0,20.0,-5.0;\nZ12.0,20.0,-5.0;\n!MC0;'
        self.assertEqual(self.levell(roland), roland)

    def test_a_sloped_surface_changes_the_depth_along_the_path(self):
        tool = get_levelling_tool(method='b', rows=2, columns=2)
        tool.al_bilinear_geo_storage = [
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 1.0),
            (0.0, 10.0, 0.0),
            (10.0, 10.0, 1.0),
        ]
        gcode = 'G00 X0.0000 Y0.0000\nG01 Z-1.0000\nG01 X5.0000 Y0.0000\nG01 X10.0000 Y0.0000'
        levelled = tool.autolevell_gcode_block(gcode, dec=4).splitlines()
        self.assertEqual(levelled[1], 'G01 Z-1.0000')
        self.assertEqual(levelled[2], 'G01 X5.0000 Y0.0000 Z-0.5000')
        self.assertEqual(levelled[3], 'G01 X10.0000 Y0.0000 Z0.0000')


class TestProbeAnswerParsing(unittest.TestCase):
    def setUp(self):
        self.tool = get_levelling_tool()

    def test_successful_probe(self):
        self.assertAlmostEqual(self.tool.parse_probe_answer('[PRB:10.000,20.000,-1.234:1]\nok'), -1.234)

    def test_probe_that_did_not_trigger(self):
        self.assertIsNone(self.tool.parse_probe_answer('[PRB:10.000,20.000,-5.000:0]\nok'))

    def test_answer_without_probe_result(self):
        self.assertIsNone(self.tool.parse_probe_answer('ok'))
        self.assertIsNone(self.tool.parse_probe_answer(''))
        self.assertIsNone(self.tool.parse_probe_answer(None))


class TestStoreProbedPoints(unittest.TestCase):
    def test_both_storages_are_filled(self):
        tool = get_levelling_tool()
        cell = box(0.0, 0.0, 1.0, 1.0)
        # the probe points were added before the probing, with a Voronoi diagram already calculated
        tool.al_voronoi_geo_storage = {
            1: {'point': Point((0.25, 0.25)), 'geo': cell, 'height': 0.0},
            2: {'point': Point((0.75, 0.75)), 'geo': cell, 'height': 0.0},
        }

        tool.store_probed_points([(0.25, 0.25, -0.05), (0.75, 0.75, 0.05)])

        self.assertEqual(tool.al_bilinear_geo_storage, [(0.25, 0.25, -0.05), (0.75, 0.75, 0.05)])
        self.assertAlmostEqual(tool.al_voronoi_geo_storage[1]['height'], -0.05)
        self.assertAlmostEqual(tool.al_voronoi_geo_storage[2]['height'], 0.05)
        # the Voronoi cells are not lost
        self.assertIs(tool.al_voronoi_geo_storage[1]['geo'], cell)

    def test_points_without_a_prior_storage(self):
        tool = get_levelling_tool()
        tool.store_probed_points([(1.0, 2.0, 0.3)])
        self.assertAlmostEqual(tool.al_voronoi_geo_storage[1]['point'].x, 1.0)
        self.assertAlmostEqual(tool.al_voronoi_geo_storage[1]['height'], 0.3)
        self.assertIsNone(tool.al_voronoi_geo_storage[1]['geo'])


class TestGrblPrepareJob(unittest.TestCase):
    """What of a GCode program is sent to a GRBL controller and what is not."""

    def test_empty_lines_and_comments_are_not_sent(self):
        gcode = "G21\n\n(a plain comment)\n; another one\nG90\n"
        self.assertEqual([line for line, _m, _p in ToolLevelling.grbl_prepare_job(gcode)], ['G21', 'G90'])

    def test_inline_comment_is_taken_out_of_the_line(self):
        prepared = ToolLevelling.grbl_prepare_job("G00 X1.0 Y2.0 (rapid) ; trailing\n")
        self.assertEqual(prepared[0][0], 'G00 X1.0 Y2.0')

    def test_message_is_attached_to_the_line_that_follows_it(self):
        gcode = "T1\n(MSG, Change to Tool Dia = 0.8)\nM0\nG00 Z2.0\n"
        prepared = ToolLevelling.grbl_prepare_job(gcode)

        self.assertEqual([line for line, _m, _p in prepared], ['T1', 'M0', 'G00 Z2.0'])
        self.assertEqual(prepared[1][1], 'Change to Tool Dia = 0.8')
        # the message belongs to one line only
        self.assertIsNone(prepared[0][1])
        self.assertIsNone(prepared[2][1])

    def test_message_on_the_same_line_as_a_command(self):
        prepared = ToolLevelling.grbl_prepare_job("M0 (MSG, Mount the probe)\n")
        self.assertEqual(prepared[0][0], 'M0')
        self.assertEqual(prepared[0][1], 'Mount the probe')

    def test_m6_is_taken_out_but_the_rest_of_the_line_is_kept(self):
        prepared = ToolLevelling.grbl_prepare_job("T1\nM6\nT2 M06\n")
        self.assertEqual([line for line, _m, _p in prepared], ['T1', 'T2'])

    def test_a_pause_is_recognized(self):
        prepared = ToolLevelling.grbl_prepare_job("M0\nM01\nM5\nM30\nM09\nG01 X1.0\n")
        self.assertEqual([(line, pauses) for line, _m, pauses in prepared],
                         [('M0', True), ('M01', True), ('M5', False), ('M30', False),
                          ('M09', False), ('G01 X1.0', False)])


class TestGrblResumePreamble(unittest.TestCase):
    """The state that is restored before a stopped job is resumed."""

    def test_the_last_command_of_every_modal_group_is_restored(self):
        sent = ['G20', 'G21', 'G91', 'G90', 'G93', 'G94', 'G17', 'G55', 'G54']
        self.assertEqual(ToolLevelling.grbl_resume_preamble(sent), ['G21', 'G90', 'G94', 'G17', 'G54'])

    def test_feedrate_spindle_and_lift(self):
        sent = ['G21', 'G90', 'M03 S12000', 'G00 Z2.0000', 'G01 Z-0.0500 F120.0', 'G01 X5.0000 Y0.0000']
        self.assertEqual(ToolLevelling.grbl_resume_preamble(sent),
                         ['G21', 'G90', 'F120.0', 'M03 S12000', 'G00 Z2.0'])

    def test_a_stopped_spindle_is_not_started_again(self):
        sent = ['G21', 'M03 S12000', 'G01 Z-0.05 F120.0', 'M05', 'G00 Z15.0']
        self.assertEqual(ToolLevelling.grbl_resume_preamble(sent), ['G21', 'F120.0', 'G00 Z15.0'])

    def test_the_lift_goes_to_the_highest_z_of_the_skipped_lines(self):
        sent = ['G00 Z2.0', 'G01 Z-0.05', 'G00 Z15.0', 'G01 Z-0.1']
        self.assertIn('G00 Z15.0', ToolLevelling.grbl_resume_preamble(sent))

    def test_nothing_to_restore(self):
        self.assertEqual(ToolLevelling.grbl_resume_preamble([]), [])

    def test_a_job_that_only_moves_in_xy_gets_no_lift(self):
        self.assertEqual(ToolLevelling.grbl_resume_preamble(['G21', 'G00 X1.0 Y2.0']), ['G21'])


class TestGrblExtractMessage(unittest.TestCase):
    def test_a_message_is_found(self):
        self.assertEqual(ToolLevelling.grbl_extract_message('(MSG, Change the tool)'), 'Change the tool')

    def test_the_comma_and_the_spacing_are_optional(self):
        self.assertEqual(ToolLevelling.grbl_extract_message('( msg Change the tool )'), 'Change the tool')

    def test_a_plain_comment_holds_no_message(self):
        self.assertIsNone(ToolLevelling.grbl_extract_message('(TOOL DIAMETER: 0.8mm)'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
