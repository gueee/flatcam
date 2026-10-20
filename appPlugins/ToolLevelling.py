# ##########################################################
# FlatCAM Evo: 2D Post-processing for Manufacturing        #
# File by:  Marius Adrian Stanciu (c)                      #
# Date:     11/12/2020                                     #
# License:  MIT Licence                                    #
# ##########################################################

from PyQt6 import QtWidgets, QtCore, QtGui
from PyQt6.QtCore import Qt
from appTool import AppTool
from appGUI.GUIElements import VerticalScrollArea, FCLabel, FCButton, FCFrame, GLay, FCComboBox, FCCheckBox, \
    FCJog, RadioSet, FCDoubleSpinner, FCSpinner, FCFileSaveDialog, FCDetachableTab, FCTable, \
    FCZeroAxes, FCSliderWithDoubleSpinner, FCEntry, RotatedToolButton

import logging
from copy import deepcopy
import re
import sys

from shapely import Point, MultiPoint, MultiPolygon, box
from shapely.ops import unary_union
from shapely.affinity import translate
from datetime import datetime as dt

import gettext
import appTranslation as fcTranslate
import builtins

from appObjects.AppObjectTemplate import ObjectDeleted
from appGUI.VisPyVisuals import *
from appEditors.appTextEditor import AppTextEditor

from camlib import CNCjob

import contextlib
import time
import threading
import serial
import glob
import random
from io import StringIO

from matplotlib.backend_bases import KeyEvent as mpl_key_event

# try:
#     from foronoi import Voronoi
#     from foronoi import Polygon as voronoi_poly
#     VORONOI_ENABLED = True
# except Exception:
#     try:
#         from shapely.ops import voronoi_diagram
#         VORONOI_ENABLED = True
#         # from appCommon.Common import voronoi_diagram
#     except Exception:
#         VORONOI_ENABLED = False
try:
    from shapely.ops import voronoi_diagram
    VORONOI_ENABLED = True
    # from appCommon.Common import voronoi_diagram
except Exception:
    VORONOI_ENABLED = False

fcTranslate.apply_language('strings')
if '_' not in builtins.__dict__:
    _ = gettext.gettext

log = logging.getLogger('base')


class ToolLevelling(CNCjob, AppTool):
    build_al_table_sig = QtCore.pyqtSignal()
    grbl_job_progress_sig = QtCore.pyqtSignal(str)
    grbl_job_finished_sig = QtCore.pyqtSignal()
    # a job stopped at a program pause and waits for the operator
    grbl_job_paused_sig = QtCore.pyqtSignal(str)
    # a command that ran in the background, a homing or a probe, is done with the controller
    grbl_busy_done_sig = QtCore.pyqtSignal()

    # a motion command: G0, G1, G2, G3 and their zero padded forms. It will not match G20, G21, G38.2 and such.
    motion_re = re.compile(r"G0*([0-3])(?!\d)", re.IGNORECASE)
    # a X, Y or Z coordinate word
    coord_re = re.compile(r"([XYZ])\s*([-+]?\d*\.?\d+)", re.IGNORECASE)
    # the answer of the GRBL controller to a probing command: [PRB:0.000,0.000,-1.234:1]
    prb_re = re.compile(r"\[PRB:\s*([-+]?[\d.]+)\s*,\s*([-+]?[\d.]+)\s*,\s*([-+]?[\d.]+)\s*:\s*(\d)]")
    # a comment; GRBL ignores them but they would still take up room in its receive buffer
    comment_re = re.compile(r"\(.*?\)", re.DOTALL)
    # the text that a preprocessor addresses to the operator: (MSG, Change to Tool Dia = 0.8)
    msg_re = re.compile(r"\(\s*MSG\s*,?\s*(.*?)\s*\)", re.IGNORECASE | re.DOTALL)
    # the tool change word; GRBL does not support it and answers a line that holds it with error:20
    m6_re = re.compile(r"M0*6\b", re.IGNORECASE)
    # a program pause: M0 and M1 and their zero padded forms
    pause_re = re.compile(r"M0*[01]\b", re.IGNORECASE)
    # the word that sets the feed rate
    feed_re = re.compile(r"F\s*([-+]?\d*\.?\d+)", re.IGNORECASE)
    # the word that sets the spindle speed
    speed_re = re.compile(r"S\s*(\d*\.?\d+)", re.IGNORECASE)
    # the state of the spindle: M3 and M4 start it, M5 stops it
    spindle_re = re.compile(r"M0*([345])\b", re.IGNORECASE)
    # the modal commands whose last occurrence has to be restored before a job is resumed
    modal_groups = (
        ('G20', 'G21'),                                 # units
        ('G90', 'G91'),                                 # distance mode
        ('G93', 'G94'),                                 # feed rate mode
        ('G17', 'G18', 'G19'),                          # plane selection
        ('G54', 'G55', 'G56', 'G57', 'G58', 'G59'),     # work coordinate system
    )
    # GRBL has a 128 byte serial receive buffer of which one slot is always left empty
    grbl_rx_buffer = 127
    # the farthest a probed height may be from the work Z zero, in mm; the warp of a board is a fraction of it
    al_max_height = 2.0

    def __init__(self, app):
        self.app = app
        self.decimals = self.app.decimals

        AppTool.__init__(self, app)
        CNCjob.__init__(self, steps_per_circle=self.app.options["cncjob_steps_per_circle"], app=app)

        # updated in the self.set_tool_ui()
        self.form_fields = {}

        self.first_click = False
        self.cursor_pos = None

        # if mouse is dragging set the object True
        self.mouse_is_dragging = False

        # if mouse events are bound to local methods
        self.mouse_events_connected = False

        # event handlers references
        self.kp = None
        self.mm = None
        self.mr = None

        self.probing_gcode_text = ''
        self.grbl_probe_result = ''

        '''
        dictionary of dictionaries to store the information's for the autolevelling
        format when using Voronoi diagram:
        {
            id: {
                    'point': Shapely Point
                    'geo': Shapely Polygon from Voronoi diagram,
                    'height': float
                }
        }
        '''
        self.al_voronoi_geo_storage = {}

        '''
        list of (x, y, x) tuples to store the information's for the autolevelling
        format when using bilinear interpolation:
        [(x0, y0, z0), (x1, y1, z1), ...]
        '''
        self.al_bilinear_geo_storage = []

        self.solid_geo = None
        self.grbl_ser_port = None

        # set when the user stops a job that is being sent to the GRBL controller
        self.grbl_job_stop = False
        # set while a job waits for the operator to act on a program pause, a tool change for instance
        self.grbl_job_waiting = False
        # the lines of the job that is being sent, kept so that a job that did not finish can be resumed
        self.grbl_job_lines = []
        # the number of the last line of the job that the controller acknowledged
        self.grbl_job_last_ack = 0
        # an answer of the controller that was read before all of it arrived
        self.grbl_rx_partial = b''
        # how long to wait for the controller to acknowledge a line of the job; a slow move takes a while
        self.grbl_job_line_timeout = 300.0
        # set from the moment a job is sent until the streaming of it ended
        self.grbl_job_running = False
        # whether the last job ended before its last line, so that it can be resumed
        self.grbl_can_resume = False
        # how many commands are talking to the controller in the background; nothing else may read its answers
        self.grbl_busy = 0
        # counts the soft resets; an answer that is waited for will never come once the controller was reset
        self.grbl_resets = 0
        # what the last status report of the controller said: its state, the machine and the work position, the
        # offset of the work origin from the machine origin and the input pins that are active
        self.grbl_status = {'state': None, 'mpos': None, 'wpos': None, 'wco': None, 'pins': ''}
        self.grbl_status_line = ''
        # whether the machine was homed since it was connected; only then the top of its travel is known
        self.grbl_homed = False
        # the settings of the controller, as read with $$ when it was connected
        self.grbl_settings = {}
        # the hard limits are switched off while the machine is jogged off a limit switch, because releasing the
        # switch raises the alarm again otherwise
        self.grbl_limits_suspended = False
        # a status report that was read before all of it arrived
        self.grbl_poll_partial = b''
        # how many are reading answers from the port right now; app.inform runs processEvents(), which can run the
        # status poll in the middle of a read on the GUI thread, and the poll must not take bytes from that answer
        self.grbl_port_readers = 0
        self.grbl_port_readers_lock = threading.Lock()
        # when the last status report came in, to show that the position display is not current
        self.grbl_status_time = 0.0

        # asks the controller for its status, for the position display
        self.grbl_status_timer = QtCore.QTimer()
        self.grbl_status_timer.setInterval(250)
        self.grbl_status_timer.timeout.connect(self.on_grbl_status_tick)

        # the signals of the plugin are connected once here; set_tool_ui() runs every time the panel is built and a
        # connection made there would be made again each time, which runs the slot once more per build
        self.grbl_busy_done_sig.connect(self.on_grbl_busy_done)
        self.build_al_table_sig.connect(self.build_al_table)
        self.grbl_job_progress_sig.connect(self.on_grbl_job_progress)
        self.grbl_job_finished_sig.connect(self.on_grbl_job_finished)
        self.grbl_job_paused_sig.connect(self.on_grbl_job_paused)

        self.probing_shapes = None

        self.gcode_viewer_tab = None

        # store the current selection shape status to be restored after manual adding test points
        self.old_selection_state = self.app.options['global_selection_shape']

        # #############################################################################
        # ######################### Tool GUI ##########################################
        # #############################################################################
        self.ui = LevelUI(layout=self.layout, app=self.app)
        self.pluginName = self.ui.pluginName

        self.connect_signals_at_init()

    def install(self, icon=None, separator=None, **kwargs):
        AppTool.install(self, icon, separator, shortcut='', **kwargs)

    def run(self, toggle=True):
        self.app.defaults.report_usage("ToolLevelling()")

        if toggle:
            # if the splitter is hidden, display it
            if self.app.ui.splitter.sizes()[0] == 0:
                self.app.ui.splitter.setSizes([1, 1])

            # if the Tool Tab is hidden display it, else hide it but only if the objectName is the same
            found_idx = None
            for idx in range(self.app.ui.notebook.count()):
                if self.app.ui.notebook.widget(idx).objectName() == "plugin_tab":
                    found_idx = idx
                    break
            # show the Tab
            if not found_idx:
                try:
                    self.app.ui.notebook.addTab(self.app.ui.plugin_tab, _("Plugin"))
                except RuntimeError:
                    self.app.ui.plugin_tab = QtWidgets.QWidget()
                    self.app.ui.plugin_tab.setObjectName("plugin_tab")
                    self.app.ui.plugin_tab_layout = QtWidgets.QVBoxLayout(self.app.ui.plugin_tab)
                    self.app.ui.plugin_tab_layout.setContentsMargins(2, 2, 2, 2)

                    self.app.ui.plugin_scroll_area = VerticalScrollArea()
                    self.app.ui.plugin_tab_layout.addWidget(self.app.ui.plugin_scroll_area)
                    self.app.ui.notebook.addTab(self.app.ui.plugin_tab, _("Plugin"))
                # focus on Tool Tab
                self.app.ui.notebook.setCurrentWidget(self.app.ui.plugin_tab)

            try:
                if self.app.ui.plugin_scroll_area.widget().objectName() == self.pluginName and found_idx:
                    # if the Tool Tab is not focused, focus on it
                    if not self.app.ui.notebook.currentWidget() is self.app.ui.plugin_tab:
                        # focus on Tool Tab
                        self.app.ui.notebook.setCurrentWidget(self.app.ui.plugin_tab)
                    else:
                        # else remove the Tool Tab
                        self.app.ui.notebook.setCurrentWidget(self.app.ui.properties_tab)
                        self.app.ui.notebook.removeTab(2)

                        # if there are no objects loaded in the app then hide the Notebook widget
                        if not self.app.collection.get_list():
                            self.app.ui.splitter.setSizes([0, 1])
            except AttributeError:
                pass
        else:
            if self.app.ui.splitter.sizes()[0] == 0:
                self.app.ui.splitter.setSizes([1, 1])

        super().run()
        self.set_tool_ui()

        self.app.ui.notebook.setTabText(2, _("Levelling"))

    def connect_signals_at_init(self):
        self.ui.level.toggled.connect(self.on_level_changed)

        self.ui.avoid_exc_holes_cb.toggled.connect(self.on_avoid_exc_holes)

        self.ui.al_mode_radio.activated_custom.connect(self.on_mode_radio)
        self.ui.al_method_radio.activated_custom.connect(self.on_method_radio)
        self.ui.al_controller_combo.currentIndexChanged.connect(self.on_controller_change)
        self.ui.plot_probing_pts_cb.toggled.connect(self.show_probing_geo)
        # GRBL
        self.ui.com_search_button.clicked.connect(self.on_grbl_search_ports)
        self.ui.add_bd_button.clicked.connect(self.on_grbl_add_baudrate)
        self.ui.del_bd_button.clicked.connect(self.on_grbl_delete_baudrate_grbl)
        self.ui.controller_reset_button.clicked.connect(self.on_grbl_reset)
        self.ui.com_connect_button.clicked.connect(self.on_grbl_connect)
        self.ui.grbl_send_button.clicked.connect(self.on_grbl_send_command)
        self.ui.grbl_command_entry.returnPressed.connect(self.on_grbl_send_command)

        # Jog
        self.ui.jog_wdg.jog_up_button.clicked.connect(lambda: self.on_grbl_jog(direction='yplus'))
        self.ui.jog_wdg.jog_down_button.clicked.connect(lambda: self.on_grbl_jog(direction='yminus'))
        self.ui.jog_wdg.jog_right_button.clicked.connect(lambda: self.on_grbl_jog(direction='xplus'))
        self.ui.jog_wdg.jog_left_button.clicked.connect(lambda: self.on_grbl_jog(direction='xminus'))
        self.ui.jog_wdg.jog_z_up_button.clicked.connect(lambda: self.on_grbl_jog(direction='zplus'))
        self.ui.jog_wdg.jog_z_down_button.clicked.connect(lambda: self.on_grbl_jog(direction='zminus'))
        self.ui.jog_wdg.jog_origin_button.clicked.connect(lambda: self.on_grbl_jog(direction='origin'))

        # Zero
        self.ui.zero_axs_wdg.grbl_zerox_button.clicked.connect(lambda: self.on_grbl_zero(axis='x'))
        self.ui.zero_axs_wdg.grbl_zeroy_button.clicked.connect(lambda: self.on_grbl_zero(axis='y'))
        self.ui.zero_axs_wdg.grbl_zeroz_button.clicked.connect(lambda: self.on_grbl_zero(axis='z'))
        self.ui.zero_axs_wdg.grbl_zero_all_button.clicked.connect(lambda: self.on_grbl_zero(axis='all'))
        self.ui.zero_axs_wdg.grbl_homing_button.clicked.connect(self.on_grbl_homing)
        self.ui.probe_z_button.clicked.connect(self.on_grbl_probe_z)
        self.ui.grbl_unlock_button.clicked.connect(self.on_grbl_unlock)

        # Sender
        self.ui.grbl_report_button.clicked.connect(self.on_grbl_report)
        self.ui.grbl_get_param_button.clicked.connect(
            lambda: self.on_grbl_get_parameter(param=self.ui.grbl_parameter_entry.get_value()))
        self.ui.view_h_gcode_button.clicked.connect(self.on_edit_probing_gcode)
        self.ui.h_gcode_button.clicked.connect(self.on_save_probing_gcode)
        self.ui.import_heights_button.clicked.connect(self.on_import_height_map)
        self.ui.pause_resume_button.clicked.connect(self.on_grbl_pause_resume)
        self.ui.grbl_get_heightmap_button.clicked.connect(self.on_grbl_autolevel)
        self.ui.grbl_save_height_map_button.clicked.connect(self.on_grbl_heightmap_save)
        # the lambda keeps the checked state that clicked() carries out of the start line parameter
        self.ui.grbl_send_job_button.clicked.connect(lambda: self.on_grbl_send_job())
        self.ui.grbl_stop_job_button.clicked.connect(self.on_grbl_stop_job)
        self.ui.grbl_continue_button.clicked.connect(self.on_grbl_continue_job)
        self.ui.grbl_resume_job_button.clicked.connect(self.on_grbl_resume_job)

        # When object selection on canvas change
        self.app.proj_selection_changed.connect(self.on_object_selection_changed)

        # Reset Tool
        self.ui.reset_button.clicked.connect(self.set_tool_ui)
        # Cleanup on Graceful exit (CTRL+ALT+X combo key)
        self.app.cleanup.connect(self.set_tool_ui)

    def set_tool_ui(self):
        self.units = self.app.app_units.upper()

        self.clear_ui(self.layout)
        self.ui = LevelUI(layout=self.layout, app=self.app)
        self.pluginName = self.ui.pluginName
        self.connect_signals_at_init()

        # try to select in the CNCJob combobox the active object
        try:
            selected_obj = self.app.collection.get_active()
            if selected_obj.kind == 'cncjob':
                current_name = selected_obj.obj_options['name']
                self.ui.object_combo.set_value(current_name)
        except Exception:
            pass

        loaded_obj = self.app.collection.get_by_name(self.ui.object_combo.get_value())
        if loaded_obj and loaded_obj.kind == 'cncjob':
            name = loaded_obj.obj_options['name']
        else:
            name = ''

        # Shapes container for the Voronoi cells in Autolevelling
        if self.app.use_3d_engine:
            self.probing_shapes = ShapeCollection(parent=self.app.plotcanvas.view.scene, layers=1, pool=self.app.pool)
        else:
            from appGUI.PlotCanvasLegacy import ShapeCollectionLegacy
            self.probing_shapes = ShapeCollectionLegacy(obj=self, app=self.app, name=name + "_probing_shapes")

        self.form_fields.update({
            "tools_al_probe_tip_dia":           self.ui.probe_tip_dia_entry,
            "tools_al_travel_z":                self.ui.ptravelz_entry,
            "tools_al_probe_depth":             self.ui.pdepth_entry,
            "tools_al_probe_fr":                self.ui.feedrate_probe_entry,
            "tools_al_controller":              self.ui.al_controller_combo,
            "tools_al_method":                  self.ui.al_method_radio,
            "tools_al_mode":                    self.ui.al_mode_radio,
            "tools_al_avoid_exc_holes_size":    self.ui.avoid_exc_holes_size_entry,
            "tools_al_rows":                    self.ui.al_rows_entry,
            "tools_al_columns":                 self.ui.al_columns_entry,
            "tools_al_grbl_jog_step":           self.ui.jog_step_entry,
            "tools_al_grbl_jog_fr":             self.ui.jog_fr_entry,
            "tools_al_grbl_probe_travel":       self.ui.probe_travel_entry,
            "tools_al_grbl_probe_fr":           self.ui.probe_fr_entry,
            "tools_al_grbl_plate_thickness":    self.ui.plate_thickness_entry,
        })

        # Fill Form fields
        self.to_form()
        self.on_controller_change_alter_ui()

        self.ui.plot_probing_pts_cb.set_value(self.app.options["tools_al_plot_points"])
        self.ui.avoid_exc_holes_cb.set_value(self.app.options["tools_al_avoid_exc_holes"])

        self.ui.al_probe_points_table.setRowCount(0)
        self.ui.al_probe_points_table.resizeColumnsToContents()
        self.ui.al_probe_points_table.resizeRowsToContents()
        v_header = self.ui.al_probe_points_table.verticalHeader()
        v_header.hide()
        self.ui.al_probe_points_table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        h_header = self.ui.al_probe_points_table.horizontalHeader()
        h_header.setMinimumSectionSize(10)
        h_header.setDefaultSectionSize(70)
        h_header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeMode.Fixed)
        h_header.resizeSection(0, 20)
        h_header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeMode.Stretch)
        h_header.setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeMode.ResizeToContents)

        self.ui.al_probe_points_table.setMinimumHeight(self.ui.al_probe_points_table.getHeight())
        self.ui.al_probe_points_table.setMaximumHeight(self.ui.al_probe_points_table.getHeight())

        # Set initial UI
        self.ui.al_rows_entry.setDisabled(True)
        self.ui.al_rows_label.setDisabled(True)
        self.ui.al_columns_entry.setDisabled(True)
        self.ui.al_columns_label.setDisabled(True)
        self.ui.al_method_lbl.setDisabled(True)
        self.ui.al_method_radio.set_value('v')
        self.ui.al_method_radio.setDisabled(True)

        # Show/Hide Advanced Options
        app_mode = self.app.options["global_app_level"]
        self.change_level(app_mode)

        try:
            self.ui.object_combo.currentIndexChanged.disconnect()
        except (AttributeError, TypeError):
            pass
        self.ui.object_combo.currentIndexChanged.connect(self.on_object_changed)

        self.build_tool_ui()

        if self.can_be_levelled(loaded_obj):
            self.on_levelling_available(True)
            self.ui.al_mode_radio.set_value(loaded_obj.obj_options['tools_al_mode'])
            self.on_controller_change()

            self.on_mode_radio(val=loaded_obj.obj_options['tools_al_mode'])
            self.on_method_radio(val=loaded_obj.obj_options['tools_al_method'])
        else:
            self.on_levelling_available(False)

        self.on_avoid_exc_holes(self.app.options["tools_al_avoid_exc_holes"])

        # the connection outlives the panel; a panel that was built again has to show it
        if self.grbl_is_connected():
            self.grbl_set_connected_ui(True)
            self.grbl_refresh_controls()

    def on_object_changed(self):

        # load the object
        obj_name = self.ui.object_combo.currentText()

        # Get source object.
        try:
            target_obj = self.app.collection.get_by_name(obj_name)
        except Exception:
            self.app.inform.emit('[ERROR_NOTCL] %s: %s' % (_("Could not retrieve object"), str(obj_name)))
            return

        if self.can_be_levelled(target_obj):
            self.on_levelling_available(True)

            # Shapes container for the Voronoi cells in Autolevelling
            if self.app.use_3d_engine:
                self.probing_shapes = ShapeCollection(parent=self.app.plotcanvas.view.scene, layers=1,
                                                      pool=self.app.pool)
            else:
                self.probing_shapes = ShapeCollectionLegacy(obj=self, app=self.app, name=obj_name + "_probing_shapes")
        else:
            self.on_levelling_available(False)

    @staticmethod
    def can_be_levelled(cnc_obj):
        """
        Whether the GCode of a CNCJob object can have the probed heights applied over it. Only a job that was
        made out of a Geometry object with segmented GCode has moves short enough to be corrected.

        :param cnc_obj: the object to check
        :type cnc_obj:  class
        :return:        True when the job can be levelled
        :rtype:         bool
        """
        return cnc_obj is not None and cnc_obj.is_segmented_gcode is True and \
            cnc_obj.obj_options["type"] == 'Geometry'

    def on_levelling_available(self, available):
        """
        Enable the parts of the panel that only make sense for a job that can be levelled. The controller
        section is not one of them: connecting to the machine, jogging it and sending a job work for any kind
        of CNC job.

        :param available:   whether the selected object can be levelled
        :type available:    bool
        """
        self.ui.al_frame.setDisabled(not available)
        self.ui.al_frame2.setDisabled(not available)

        if available:
            self.build_al_table()
        else:
            # probe points left over from an earlier object must not make these look usable
            self.ui.grbl_get_heightmap_button.setDisabled(True)
            self.ui.grbl_save_height_map_button.setDisabled(True)
            self.ui.h_gcode_button.setDisabled(True)
            self.ui.view_h_gcode_button.setDisabled(True)

    def on_object_selection_changed(self, current, previous):
        found_idx = None
        for tab_idx in range(self.app.ui.notebook.count()):
            if self.app.ui.notebook.tabText(tab_idx) == self.ui.pluginName:
                found_idx = True
                break

        if found_idx:
            try:
                sel_obj = current.indexes()[0].internalPointer().obj
                name = sel_obj.obj_options['name']
                kind = sel_obj.kind

                if kind == 'cncjob':
                    self.ui.object_combo.set_value(name)
            except IndexError:
                pass

    def change_level(self, level):
        """

        :param level:   application level: either 'b' or 'a'
        :type level:    str
        :return:
        """

        if level == 'a':
            self.ui.level.setChecked(True)
        else:
            self.ui.level.setChecked(False)
        self.on_level_changed(self.ui.level.isChecked())

    def on_level_changed(self, checked):
        if not checked:
            self.ui.level.setText('%s' % _('Beginner'))
            self.ui.level.setStyleSheet("""
                                        QToolButton
                                        {
                                            color: green;
                                        }
                                        """)

            self.ui.al_title.hide()
            self.ui.show_al_table.hide()
            self.ui.al_probe_points_table.hide()

            # Context Menu section
            # self.ui.al_probe_points_table.removeContextMenu()
        else:
            self.ui.level.setText('%s' % _('Advanced'))
            self.ui.level.setStyleSheet("""
                                        QToolButton
                                        {
                                            color: red;
                                        }
                                        """)

            self.ui.al_title.show()
            self.ui.show_al_table.show()
            if self.ui.show_al_table.get_value():
                self.ui.al_probe_points_table.show()

            # Context Menu section
            # self.ui.al_probe_points_table.setupContextMenu()

    def build_tool_ui(self):
        self.ui_disconnect()

        self.build_al_table()

        self.ui_connect()

    def build_al_table(self):
        tool_idx = 0

        n = len(self.al_voronoi_geo_storage)
        self.ui.al_probe_points_table.setRowCount(n)

        for id_key, value in self.al_voronoi_geo_storage.items():
            tool_idx += 1
            row_no = tool_idx - 1

            t_id = QtWidgets.QTableWidgetItem('%d' % int(tool_idx))
            point = value.get('point')
            if point is None or point.is_empty:
                # a Voronoi diagram that could not be made leaves points without coordinates
                coords_item = QtWidgets.QTableWidgetItem('-')
            else:
                xy_coords = (self.app.dec_format(point.x, dec=self.app.decimals),
                             self.app.dec_format(point.y, dec=self.app.decimals))
                coords_item = QtWidgets.QTableWidgetItem(str(xy_coords))
            height = self.app.dec_format(value['height'], dec=self.app.decimals)
            height_item = QtWidgets.QTableWidgetItem(str(height))

            t_id.setFlags(QtCore.Qt.ItemFlag.ItemIsEnabled)
            coords_item.setFlags(QtCore.Qt.ItemFlag.ItemIsEnabled)
            height_item.setFlags(QtCore.Qt.ItemFlag.ItemIsEnabled)

            self.ui.al_probe_points_table.setItem(row_no, 0, t_id)  # Tool name/id
            self.ui.al_probe_points_table.setItem(row_no, 1, coords_item)  # X-Y coords
            self.ui.al_probe_points_table.setItem(row_no, 2, height_item)  # Determined Height

        self.ui.al_probe_points_table.resizeColumnsToContents()
        self.ui.al_probe_points_table.resizeRowsToContents()

        h_header = self.ui.al_probe_points_table.horizontalHeader()
        h_header.setMinimumSectionSize(10)
        h_header.setDefaultSectionSize(70)
        h_header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeMode.Fixed)
        h_header.resizeSection(0, 20)
        h_header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeMode.Stretch)
        h_header.setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeMode.ResizeToContents)

        self.ui.al_probe_points_table.setMinimumHeight(self.ui.al_probe_points_table.getHeight())
        self.ui.al_probe_points_table.setMaximumHeight(self.ui.al_probe_points_table.getHeight())

        if self.ui.al_probe_points_table.model().rowCount():
            self.ui.grbl_get_heightmap_button.setDisabled(False)
            self.ui.grbl_save_height_map_button.setDisabled(False)
            self.ui.h_gcode_button.setDisabled(False)
            self.ui.view_h_gcode_button.setDisabled(False)
        else:
            self.ui.grbl_get_heightmap_button.setDisabled(True)
            self.ui.grbl_save_height_map_button.setDisabled(True)
            self.ui.h_gcode_button.setDisabled(True)
            self.ui.view_h_gcode_button.setDisabled(True)

    def to_form(self, storage=None):
        if storage is None:
            storage = self.app.options

        for k in self.form_fields:
            for option in storage:
                if option.startswith('tools_al_'):
                    if k == option:
                        try:
                            self.form_fields[k].set_value(storage[option])
                        except Exception:
                            # it may fail for form fields found in the tools tables if there are no rows
                            pass

    def on_add_al_probepoints(self):
        # create the solid_geo

        loaded_obj = self.app.collection.get_by_name(self.ui.object_combo.get_value())
        if loaded_obj is None:
            self.app.log.error("ToolLevelling.on_add_al_probepoints() -> No object loaded.")
            return 'fail'

        try:
            self.solid_geo = unary_union([geo['geom'] for geo in loaded_obj.gcode_parsed if geo['kind'][0] == 'C'])
        except TypeError:
            return 'fail'

        # reset al table
        self.ui.al_probe_points_table.setRowCount(0)

        # reset the al dict
        self.al_voronoi_geo_storage.clear()

        if self.ui.al_mode_radio.get_value() == 'grid':
            self.on_add_grid_points()
        else:
            self.on_add_manual_points()

    def check_point_over_excellon(self, pol: Polygon, check: bool) -> MultiPolygon:
        if not check:
            return MultiPolygon()

        fused_geometries = [
            exc_geo
            for obj_in_collection in self.app.collection.get_list()
            if obj_in_collection.kind == 'excellon' and obj_in_collection.obj_options['plot']
            for exc_geo in MultiPolygon(obj_in_collection.solid_geometry).geoms
            if isinstance(exc_geo, Polygon) and pol.intersects(exc_geo)
        ]
        return unary_union(fused_geometries)

    def on_add_grid_points(self):
        check_overlap = self.ui.avoid_exc_holes_cb.get_value()
        avoid_step = self.ui.avoid_exc_holes_size_entry.get_value()
        radius = self.ui.probe_tip_dia_entry.get_value() / 2

        xmin, ymin, xmax, ymax = self.solid_geo.bounds

        width = abs(xmax - xmin)
        height = abs(ymax - ymin)
        cols = self.ui.al_columns_entry.get_value()
        rows = self.ui.al_rows_entry.get_value()

        dx = 0 if cols == 1 else width / (cols - 1)
        dy = 0 if rows == 1 else height / (rows - 1)

        points = []
        new_y = ymin
        for x in range(rows):
            new_x = xmin
            for y in range(cols):
                formatted_point = (
                    self.app.dec_format(new_x, self.app.decimals),
                    self.app.dec_format(new_y, self.app.decimals)
                )

                point_buffered = Point(formatted_point).buffer(radius)
                if self.check_point_over_excellon(pol=point_buffered, check=check_overlap).is_empty:
                    # do not add the point if is already added
                    if formatted_point not in points:
                        points.append(formatted_point)
                        new_x += dx
                        continue

                box_poly: Polygon = box(
                    new_x - dx if (new_x - dx) > xmin else xmin,
                    new_y - dy if (new_y - dy) > ymin else ymin,
                    new_x + dx if (new_x + dx) < xmax else xmax,
                    new_y + dy if (new_y + dy) < ymax else ymax
                )

                increments = [
                    (avoid_step, 0),
                    (avoid_step * -1, 0),
                    (0, avoid_step),
                    (0, avoid_step * -1)
                ]

                for increment in increments:
                    break_for_loop = False
                    while True:
                        # check if the point is within the box
                        formatted_point = (
                            formatted_point[0] + increment[0],
                            formatted_point[1] + increment[1]
                        )
                        point_buffered = Point(formatted_point).buffer(radius)

                        if not box_poly.contains(point_buffered):
                            break

                        # check if the point overlaps an excellon hole
                        if self.check_point_over_excellon(pol=point_buffered, check=check_overlap).is_empty:
                            # do not add the point if it is already added
                            if formatted_point not in points:
                                points.append(formatted_point)
                                break_for_loop = True
                                break
                    if break_for_loop:
                        break
                new_x += dx
            new_y += dy

        pt_id = 0
        vor_pts_list = []
        bl_pts_list = []
        for point in points:
            pt_id += 1
            pt = Point(point)
            vor_pts_list.append(pt)
            bl_pts_list.append((point[0], point[1], 0.0))
            new_dict = {
                'point': pt,
                'geo': None,
                'height': 0.0
            }
            self.al_voronoi_geo_storage[pt_id] = deepcopy(new_dict)

        al_method = self.ui.al_method_radio.get_value()
        if al_method == 'v':
            if VORONOI_ENABLED is True:
                self.generate_voronoi_geometry(pts=vor_pts_list)
                # generate Probing GCode
                self.probing_gcode_text = self.probing_gcode(storage=self.al_voronoi_geo_storage)
            else:
                self.app.inform.emit('[ERROR_NOTCL] %s' % _("Voronoi function can not be loaded.\n"
                                                            "Shapely >= 1.8 is required"))
        else:
            self.generate_bilinear_geometry(pts=bl_pts_list)
            # generate Probing GCode
            self.probing_gcode_text = self.probing_gcode(storage=self.al_bilinear_geo_storage)

        self.build_al_table_sig.emit()
        if self.ui.plot_probing_pts_cb.get_value():
            self.show_probing_geo(state=True, reset=True)
        else:
            # clear probe shapes
            self.plot_probing_geo(None, False)

    def on_add_manual_points(self):
        xmin, ymin, xmax, ymax = self.solid_geo.bounds
        f_probe_pt = Point([xmin, xmin])
        int_keys = [int(k) for k in self.al_voronoi_geo_storage.keys()]
        new_id = max(int_keys) + 1 if int_keys else 1
        new_dict = {
            'point': f_probe_pt,
            'geo': None,
            'height': 0.0
        }
        self.al_voronoi_geo_storage[new_id] = deepcopy(new_dict)

        radius = self.ui.probe_tip_dia_entry.get_value() / 2
        fprobe_pt_buff = f_probe_pt.buffer(radius)

        self.app.inform.emit(_("Click on canvas to add a Probe Point..."))
        self.app.options['global_selection_shape'] = False

        if self.app.use_3d_engine:
            self.app.plotcanvas.graph_event_disconnect('key_press', self.app.ui.keyPressEvent)
            self.app.plotcanvas.graph_event_disconnect('mouse_press', self.app.on_mouse_click_over_plot)
            self.app.plotcanvas.graph_event_disconnect('mouse_release', self.app.on_mouse_click_release_over_plot)
        else:
            self.app.plotcanvas.graph_event_disconnect(self.app.kp)
            self.app.plotcanvas.graph_event_disconnect(self.app.mp)
            self.app.plotcanvas.graph_event_disconnect(self.app.mr)

        self.kp = self.app.plotcanvas.graph_event_connect('key_press', self.on_key_press)
        self.mr = self.app.plotcanvas.graph_event_connect('mouse_release', self.on_mouse_click_release)
        self.mm = self.app.plotcanvas.graph_event_connect('mouse_move', self.on_mouse_move)

        self.mouse_events_connected = True

        self.build_al_table_sig.emit()
        if self.ui.plot_probing_pts_cb.get_value():
            self.show_probing_geo(state=True, reset=True)
        else:
            # clear probe shapes
            self.plot_probing_geo(None, False)

        self.plot_probing_geo(geometry=fprobe_pt_buff, visibility=True, custom_color="#0000FFFA")

    def show_probing_geo(self, state, reset=False):
        self.app.log.debug("ToolLevelling.show_probing_geo() -> %s" % ('cleared' if state is False else 'displayed'))
        if reset:
            self.probing_shapes.clear(update=True)

        points_geo = []
        poly_geo = []

        al_method = self.ui.al_method_radio.get_value()
        radius = self.ui.probe_tip_dia_entry.get_value() / 2

        # voronoi diagram
        if al_method == 'v':
            # create the geometry
            for pt in self.al_voronoi_geo_storage:
                if not self.al_voronoi_geo_storage[pt]['geo']:
                    continue

                p_geo = self.al_voronoi_geo_storage[pt]['point'].buffer(radius)
                s_geo = self.al_voronoi_geo_storage[pt]['geo'].buffer(0.0000001)

                points_geo.append(p_geo)
                poly_geo.append(s_geo)

            if not points_geo and not poly_geo:
                return

            self.plot_probing_geo(geometry=points_geo, visibility=state, custom_color='#000000FF')
            self.plot_probing_geo(geometry=poly_geo, visibility=state)
        # bilinear interpolation
        elif al_method == 'b':
            for pt in self.al_bilinear_geo_storage:

                x_pt = pt[0]
                y_pt = pt[1]
                p_geo = Point([x_pt, y_pt]).buffer(radius)

                if p_geo.is_valid:
                    points_geo.append(p_geo)

            if not points_geo:
                return

            self.plot_probing_geo(geometry=points_geo, visibility=state, custom_color='#000000FF')

    def plot_probing_geo(self, geometry, visibility, custom_color=None):
        if visibility:
            if self.app.use_3d_engine:
                def random_color():
                    r_color = np.random.rand(4)
                    r_color[3] = 0.5
                    return r_color
            else:
                def random_color():
                    while True:
                        r_color = np.random.rand(4)
                        r_color[3] = 0.5

                        new_color = '#'
                        for idx in range(len(r_color)):
                            new_color += '%x' % int(r_color[idx] * 255)
                        # do it until a valid color is generated
                        # a valid color has the # symbol, another 6 chars for the color and the last 2 chars for alpha
                        # for a total of 9 chars
                        if len(new_color) == 9:
                            break
                    return new_color

            try:
                # if self.app.use_3d_engine:
                #     color = "#0000FFFE"
                # else:
                #     color = "#0000FFFE"
                # for sh in points_geo:
                #     self.add_probing_shape(shape=sh, color=color, face_color=color, visible=True)

                edge_color = "#000000FF"

                try:
                    for sh in geometry:
                        if custom_color is None:
                            k = self.add_probing_shape(shape=sh, color=edge_color, face_color=random_color(),
                                                       visible=True)
                        else:
                            k = self.add_probing_shape(shape=sh, color=custom_color, face_color=custom_color,
                                                       visible=True)
                except TypeError:
                    if custom_color is None:
                        self.add_probing_shape(
                            shape=geometry, color=edge_color, face_color=random_color(), visible=True)
                    else:
                        self.add_probing_shape(
                            shape=geometry, color=custom_color, face_color=custom_color, visible=True)

                self.probing_shapes.redraw()
            except (ObjectDeleted, AttributeError) as e:
                self.app.log.error("ToolLevelling.plot_probing_geo() -> %s" % str(e))
                self.probing_shapes.clear(update=True)
            except Exception as e:
                self.app.log.error("CNCJobObject.plot_probing_geo() --> %s" % str(e))
        else:
            self.probing_shapes.clear(update=True)

    def add_probing_shape(self, **kwargs):
        key = self.probing_shapes.add(tolerance=self.drawing_tolerance, layer=0, **kwargs)
        return key

    def generate_voronoi_geometry(self, pts):
        env = self.solid_geo.envelope
        fact = 1 if self.units == 'MM' else 0.039
        env = env.buffer(fact)

        new_pts = deepcopy(pts)
        try:
            pts_union = MultiPoint(pts)
            voronoi_union = voronoi_diagram(geom=pts_union, envelope=env)
        except Exception as e:
            self.app.log.error("CNCJobObject.generate_voronoi_geometry() --> %s" % str(e))
            for pt_index in range(len(pts)):
                new_pts[pt_index] = translate(
                    new_pts[pt_index], random.random() * 1e-09, random.random() * 1e-09)

            pts_union = MultiPoint(new_pts)
            try:
                voronoi_union = voronoi_diagram(geom=pts_union, envelope=env)
            except Exception:
                return

        new_voronoi = []
        for p in voronoi_union.geoms:
            new_voronoi.append(p.intersection(env))

        for pt_key in list(self.al_voronoi_geo_storage.keys()):
            for poly in new_voronoi:
                if self.al_voronoi_geo_storage[pt_key]['point'].within(poly):
                    self.al_voronoi_geo_storage[pt_key]['geo'] = poly

    # def generate_voronoi_geometry_2(self, pts):
    #     env = self.solid_geo.envelope
    #     fact = 1 if self.units == 'MM' else 0.039
    #     env = env.buffer(fact)
    #     env_poly = voronoi_poly(tuple(env.exterior.coords))
    #
    #     new_pts = [[pt.x, pt.y] for pt in pts]
    #     print(new_pts)
    #     print(env_poly)
    #
    #     # Initialize the algorithm
    #     v = Voronoi(env_poly)
    #
    #     # calculate the Voronoi diagram
    #     try:
    #         v.create_diagram(new_pts)
    #     except AttributeError as e:
    #         self.app.log.error("CNCJobObject.generate_voronoi_geometry_2() --> %s" % str(e))
    #         new_pts_2 = []
    #         for pt_index in range(len(new_pts)):
    #             new_pts_2.append([
    #                 new_pts[pt_index][0] + random.random() * 1e-03,
    #                 new_pts[pt_index][1] + random.random() * 1e-03
    #             ])
    #
    #         try:
    #             v.create_diagram(new_pts_2)
    #         except Exception:
    #             print("Didn't work.")
    #             return
    #
    #     new_voronoi = []
    #     for p in v.sites:
    #         # p_coords = [(coord.x, coord.y) for coord in p.get_coordinates()]
    #         p_coords = [(p.x, p.y)]
    #         new_pol = Polygon(p_coords)
    #         new_voronoi.append(new_pol)
    #
    #     new_voronoi = MultiPolygon(new_voronoi)
    #
    #     # new_voronoi = []
    #     # for p in voronoi_union:
    #     #     new_voronoi.append(p.intersection(env))
    #     #
    #     for pt_key in list(self.al_voronoi_geo_storage.keys()):
    #         for poly in new_voronoi:
    #             if self.al_voronoi_geo_storage[pt_key]['point'].within(poly) or \
    #                     self.al_voronoi_geo_storage[pt_key]['point'].intersects(poly):
    #                 self.al_voronoi_geo_storage[pt_key]['geo'] = poly

    def generate_bilinear_geometry(self, pts):
        self.al_bilinear_geo_storage = pts

    def on_mouse_move(self, event):
        """
        Callback for the mouse motion event over the plot.

        :param event: Contains information about the event.
        :return: None
        """

        if self.app.use_3d_engine:
            self.mouse_is_dragging = event.is_dragging
        else:
            self.mouse_is_dragging = self.app.plotcanvas.is_dragging

        # So it can receive key presses but not when the Tcl Shell is active
        if not self.app.ui.shell_dock.isVisible():
            if not self.app.plotcanvas.native.hasFocus():
                self.app.plotcanvas.native.setFocus()

    # To be called after clicking on the plot.
    def on_mouse_click_release(self, event):
        if self.app.use_3d_engine:
            event_pos = event.pos
            right_button = 2
        else:
            event_pos = (event.xdata, event.ydata)
            right_button = 3

        try:
            x = float(event_pos[0])
            y = float(event_pos[1])
        except TypeError:
            return
        event_pos = (x, y)

        # do paint single only for left mouse clicks
        if event.button == 1:
            check_for_exc_hole = self.ui.avoid_exc_holes_cb.get_value()

            pos = self.app.plotcanvas.translate_coords(event_pos)
            # use the snapped position as reference
            snapped_pos = self.app.geo_editor.snap(pos[0], pos[1])

            # do not add the point if is already added
            old_points_coords = [(pt['point'].x, pt['point'].y) for pt in self.al_voronoi_geo_storage.values()]
            if (snapped_pos[0], snapped_pos[1]) in old_points_coords:
                return

            # Clicked Point
            probe_pt = Point(snapped_pos)

            xxmin, yymin, xxmax, yymax = self.solid_geo.bounds
            box_geo = box(xxmin, yymin, xxmax, yymax)
            if not probe_pt.within(box_geo):
                self.app.inform.emit(_("Point is not within the object area. Choose another point."))
                return

            # check if chosen point is within an Excellon drill hole geometry
            if check_for_exc_hole is True:
                for obj_in_collection in self.app.collection.get_list():
                    if obj_in_collection.kind == 'excellon' and obj_in_collection.obj_options['plot'] is True:
                        exc_solid_geometry = MultiPolygon(obj_in_collection.solid_geometry)
                        for exc_geo in exc_solid_geometry.geoms:
                            if probe_pt.within(exc_geo):
                                self.app.inform.emit(_("Point on an Excellon drill hole. Choose another point."))
                                return

            int_keys = [int(k) for k in self.al_voronoi_geo_storage.keys()]
            new_id = max(int_keys) + 1 if int_keys else 1
            new_dict = {
                'point': probe_pt,
                'geo': None,
                'height': 0.0
            }
            self.al_voronoi_geo_storage[new_id] = deepcopy(new_dict)

            # rebuild the al table
            self.build_al_table_sig.emit()

            radius = self.ui.probe_tip_dia_entry.get_value() / 2
            probe_pt_buff = probe_pt.buffer(radius)

            self.plot_probing_geo(geometry=probe_pt_buff, visibility=True, custom_color="#0000FFFA")

            self.app.inform.emit(_("Added a Probe Point... Click again to add another or right click to finish ..."))

        # if RMB then we exit
        elif event.button == right_button and self.mouse_is_dragging is False:
            if self.app.use_3d_engine:
                self.app.plotcanvas.graph_event_disconnect('key_press', self.on_key_press)
                self.app.plotcanvas.graph_event_disconnect('mouse_release', self.on_mouse_click_release)
                self.app.plotcanvas.graph_event_disconnect('mouse_move', self.on_mouse_move)
            else:
                self.app.plotcanvas.graph_event_disconnect(self.kp)
                self.app.plotcanvas.graph_event_disconnect(self.mr)
                self.app.plotcanvas.graph_event_disconnect(self.mm)

            self.app.kp = self.app.plotcanvas.graph_event_connect('key_press', self.app.ui.keyPressEvent)
            self.app.mp = self.app.plotcanvas.graph_event_connect('mouse_press', self.app.on_mouse_click_over_plot)
            self.app.mr = self.app.plotcanvas.graph_event_connect('mouse_release',
                                                                  self.app.on_mouse_click_release_over_plot)

            # signal that the mouse events are disconnected from local methods
            self.mouse_events_connected = False

            # restore selection
            self.app.options['global_selection_shape'] = self.old_selection_state

            self.app.inform.emit(_("Finished adding Probe Points..."))

            al_method = self.ui.al_method_radio.get_value()
            if al_method == 'v':
                if VORONOI_ENABLED is True:
                    pts_list = []
                    for k in self.al_voronoi_geo_storage:
                        pts_list.append(self.al_voronoi_geo_storage[k]['point'])
                    self.generate_voronoi_geometry(pts=pts_list)

                    self.probing_gcode_text = self.probing_gcode(self.al_voronoi_geo_storage)
                else:
                    self.app.inform.emit('[ERROR_NOTCL] %s' % _("Voronoi function can not be loaded.\n"
                                                                "Shapely >= 1.8 is required"))

            # rebuild the al table
            self.build_al_table_sig.emit()

            if self.ui.plot_probing_pts_cb.get_value():
                self.show_probing_geo(state=True, reset=True)
            else:
                # clear probe shapes
                self.plot_probing_geo(None, False)

    def on_key_press(self, event):
        # events out of the self.app.collection view (it's about Project Tab) are of type int
        if isinstance(event, int):
            key = event
        # events from the GUI are of type QKeyEvent
        elif isinstance(event, QtGui.QKeyEvent):
            key = event.key()
        elif isinstance(event, mpl_key_event):  # MatPlotLib key events are trickier to interpret than the rest
            key = event.key
            key = QtGui.QKeySequence(key)

            # check for modifiers
            key_string = key.toString().lower()
            if '+' in key_string:
                mod, __, key_text = key_string.rpartition('+')
                if mod.lower() == 'ctrl':
                    # modifiers = QtCore.Qt.KeyboardModifier.ControlModifier
                    pass
                elif mod.lower() == 'alt':
                    # modifiers = QtCore.Qt.KeyboardModifier.AltModifier
                    pass
                elif mod.lower() == 'shift':
                    # modifiers = QtCore.Qt.KeyboardModifier.
                    pass
                else:
                    # modifiers = QtCore.Qt.KeyboardModifier.NoModifier
                    pass
                key = QtGui.QKeySequence(key_text)
        # events from Vispy are of type KeyEvent
        else:
            key = event.key

        # Escape = Deselect All
        if key == QtCore.Qt.Key.Key_Escape or key == 'Escape':
            if self.mouse_events_connected is True:
                self.mouse_events_connected = False
                if self.app.use_3d_engine:
                    self.app.plotcanvas.graph_event_disconnect('key_press', self.on_key_press)
                    self.app.plotcanvas.graph_event_disconnect('mouse_release', self.on_mouse_click_release)
                    self.app.plotcanvas.graph_event_disconnect('mouse_move', self.on_mouse_move)
                else:
                    self.app.plotcanvas.graph_event_disconnect(self.kp)
                    self.app.plotcanvas.graph_event_disconnect(self.mr)

                self.app.kp = self.app.plotcanvas.graph_event_connect('key_press', self.app.ui.keyPressEvent)
                self.app.mp = self.app.plotcanvas.graph_event_connect('mouse_press', self.app.on_mouse_click_over_plot)
                self.app.mr = self.app.plotcanvas.graph_event_connect('mouse_release',
                                                                      self.app.on_mouse_click_release_over_plot)
                # restore selection
                self.app.options['global_selection_shape'] = self.old_selection_state

        # Grid toggle
        if key == QtCore.Qt.Key.Key_G or key == 'G':
            self.app.ui.grid_snap_btn.trigger()

        # Jump to coords
        if key == QtCore.Qt.Key.Key_J or key == 'J':
            self.app.on_jump_to()

    def autolevell_gcode(self):
        """
        Apply the probed heights over the GCode of the selected CNCJob object such that the cutting depth follows
        the measured surface of the material.

        :return:    'fail' if the levelling could not be done, None otherwise
        :rtype:     str
        """
        target_obj = self.app.collection.get_by_name(self.ui.object_combo.get_value())
        if target_obj is None:
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("Could not retrieve object"))
            return 'fail'

        if getattr(target_obj, 'is_levelled_gcode', False) is True:
            self.app.inform.emit(
                '[WARNING_NOTCL] %s' % _("The GCode of this object is already levelled. Generate it again to "
                                        "level it with new probing data.")
            )
            return 'fail'

        al_method = self.ui.al_method_radio.get_value()
        if al_method == 'b':
            heights = [pt[2] for pt in self.al_bilinear_geo_storage]
        else:
            heights = [val['height'] for val in self.al_voronoi_geo_storage.values()]

        if not heights or all(h == 0.0 for h in heights):
            self.app.inform.emit(
                '[ERROR_NOTCL] %s' % _("There is no height data. Do a probing job or import a height map first.")
            )
            return 'fail'

        # the heights are added to every Z of the job; far from the work Z zero they are not the warp of the
        # material but a Z zero that is not on its surface, and applying them would drive the tool into it
        limit = self.al_max_height if self.units == 'MM' else self.al_max_height / 25.4
        worst = max(heights, key=abs)
        if abs(worst) > limit:
            self.app.inform.emit('[ERROR_NOTCL] %s' % (
                _("A probed height is %.3f, more than %.3f away from the work Z zero. The levelling was not "
                  "applied: set the work Z zero on the surface of the material, with Probe Z for instance, and "
                  "probe again.") % (worst, limit)))
            return 'fail'

        dec = target_obj.coords_decimals

        levelled_something = False

        # for a multi tool job the GCode of each tool is stored separately and it is the one that gets exported
        if target_obj.multitool is True and target_obj.tools:
            for tooluid_key in target_obj.tools:
                tool_gcode = target_obj.tools[tooluid_key].get('gcode', None)
                if tool_gcode:
                    new_gcode = self.autolevell_gcode_block(tool_gcode, dec=dec)
                    levelled_something = levelled_something or new_gcode != tool_gcode
                    target_obj.tools[tooluid_key]['gcode'] = new_gcode

        if target_obj.gcode:
            new_gcode = self.autolevell_gcode_block(target_obj.gcode, dec=dec)
            levelled_something = levelled_something or new_gcode != target_obj.gcode
            target_obj.gcode = new_gcode

        if not levelled_something:
            # the job was made with a preprocessor that does not output GCode moves, HPGL and Roland for example
            self.app.inform.emit(
                '[WARNING_NOTCL] %s' % _("Nothing was levelled. This job holds no moves that the levelling can "
                                        "work with.")
            )
            return 'fail'

        target_obj.is_levelled_gcode = True

        # rebuild the text that the Code Review tab displays and that is saved on export
        try:
            new_code = target_obj.export_gcode(preamble=target_obj.prepend_snippet,
                                               postamble=target_obj.append_snippet,
                                               to_file=True,
                                               s_code=target_obj.gc_start)
            try:
                target_obj.source_file = new_code.getvalue()
            except AttributeError:
                target_obj.source_file = new_code
        except Exception as e:
            self.app.log.error("ToolLevelling.autolevell_gcode() --> %s" % str(e))

        self.app.inform.emit('[success] %s' % _("Finished autolevelling."))

    def autolevell_gcode_block(self, gcode, dec=4):
        """
        Apply the probed heights over a block of GCode.

        :param gcode:   the GCode to be levelled
        :type gcode:    str
        :param dec:     number of decimals used for the Z coordinate
        :type dec:      int
        :return:        the levelled GCode
        :rtype:         str
        """
        new_lines = []
        # the position before the current line; the Z is the programmed one, without any levelling applied
        coords = (None, None, None)

        for gcode_line in gcode.splitlines():
            new_line, coords = self.autolevell_gcode_line(gcode_line, coords, dec=dec)
            new_lines.append(new_line)

        levelled_gcode = '\n'.join(new_lines)
        if gcode.endswith('\n'):
            levelled_gcode += '\n'
        return levelled_gcode

    def autolevell_gcode_line(self, gcode_line, coords=(None, None, None), dec=4):
        """
        Apply the probed heights over a single line of GCode. The programmed depth is kept and the height of the
        material surface is added to it.

        :param gcode_line:  a line of GCode
        :type gcode_line:   str
        :param coords:      the (x, y, z) position before this line; the Z is the programmed one
        :type coords:       tuple
        :param dec:         number of decimals used for the Z coordinate
        :type dec:          int
        :return:            the new line and the (x, y, z) position after this line
        :rtype:             tuple
        """
        pos_x, pos_y, nominal_z = coords

        # a comment can hold anything, therefore it is not parsed
        body, sep, comment = gcode_line.partition('(')

        if not self.motion_re.search(body):
            return gcode_line, coords

        words = {}
        for match in self.coord_re.finditer(body):
            words[match.group(1).upper()] = float(match.group(2))

        new_x = words.get('X', pos_x)
        new_y = words.get('Y', pos_y)
        if 'Z' in words:
            nominal_z = words['Z']

        new_coords = (new_x, new_y, nominal_z)

        # a line without any coordinate (setting the feedrate for example) is not a move
        if 'Z' not in words and 'X' not in words and 'Y' not in words:
            return gcode_line, new_coords

        # the height is not known before the first move or before the first Z
        if new_x is None or new_y is None or nominal_z is None:
            return gcode_line, new_coords

        # an arc holds no Z of its own; adding one would turn it into a helix
        motion = int(self.motion_re.search(body).group(1))
        if motion > 1 and 'Z' not in words:
            return gcode_line, new_coords

        z_txt = 'Z' + '%.*f' % (dec, nominal_z + self.autolevell_height(new_x, new_y))

        if 'Z' in words:
            body = self.coord_re.sub(
                lambda m: z_txt if m.group(1).upper() == 'Z' else m.group(0), body)
        else:
            body = body.rstrip() + ' ' + z_txt

        return body + sep + comment, new_coords

    def autolevell_height(self, x, y):
        """
        The height of the material surface in the (x, y) point, according to the probed data.

        :return:    the surface height
        :rtype:     float
        """
        if self.ui.al_method_radio.get_value() == 'b':
            return self.autolevell_bilinear(x, y)
        return self.autolevell_voronoi(x, y)

    def autolevell_bilinear(self, x, y):
        """
        The surface height in the (x, y) point, interpolated from the four probe points that surround it.

        :return:    the surface height
        :rtype:     float
        """
        pts = self.al_bilinear_geo_storage
        rows = self.ui.al_rows_entry.get_value()
        cols = self.ui.al_columns_entry.get_value()

        # points that avoid the Excellon holes are moved therefore the grid may be incomplete
        if rows < 2 or cols < 2 or len(pts) != (rows * cols):
            return self.autolevell_closest_height(x, y)

        # the probe points are stored row by row, starting with the lower left corner
        grid_x = [pts[col][0] for col in range(cols)]
        grid_y = [pts[row * cols][1] for row in range(rows)]

        col = self.autolevell_cell_index(grid_x, x)
        row = self.autolevell_cell_index(grid_y, y)

        x0, x1 = grid_x[col], grid_x[col + 1]
        y0, y1 = grid_y[row], grid_y[row + 1]

        z00 = pts[row * cols + col][2]
        z10 = pts[row * cols + col + 1][2]
        z01 = pts[(row + 1) * cols + col][2]
        z11 = pts[(row + 1) * cols + col + 1][2]

        ratio_x = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
        ratio_y = 0.0 if y1 == y0 else (y - y0) / (y1 - y0)

        z_low = z00 + (z10 - z00) * ratio_x
        z_high = z01 + (z11 - z01) * ratio_x
        return z_low + (z_high - z_low) * ratio_y

    def autolevell_voronoi(self, x, y):
        """
        The surface height in the (x, y) point, taken from the probe point whose Voronoi cell holds it.

        :return:    the surface height
        :rtype:     float
        """
        pt = Point((x, y))

        for val in self.al_voronoi_geo_storage.values():
            cell = val['geo']
            if cell is not None and cell.intersects(pt):
                return val['height']

        # outside of the probed area or without a Voronoi diagram
        return self.autolevell_closest_height(x, y)

    def autolevell_closest_height(self, x, y):
        """
        The height of the probe point that is the closest to the (x, y) point.

        :return:    the surface height
        :rtype:     float
        """
        if self.ui.al_method_radio.get_value() == 'b':
            pts = [(pt[0], pt[1], pt[2]) for pt in self.al_bilinear_geo_storage]
        else:
            pts = [
                (val['point'].x, val['point'].y, val['height']) for val in self.al_voronoi_geo_storage.values()
            ]

        if not pts:
            return 0.0

        return min(pts, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)[2]

    @staticmethod
    def autolevell_cell_index(grid_values, val):
        """
        The index of the grid cell that holds val. A value outside of the grid gets the closest border cell which
        means that it is extrapolated.

        :return:    index of the lower edge of the cell
        :rtype:     int
        """
        idx = 0
        for i in range(len(grid_values) - 1):
            if val >= grid_values[i]:
                idx = i
        return min(idx, len(grid_values) - 2)

    def on_show_al_table(self, state):
        self.ui.al_probe_points_table.show() if state else self.ui.al_probe_points_table.hide()

    def on_mode_radio(self, val):
        # reset al table
        self.ui.al_probe_points_table.setRowCount(0)

        # reset the al dict
        self.al_voronoi_geo_storage.clear()

        # reset Voronoi Shapes
        self.probing_shapes.clear(update=True)

        # build AL table
        self.build_al_table()

        if val == "manual":
            self.ui.al_method_radio.set_value('v')
            self.ui.al_rows_entry.setDisabled(True)
            self.ui.al_rows_label.setDisabled(True)
            self.ui.al_columns_entry.setDisabled(True)
            self.ui.al_columns_label.setDisabled(True)
            self.ui.al_method_lbl.setDisabled(True)
            self.ui.al_method_radio.setDisabled(True)
            # self.ui.avoid_exc_holes_cb.setDisabled(False)
        else:
            self.ui.al_rows_entry.setDisabled(False)
            self.ui.al_rows_label.setDisabled(False)
            self.ui.al_columns_entry.setDisabled(False)
            self.ui.al_columns_label.setDisabled(False)
            self.ui.al_method_lbl.setDisabled(False)
            self.ui.al_method_radio.setDisabled(False)
            self.ui.al_method_radio.set_value(self.app.options['tools_al_method'])
            # self.ui.avoid_exc_holes_cb.setDisabled(True)

    def on_avoid_exc_holes(self, state):
        self.ui.avoid_exc_holes_size_label.show() if state else self.ui.avoid_exc_holes_size_label.hide()
        self.ui.avoid_exc_holes_size_entry.show() if state else self.ui.avoid_exc_holes_size_entry.hide()

    def on_method_radio(self, val):
        if val == 'b':
            self.ui.al_columns_entry.setMinimum(2)
            self.ui.al_rows_entry.setMinimum(2)
        else:
            self.ui.al_columns_entry.setMinimum(1)
            self.ui.al_rows_entry.setMinimum(1)

    def on_controller_change(self):
        # kept in the options, which are saved when the application closes, so the choice survives a restart
        self.app.options["tools_al_controller"] = self.ui.al_controller_combo.get_value()
        self.on_controller_change_alter_ui()

        # if the is empty then there is a chance that we've added probe points but the GRBL controller was selected
        # therefore no Probing GCode was generated (it is different for GRBL on how it gets it's Probing GCode
        target_obj = self.app.collection.get_by_name(self.ui.object_combo.get_value())
        if (not self.probing_gcode_text or self.probing_gcode_text == '') and target_obj is not None:
            # generate Probing GCode
            al_method = self.ui.al_method_radio.get_value()
            storage = self.al_voronoi_geo_storage if al_method == 'v' else self.al_bilinear_geo_storage
            self.probing_gcode_text = self.probing_gcode(storage=storage)

    def on_controller_change_alter_ui(self):
        if self.ui.al_controller_combo.get_value() == 'GRBL':
            self.ui.h_gcode_button.hide()
            self.ui.view_h_gcode_button.hide()

            self.ui.import_heights_button.hide()
            self.ui.grbl_frame.show()
            self.on_grbl_search_ports(muted=True)
        else:
            self.ui.h_gcode_button.show()
            self.ui.view_h_gcode_button.show()

            self.ui.import_heights_button.show()
            self.ui.grbl_frame.hide()

    @staticmethod
    def on_grbl_list_serial_ports():
        """
        Lists serial port names.
        From here: https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

        :raises EnvironmentError:   On unsupported or unknown platforms
        :returns:                   A list of the serial ports available on the system
        """

        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        s = serial.Serial()

        for port in ports:
            s.port = port

            try:
                s.open()
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                # result.append(port + " (in use)")
                pass

        return result

    def on_grbl_search_ports(self, muted=None):
        port_list = self.on_grbl_list_serial_ports()
        self.ui.com_list_combo.clear()
        self.ui.com_list_combo.addItems(port_list)
        if muted is not True:
            self.app.inform.emit('[WARNING_NOTCL] %s' % _("COM list updated ..."))

    def grbl_is_connected(self):
        return self.grbl_ser_port is not None and self.grbl_ser_port.is_open

    def grbl_set_connected_ui(self, connected):
        """
        Show whether the controller is connected and make the tabs that need it usable or not.

        :param connected:   whether the controller is connected
        :type connected:    bool
        """
        color = 'seagreen' if connected else 'red'
        self.ui.com_connect_button.setStyleSheet("QPushButton {background-color: %s;}" % color)
        self.ui.com_connect_button.setText(_("Connected") if connected else _("Disconnected"))
        self.ui.controller_reset_button.setDisabled(not connected)

        for idx in range(self.ui.al_toolbar.count()):
            if self.ui.al_toolbar.tabText(idx) == _("Connect"):
                self.ui.al_toolbar.tabBar.setTabTextColor(idx, QtGui.QColor(color))
            if self.ui.al_toolbar.tabText(idx) in (_("Control"), _("Sender")):
                self.ui.al_toolbar.tabBar.setTabEnabled(idx, connected)

        if connected:
            self.grbl_status_timer.start()
        else:
            self.grbl_status_timer.stop()
            self.grbl_status = {'state': None, 'mpos': None, 'wpos': None, 'wco': None, 'pins': ''}
        self.grbl_update_position_ui()

    def on_grbl_connect(self):
        if self.grbl_is_connected():
            # the button toggles: a connected controller is disconnected
            if self.grbl_job_running or self.grbl_busy:
                self.app.inform.emit('[WARNING_NOTCL] %s' % _("Stop the job before disconnecting."))
                return

            port_name = self.grbl_ser_port.port
            try:
                # the hard limits are not left off on a machine that is not watched anymore
                self.grbl_restore_hard_limits()
                self.grbl_ser_port.close()
            except Exception as e:
                self.app.log.error("ToolLevelling.on_grbl_connect() --> %s" % str(e))
            self.grbl_set_connected_ui(False)
            self.app.inform.emit("%s: %s" % (_("Disconnected from port"), port_name))
            return

        port_name = self.ui.com_list_combo.currentText()
        if " (" in port_name:
            port_name = port_name.rpartition(" (")[0]

        baudrate = int(self.ui.baudrates_list_combo.currentText())

        try:
            self.grbl_ser_port = serial.serial_for_url(port_name, baudrate,
                                                       bytesize=serial.EIGHTBITS,
                                                       parity=serial.PARITY_NONE,
                                                       stopbits=serial.STOPBITS_ONE,
                                                       timeout=0.1,
                                                       xonxoff=False,
                                                       rtscts=False)

            # Toggle DTR to reset the controller loaded with GRBL (Arduino, ESP32, etc.)
            try:
                self.grbl_ser_port.dtr = False
            except IOError:
                pass

            self.grbl_ser_port.reset_input_buffer()

            try:
                self.grbl_ser_port.dtr = True
            except IOError:
                pass

            answer = self.on_grbl_wake()
            for line in answer:
                if 'ok' in line.lower() or 'grbl' in line.lower():
                    # toggling DTR restarted the controller, whatever homing was done before is lost
                    self.grbl_homed = False
                    self.grbl_limits_suspended = False
                    self.grbl_poll_partial = b''
                    self.grbl_read_settings()
                    self.grbl_set_connected_ui(True)
                    self.grbl_refresh_controls()
                    state = self.grbl_read_status()
                    if state is not None and state.startswith('alarm'):
                        # with homing enabled GRBL starts up locked, the machine has not moved
                        self.app.inform.emit('[WARNING_NOTCL] %s: %s. %s' % (
                            _("Port connected"), port_name,
                            _("GRBL starts locked until the machine is homed: press Do Home, or Unlock "
                              "to work without homing.")))
                    else:
                        self.app.inform.emit("%s: %s" % (_("Port connected"), port_name))
                    return

            self.grbl_ser_port.close()
            self.grbl_set_connected_ui(False)
            self.app.inform.emit("[ERROR_NOTCL] %s: %s" % (_("Could not connect to GRBL on port"), port_name))

        except serial.SerialException as e:
            # the port does not exist, or another program, a second FlatCAM for instance, holds it open
            self.grbl_ser_port = None
            self.grbl_set_connected_ui(False)
            self.app.inform.emit("[ERROR_NOTCL] %s: %s (%s)" % (_("Could not connect to port"), port_name, str(e)))
        except Exception as e:
            self.app.inform.emit("[ERROR_NOTCL] %s: %s (%s)" % (_("Could not connect to port"), port_name, str(e)))

    def on_grbl_add_baudrate(self):
        new_bd = str(self.ui.new_baudrate_entry.get_value())
        if int(new_bd) >= 40 and new_bd not in self.ui.baudrates_list_combo.model().stringList():
            self.ui.baudrates_list_combo.addItem(new_bd)
            self.ui.baudrates_list_combo.setCurrentText(new_bd)

    def on_grbl_delete_baudrate_grbl(self):
        current_idx = self.ui.baudrates_list_combo.currentIndex()
        self.ui.baudrates_list_combo.removeItem(current_idx)

    def on_grbl_wake(self):
        # Wake up grbl
        self.grbl_ser_port.write("\r\n\r\n".encode('utf-8'))
        # Wait for GRBL controller to initialize
        time.sleep(1)

        grbl_out = self.grbl_decode(self.grbl_ser_port.readlines())
        self.grbl_ser_port.reset_input_buffer()

        return grbl_out

    @staticmethod
    def grbl_decode(grbl_lines):
        """
        Decode the lines read from the serial port.

        :param grbl_lines:  lines of bytes as read from the serial port
        :type grbl_lines:   list
        :return:            the decoded lines, without the line endings
        :rtype:             list
        """
        decoded = []
        for line in grbl_lines:
            try:
                decoded.append(line.decode('utf-8', errors='replace').strip())
            except AttributeError:
                # already a string
                decoded.append(str(line).strip())
        return decoded

    def grbl_note_lines(self, lines):
        """
        Keep what the controller tells about itself: its status reports and the alarms that lose the position.

        :param lines:   decoded lines read from the controller
        :type lines:    list
        """
        for line in lines:
            if line.startswith('<'):
                self.grbl_note_status(line)
            elif line.upper() in ('ALARM:1', 'ALARM:2', 'ALARM:3'):
                # a limit was hit or the controller was reset while moving, the position is not trusted anymore
                self.grbl_homed = False

    @contextlib.contextmanager
    def grbl_port_reader(self):
        with self.grbl_port_readers_lock:
            self.grbl_port_readers += 1
        try:
            yield
        finally:
            with self.grbl_port_readers_lock:
                self.grbl_port_readers -= 1

    def grbl_note_status(self, line):
        """
        Parse a status report like <Idle|MPos:0.000,0.000,0.000|FS:0,0|WCO:0.000,0.000,0.000|Pn:Z>

        :param line:    the status report
        :type line:     str
        """
        if not line.strip().endswith('>'):
            # a piece of a report; the numbers in it are not to be trusted
            return
        self.grbl_status_time = time.monotonic()
        self.grbl_status_line = line.strip()
        fields = line.strip().strip('<>').split('|')
        status = self.grbl_status
        status['state'] = fields[0]
        status['pins'] = ''

        reported = None
        for field in fields[1:]:
            key, __, value = field.partition(':')
            if key in ('MPos', 'WPos', 'WCO'):
                try:
                    numbers = [float(v) for v in value.split(',')[:3]]
                except ValueError:
                    continue
                if len(numbers) != 3:
                    continue
                status[key.lower()] = numbers
                if key != 'WCO':
                    reported = key
            elif key == 'Pn':
                status['pins'] = value

        # the controller reports either the machine or the work position ($10), the other one comes from the
        # offset of the work origin that it reports every now and then
        wco = status['wco']
        if reported == 'MPos':
            status['wpos'] = [m - o for m, o in zip(status['mpos'], wco)] if wco else None
        elif reported == 'WPos':
            status['mpos'] = [w + o for w, o in zip(status['wpos'], wco)] if wco else None

    def grbl_read_settings(self):
        """
        Read the settings of the controller ($$), they tell how the machine homes and how far it can travel.
        """
        self.grbl_settings = {}
        try:
            answer = self.send_grbl_command('$$', echo=False, timeout=5.0)
        except Exception as e:
            self.app.log.error("ToolLevelling.grbl_read_settings() --> %s" % str(e))
            return
        for line in answer.splitlines():
            key, __, value = line.partition('=')
            if key.startswith('$') and key[1:].isdigit():
                self.grbl_settings[int(key[1:])] = value.partition(' ')[0]

    def grbl_setting(self, number, default):
        try:
            return float(self.grbl_settings.get(number, default))
        except ValueError:
            return float(default)

    def grbl_ceilings(self):
        """
        For X, Y and Z the highest machine position the machine can be sent to without running into the limit
        switch it homes against. It is known only once the machine was homed, for an axis that homes in the
        positive direction, Z up for instance: GRBL leaves it at -$27 then, the pull-off away from the switch.

        :return:    the machine positions in mm, None for an axis where it is not known
        :rtype:     list
        """
        if not self.grbl_homed or int(self.grbl_setting(22, 0)) & 1 == 0:
            return [None, None, None]
        pulloff = self.grbl_setting(27, 1.0)
        # a bit of $23 that is set makes the axis home in the negative direction
        direction_mask = int(self.grbl_setting(23, 0))
        return [None if direction_mask & (1 << axis) else -pulloff for axis in range(3)]

    def on_grbl_status_tick(self):
        """
        Ask the controller for a status report for the position display. While a job streams or a task runs in the
        background they read the answers, otherwise the reports are read here without waiting for them.
        """
        if not self.grbl_is_connected():
            self.grbl_status_timer.stop()
            return

        reading = not self.grbl_busy and not (self.grbl_job_running and not self.grbl_job_waiting) and \
            self.grbl_port_readers == 0
        try:
            self.grbl_ser_port.write(b'?')
            if reading:
                self.grbl_poll_port()
            else:
                self.grbl_poll_partial = b''
        except Exception as e:
            self.app.log.error("ToolLevelling.on_grbl_status_tick() --> %s" % str(e))
            self.grbl_status_timer.stop()
            return

        self.grbl_update_position_ui()

        state = (self.grbl_status['state'] or '').lower()
        if reading and self.grbl_limits_suspended and state == 'idle' and \
                not any(axis in self.grbl_status['pins'] for axis in 'XYZ'):
            self.grbl_run_in_background(self.grbl_restore_hard_limits)

    def grbl_poll_port(self):
        """
        Read what came in for the position display without waiting for it.
        """
        with self.grbl_port_reader():
            waiting = self.grbl_ser_port.in_waiting
            if not waiting:
                return
            raw = self.grbl_poll_partial + self.grbl_ser_port.read(waiting)
            lines = raw.split(b'\n')
            self.grbl_poll_partial = lines.pop()
            decoded = self.grbl_decode(lines)
            self.grbl_note_lines(decoded)

        for line in decoded:
            # what the controller says on its own, a limit alarm for instance, would be lost otherwise
            if line and not line.startswith('<') and line.lower() != 'ok':
                self.app.inform_shell.emit('\t\t\t: ' + line.upper())

    @staticmethod
    def grbl_origin_target(travelz, wco, ceilings):
        """
        Where Go to origin sends the tool, in machine coordinates: over the work origin at the travel height,
        but not past the limit switches.

        :param travelz:     the travel height, in work coordinates
        :type travelz:      float
        :param wco:         the machine position of the work origin
        :type wco:          list
        :param ceilings:    as they come out of grbl_ceilings()
        :type ceilings:     list
        :return:            the machine X, Y and Z, and for each of them whether it was limited
        :rtype:             tuple
        """
        wanted = [wco[0], wco[1], wco[2] + float(travelz)]
        target = [value if ceiling is None else min(value, ceiling) for value, ceiling in zip(wanted, ceilings)]
        return target, [t < w - 1e-9 for t, w in zip(target, wanted)]

    def grbl_update_position_ui(self):
        """
        Show the state and the position of the machine, where its work origin is and where Go to origin moves to.
        """
        status = self.grbl_status
        state = status['state']

        def show(labels, numbers):
            for label, number in zip(labels, numbers if numbers else [None] * 3):
                label.set_value('-' if number is None else '%.3f' % number)

        try:
            if state is None:
                self.ui.grbl_state_label.set_value('-')
                self.ui.grbl_state_label.setStyleSheet("")
            else:
                text = state
                if status['pins']:
                    limits = ''.join(axis for axis in 'XYZ' if axis in status['pins'])
                    if limits:
                        text += '  (%s: %s)' % (_("limit switch pressed"), limits)
                    if 'P' in status['pins']:
                        text += '  (%s)' % _("probe touching")
                if self.grbl_limits_suspended:
                    text += '  (%s)' % _("hard limits off")
                age = time.monotonic() - self.grbl_status_time
                stale = age > 2.0
                if stale:
                    text += '  (%s %d s)' % (_("NOT UPDATING, last report"), age)
                self.ui.grbl_state_label.set_value(text)
                color = 'gray' if stale else 'red' if state.lower().startswith('alarm') else \
                    'darkorange' if state.lower().startswith(('hold', 'door')) else 'seagreen'
                self.ui.grbl_state_label.setStyleSheet("QLabel {color: %s; font-weight: bold;}" % color)

            show(self.ui.grbl_wpos_labels, status['wpos'])
            show(self.ui.grbl_mpos_labels, status['mpos'])
            show(self.ui.grbl_wco_labels, status['wco'])

            wco = status['wco']
            if state is None or wco is None:
                self.ui.grbl_origin_label.set_value('')
                return
            travelz = float(self.app.options["tools_al_grbl_travelz"])
            ceilings = self.grbl_ceilings() if self.units == 'MM' else [None, None, None]
            target, limited = self.grbl_origin_target(travelz, wco, ceilings)
            text = '%s: %s X%.3f Y%.3f Z%.3f  (%s X%.3f Y%.3f Z%.3f)' % (
                _("Go to origin"), _("machine"), *target,
                _("work"), *[t - o for t, o in zip(target, wco)])
            if any(limited):
                text += '\n%s: %s' % (_("Stops short of the limit switch on"),
                                      ''.join(axis for axis, flag in zip('XYZ', limited) if flag))
            elif not self.grbl_homed:
                text += '\n%s' % _("Not homed: where the limit switches are is not known, check the move is safe.")
            self.ui.grbl_origin_label.set_value(text)
        except (RuntimeError, AttributeError):
            # the panel was not built or was already deleted
            pass

    def grbl_read_reply(self, timeout=30.0, keep_status=False):
        with self.grbl_port_reader():
            return self._grbl_read_reply(timeout=timeout, keep_status=keep_status)

    def _grbl_read_reply(self, timeout=30.0, keep_status=False):
        """
        Read the answer of the GRBL controller until it acknowledges the command with 'ok' or reports a problem.
        The serial port has a short read timeout therefore reading is repeated until the deadline.

        :param timeout:     for how long to wait for an acknowledgement, in seconds
        :type timeout:      float
        :param keep_status: if the status reports are part of the answer; the position display asks for them all
                            the time therefore they are left out unless the command asked for one
        :type keep_status:  bool
        :return:            the answer lines of the controller and whether it acknowledged the command
        :rtype:             tuple
        """
        deadline = time.monotonic() + timeout
        answer = []
        # a line that was read before all of it arrived, a long one of a $$ answer for instance
        partial = b''
        resets = self.grbl_resets

        while time.monotonic() < deadline:
            if self.grbl_resets != resets:
                # the controller restarted and forgot the command
                return answer, False

            raw = self.grbl_ser_port.readlines()
            if raw:
                raw[0] = partial + raw[0]
                partial = b''
                if not raw[-1].endswith(b'\n'):
                    partial = raw.pop()
            new_lines = self.grbl_decode(raw)
            self.grbl_note_lines(new_lines)
            answer += [line for line in new_lines if line != '' and (keep_status or not line.startswith('<'))]

            for line in answer:
                low_line = line.lower()
                if low_line.startswith('ok'):
                    return answer, True
                if low_line.startswith('error') or low_line.startswith('alarm'):
                    return answer, False

            time.sleep(0.01)

        return answer, False

    def on_grbl_send_command(self):
        cmd = self.ui.grbl_command_entry.get_value()
        if not cmd or not self.grbl_is_connected():
            return

        # show the Shell Dock
        self.app.ui.shell_dock.show()

        # a command can take long to be acknowledged, a homing for instance, therefore it runs in the background
        self.grbl_run_in_background(lambda: self.send_grbl_command(command=cmd, timeout=180.0))

    def send_grbl_command(self, command, echo=True, timeout=30.0):
        """

        :param command: GCode command
        :type command:  str
        :param echo:    if to print the command and the answer in the shell
        :type echo:     bool
        :param timeout: for how long to wait for an acknowledgement, in seconds
        :type timeout:  float
        :return:        the text returned by the GRBL controller after each command
        :rtype:         str
        """
        cmd = command.strip()
        if echo:
            self.app.inform_shell[str, bool].emit(cmd, False)

        # Send Gcode command to GRBL
        snd = cmd + '\n'
        self.grbl_ser_port.write(snd.encode('utf-8'))
        answer, acknowledged = self.grbl_read_reply(timeout=timeout, keep_status=cmd == '?')
        if acknowledged and cmd.upper() == '$H':
            self.grbl_homed = True

        if not answer:
            self.app.inform_shell[str, bool].emit('\t\t\t: No answer\n', False)

        if not echo and not acknowledged and answer:
            # a refusal is always shown, otherwise a button whose command was refused looks like it does nothing
            self.app.inform_shell[str, bool].emit(cmd, False)
        if echo or not acknowledged:
            for line in answer:
                self.app.inform_shell.emit('\t\t\t: ' + line.upper())

        return '\n'.join(answer) if acknowledged else ''

    def send_grbl_block(self, command, echo=True):
        stripped_cmd = command.strip()

        for grbl_line in stripped_cmd.split('\n'):
            if echo:
                self.app.inform_shell[str, bool].emit(grbl_line, False)

            # Send Gcode block to GRBL
            snd = grbl_line + '\n'
            self.grbl_ser_port.write(snd.encode('utf-8'))
            answer, _ack = self.grbl_read_reply()

            if echo:
                for line in answer:
                    self.app.inform_shell.emit(' : ' + line.upper())

    def on_grbl_report(self):
        if not self.grbl_is_connected():
            return

        # a status report is not acknowledged with 'ok', therefore it is not sent like a command
        self.app.ui.shell_dock.show()
        self.app.inform_shell[str, bool].emit('?', False)
        if self.grbl_read_status() is None:
            self.app.inform_shell[str, bool].emit('\t\t\t: No answer\n', False)
        else:
            self.app.inform_shell.emit('\t\t\t: ' + self.grbl_status_line)

    def on_grbl_get_parameter(self, param):
        param = str(param).replace('$', '').strip()

        snd = '$$\n'
        self.grbl_ser_port.write(snd.encode('utf-8'))
        answer, _ack = self.grbl_read_reply()
        # an exact match: '$1' is also the start of '$10', '$11' and '$100'
        par = '$%s=' % param
        for decoded_line in answer:
            if decoded_line.startswith(par):
                try:
                    result = float(decoded_line.partition('=')[2].split()[0])
                except (ValueError, IndexError):
                    continue
                self.app.shell_message("GRBL Parameter: %s = %s" % (str(param), str(result)), show=True)
                return result

        self.app.inform.emit('[WARNING_NOTCL] %s: $%s' % (_("The GRBL controller did not report the parameter"),
                                                          param))
        return None

    @staticmethod
    def grbl_jog_commands(direction, units, step, feedrate, travelz, ceilings=None, wco=None, mpos=None):
        """
        The jog commands that move the machine one step in a direction, or back to the work origin.

        A move that would go past the limit switch the machine homes against stops short of it instead: hitting
        the switch puts the controller in an alarm that only a reset clears.

        :param direction:   'xplus', 'xminus', 'yplus', 'yminus', 'zplus', 'zminus' or 'origin'
        :type direction:    str
        :param units:       'MM' or 'IN'
        :type units:        str
        :param step:        the length of a step
        :type step:         str
        :param feedrate:    the feedrate of the jog
        :type feedrate:     str
        :param travelz:     the height to lift to before going to the origin, in work coordinates
        :type travelz:      str
        :param ceilings:    as they come out of grbl_ceilings(), or None when they are not known
        :type ceilings:     list
        :param wco:         the machine position of the work origin, in mm
        :type wco:          list
        :param mpos:        the machine position, in mm
        :type mpos:         list
        :return:            the commands, in the order they are to be sent; none for an unknown direction or for a
                            step towards a limit switch the machine is at already
        :rtype:             list
        """
        unit_code = {'IN': 'G20', 'MM': 'G21'}[units]
        # the positions the controller reports are in mm
        if units != 'MM' or ceilings is None:
            ceilings = [None, None, None]

        if direction == 'origin':
            lift = '$J=G90 %s Z%s F%s' % (unit_code, travelz, feedrate)
            over = '$J=G90 %s X0.0 Y0.0 F%s' % (unit_code, feedrate)
            if wco is not None:
                target, limited = ToolLevelling.grbl_origin_target(travelz, wco, ceilings)
                if limited[2]:
                    lift = '$J=G53 G21 Z%.3f F%s' % (target[2], feedrate)
                if limited[0] or limited[1]:
                    over = '$J=G53 G21 X%.3f Y%.3f F%s' % (target[0], target[1], feedrate)
            # lifted first, so that the tool is not dragged over the work on the way
            return [lift, over]

        axis = {'xplus': 0, 'yplus': 1, 'zplus': 2}.get(direction)
        if axis is not None and ceilings[axis] is not None and mpos is not None and \
                mpos[axis] + float(step) > ceilings[axis]:
            if mpos[axis] >= ceilings[axis] - 0.001:
                return []
            return ['$J=G53 G21 %s%.3f F%s' % ('XYZ'[axis], ceilings[axis], feedrate)]

        move = {
            'xplus': 'X%s', 'xminus': 'X-%s',
            'yplus': 'Y%s', 'yminus': 'Y-%s',
            'zplus': 'Z%s', 'zminus': 'Z-%s',
        }.get(direction)
        if move is None:
            return []
        return ['$J=G91 %s %s F%s' % (unit_code, move % step, feedrate)]

    def on_grbl_jog(self, direction=None):
        if self.grbl_check_manual() is None:
            return

        commands = self.grbl_jog_commands(
            direction=direction,
            units=self.units,
            step=str(self.ui.jog_step_entry.get_value()),
            feedrate=str(self.ui.jog_fr_entry.get_value()),
            travelz=str(float(self.app.options["tools_al_grbl_travelz"])),
            ceilings=self.grbl_ceilings(),
            wco=self.grbl_status['wco'],
            mpos=self.grbl_status['mpos']
        )
        if not commands and direction in ('xplus', 'yplus', 'zplus'):
            self.app.inform.emit('[WARNING_NOTCL] %s' % _("The axis is at its limit switch already."))
            return

        for command in commands:
            if self.grbl_send_manual(command, check_state=False) is False:
                return

    def grbl_check_manual(self):
        """
        Find out if the controller takes commands. In an alarm or a hold it refuses them, and after a limit was
        hit it does not even answer them, therefore nothing is sent then and the operator is told what to do.

        :return:    the state of the controller, or None when it cannot take commands
        :rtype:     str
        """
        if not self.grbl_is_connected():
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller is not connected."))
            return None

        state = self.grbl_read_status()
        if state is None:
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller does not answer. "
                                                        "Press Reset on the Connect tab."))
            return None
        if state.startswith('alarm'):
            limits = ''.join(axis for axis in 'XYZ' if axis in self.grbl_status['pins'])
            if limits:
                self.app.inform.emit('[ERROR_NOTCL] %s: %s. %s' % (
                    _("The GRBL controller is in an alarm state and a limit switch is pressed"), limits,
                    _("Press Do Home, or Unlock and jog away from the switch.")))
            else:
                self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller is in an alarm state. "
                                                            "Press Do Home, or Unlock."))
            return None
        if state.startswith(('hold', 'door')):
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller holds the motion. "
                                                        "Press Pause/Resume to go on, or Reset."))
            return None
        return state

    def grbl_send_manual(self, command, check_state=True):
        """
        Send a command that the operator gave by pressing a button and say so when the controller refused it.
        A jog that is out of the machine travel, or anything sent while the controller is in an alarm, is
        answered with an error and the machine does not move, which without this looks like a button that does
        nothing.

        :param command:     the command
        :type command:      str
        :param check_state: if to find out first that the controller takes commands
        :type check_state:  bool
        :return:            True when the controller accepted the command
        :rtype:             bool
        """
        if not self.grbl_is_connected():
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller is not connected."))
            return False

        if check_state and self.grbl_check_manual() is None:
            return False

        if self.send_grbl_command(command=command, echo=False, timeout=10.0):
            return True

        # the state says why: an alarm has to be cleared with $X or a homing, a hold with the Pause/Resume
        # button, and a jog that is refused while the machine is idle was out of the travel the soft limits allow
        state = self.grbl_read_status()
        self.app.inform.emit('[ERROR_NOTCL] %s: %s (%s: %s)' % (
            _("The GRBL controller rejected the command"), command, _("state"), state if state else _("unknown")))
        return False

    @staticmethod
    def grbl_zero_command(axis):
        """
        The command that makes the position the machine is at the origin of the work coordinate system.

        L20 is used and not L2: L2 sets the offset of a coordinate system to a machine position, which would
        move the origin to wherever the machine was homed and throw away the origin of the job. P0 addresses
        the coordinate system that is in use instead of always G54.

        :param axis:    'x', 'y', 'z' or anything else for all of them
        :type axis:     str
        :return:        the GCode command
        :rtype:         str
        """
        return 'G10 L20 P0 %s' % {'x': 'X0', 'y': 'Y0', 'z': 'Z0'}.get(axis, 'X0 Y0 Z0')

    def on_grbl_zero(self, axis):
        # G10 L20 does not depend on how the status is reported, therefore the $10 setting is left alone; changing
        # it would write the settings memory of the controller twice for every press
        if self.grbl_send_manual(self.grbl_zero_command(axis)):
            self.app.inform.emit('[success] %s' % _("The work origin was set at the current position."))

    @staticmethod
    def grbl_probe_z_commands(units, travel, feedrate, thickness, lift):
        """
        The commands that set the length of the tool by touching the surface with it.

        The probing move is relative because the tool can be at any height above the surface. The distance mode
        is set back to absolute before the origin is set, so that a command that follows cannot be taken as a
        relative move.

        :param units:       'MM' or 'IN'
        :type units:        str
        :param travel:      how far down to probe at most
        :type travel:       str
        :param feedrate:    feedrate of the probing move
        :type feedrate:     str
        :param thickness:   the Z that the touch point is given
        :type thickness:    str
        :param lift:        the height to pull the tool up to afterwards
        :type lift:         str
        :return:            the GCode commands, in the order they are to be sent
        :rtype:             list
        """
        return [
            {'IN': 'G20', 'MM': 'G21'}[units],
            'G91 G38.2 Z-%s F%s' % (travel, feedrate),
            'G90',
            'G10 L20 P0 Z%s' % thickness,
            'G0 Z%s' % lift
        ]

    def on_grbl_probe_z(self):
        """
        Probe down for the surface and make the touch point the Z origin, which sets the length of the tool
        that was just mounted.
        """
        if self.grbl_check_manual() is None:
            return

        # show the Shell Dock
        self.app.ui.shell_dock.show()

        commands = self.grbl_probe_z_commands(
            units=self.units,
            travel=str(self.ui.probe_travel_entry.get_value()),
            feedrate=str(self.ui.probe_fr_entry.get_value()),
            thickness=str(self.ui.plate_thickness_entry.get_value()),
            lift=str(self.app.options["tools_al_grbl_travelz"])
        )

        self.grbl_run_in_background(lambda: self.grbl_probe_z(commands))

    def grbl_probe_z(self, commands):
        """
        Send the touch off commands and stop at the first one that fails.

        :param commands:    as they come out of grbl_probe_z_commands()
        :type commands:     list
        :return:            True when the tool length was set
        :rtype:             bool
        """
        self.app.inform.emit('%s' % _("Probing for the surface."))

        for command in commands:
            output = self.send_grbl_command(command=command, timeout=120.0)

            if 'G38' in command and self.parse_probe_answer(output) is None:
                # the probe did not touch anything; the controller is in an alarm state and the distance mode
                # is still relative, therefore nothing more is sent and the origin keeps the value it had
                self.app.inform.emit(
                    '[ERROR_NOTCL] %s' % _("The probe did not touch the surface. Unlock the controller "
                                           "with $X and set the distance mode back with G90.")
                )
                return False

            if not output:
                self.app.inform.emit('[ERROR_NOTCL] %s: %s' % (_("The GRBL controller rejected the line"), command))
                return False

        self.app.inform.emit('[success] %s' % _("The tool length was set."))
        return True

    def on_grbl_homing(self):
        if not self.grbl_is_connected():
            return

        self.app.inform.emit("%s" % _("GRBL is doing a home cycle."))
        self.app.ui.shell_dock.show()
        self.grbl_run_in_background(self.grbl_homing)

    def grbl_homing(self):
        """
        Run a homing cycle. The controller only acknowledges it once the machine reached its home position,
        which on a large machine takes a while.

        :return:    True when the machine is homed
        :rtype:     bool
        """
        self.grbl_ser_port.reset_input_buffer()
        state = self.grbl_read_status()
        if state is None:
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller does not answer. "
                                                        "Press Reset on the Connect tab."))
            return False
        if state.startswith('alarm') and not self.grbl_soft_reset():
            return False

        resets = self.grbl_resets
        if self.send_grbl_command(command='$H', timeout=180.0):
            self.app.inform.emit('[success] %s' % _("The machine is homed."))
            return True

        if self.grbl_resets == resets:
            self.app.inform.emit(
                '[ERROR_NOTCL] %s' % _("The homing cycle failed. The answer of the controller is in the shell; "
                                       "error:5 means that homing is disabled with $22=0."))
        return False

    def grbl_soft_reset(self):
        """
        Reset the controller and wait for it to restart. After a limit was hit GRBL takes nothing but a reset;
        the work offsets set with G10 are kept, the position is lost.

        :return:    True when the controller answers again
        :rtype:     bool
        """
        self.grbl_resets += 1
        self.grbl_homed = False
        self.grbl_ser_port.write(b'\x18')
        # the controller prints its welcome message once it restarted
        time.sleep(1.5)
        self.grbl_ser_port.reset_input_buffer()

        if self.grbl_read_status() is None:
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller did not answer after the reset."))
            return False
        return True

    def grbl_suspend_hard_limits(self):
        """
        Switch the hard limits off. A limit switch raises the alarm when it changes, released or pressed, therefore
        the machine cannot be jogged off a pressed switch while they are on.

        :return:    True when they were switched off
        :rtype:     bool
        """
        if not self.send_grbl_command(command='$21=0', echo=False, timeout=5.0):
            return False
        self.grbl_settings[21] = '0'
        self.grbl_limits_suspended = True
        return True

    def grbl_restore_hard_limits(self):
        """
        Switch the hard limits on again once no limit switch is pressed anymore.

        :return:    True when they are on
        :rtype:     bool
        """
        if not self.grbl_limits_suspended:
            return True
        if not self.send_grbl_command(command='$21=1', echo=False, timeout=5.0):
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The hard limits could not be switched on again, "
                                                        "send $21=1."))
            return False
        self.grbl_settings[21] = '1'
        self.grbl_limits_suspended = False
        self.app.inform.emit('[success] %s' % _("The limit switch is released, the hard limits are on again."))
        return True

    def on_grbl_unlock(self):
        if not self.grbl_is_connected():
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller is not connected."))
            return

        self.app.ui.shell_dock.show()
        self.grbl_run_in_background(self.grbl_unlock)

    def grbl_unlock(self):
        """
        Clear an alarm with $X. After a limit was hit the controller does not answer $X before it was reset,
        therefore it is reset then and $X is sent again.

        :return:    True when the controller is unlocked
        :rtype:     bool
        """
        self.grbl_ser_port.reset_input_buffer()
        state = self.grbl_read_status()
        if state is None:
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller does not answer. "
                                                        "Press Reset on the Connect tab."))
            return False
        if not state.startswith('alarm'):
            self.app.inform.emit('%s: %s' % (_("The GRBL controller is not locked, its state is"), state))
            return True

        # an $X that can be taken is answered at once
        unlocked = bool(self.send_grbl_command(command='$X', timeout=1.0))
        if not unlocked:
            # a critical alarm, a limit was hit
            if not self.grbl_soft_reset():
                return False
            unlocked = bool(self.send_grbl_command(command='$X', timeout=3.0))
        if not unlocked:
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller could not be unlocked. "
                                                        "Press Do Home."))
            return False

        self.grbl_read_status()
        limits = ''.join(axis for axis in 'XYZ' if axis in self.grbl_status['pins'])
        if limits and int(self.grbl_setting(21, 0)) & 1 and self.grbl_suspend_hard_limits():
            self.app.inform.emit('[WARNING_NOTCL] %s: %s. %s' % (
                _("Unlocked, a limit switch is pressed"), limits,
                _("Hard limits are off until it is released, jog away from it.")))
        elif limits:
            self.app.inform.emit('[WARNING_NOTCL] %s: %s. %s' % (
                _("Unlocked, but a limit switch is still pressed"), limits,
                _("Jog away from it in small steps; if the alarm comes back, press Do Home.")))
        elif self.grbl_homed:
            self.app.inform.emit('[success] %s' % _("The GRBL controller is unlocked."))
        else:
            self.app.inform.emit('[WARNING_NOTCL] %s' % _("The GRBL controller is unlocked, but the machine "
                                                          "position is not known. Home it before a job."))
        return True

    def on_grbl_reset(self):
        """
        Soft reset the controller. It is also the way to stop the machine at once, therefore it is written before
        anything else and nothing waits for it.
        """
        if not self.grbl_is_connected():
            return

        # the controller forgets everything it was given, a job that is being sent cannot go on
        self.grbl_job_stop = True
        self.grbl_job_waiting = False
        self.grbl_resets += 1
        try:
            self.grbl_ser_port.write(b'\x18')
        except Exception as e:
            self.app.log.error("ToolLevelling.on_grbl_reset() --> %s" % str(e))
            return

        self.ui.pause_resume_button.setChecked(False)
        self.app.inform.emit("%s" % _("GRBL software reset was sent."))
        self.grbl_run_in_background(self.grbl_report_after_reset)

    def grbl_report_after_reset(self):
        """
        Wait for the controller to restart and tell the operator the state it came up in.
        """
        # the controller prints its welcome message once it restarted
        time.sleep(1.5)
        self.grbl_ser_port.reset_input_buffer()

        state = self.grbl_read_status()
        if state is None:
            self.app.inform.emit('[WARNING_NOTCL] %s' % _("The GRBL controller did not answer after the reset."))
        elif state.startswith('alarm'):
            self.app.inform.emit(
                '[WARNING_NOTCL] %s' % _("The GRBL controller is in an alarm state after the reset. "
                                         "Home the machine, or unlock it with $X."))
        else:
            self.app.inform.emit('%s: %s' % (_("GRBL state"), state))

    def grbl_run_in_background(self, task):
        """
        Run something that talks to the controller for a while, a homing or a probe, without blocking the GUI.
        Everything else that reads the answers of the controller is disabled until it is done.

        A thread of its own and not a worker task: the worker that streams a job holds one of the few threads of
        the pool, and a probe at a tool change that is queued there would only run after the job was continued.

        :param task:    what to run
        :type task:     callable
        """
        self.grbl_busy += 1
        self.grbl_refresh_controls()

        def run():
            try:
                task()
            except Exception as e:
                self.app.log.error("ToolLevelling.grbl_run_in_background() --> %s" % str(e))
                self.app.inform.emit('[ERROR_NOTCL] %s' % str(e))
            finally:
                self.grbl_busy_done_sig.emit()

        threading.Thread(target=run, daemon=True).start()

    def on_grbl_busy_done(self):
        self.grbl_busy = max(0, self.grbl_busy - 1)
        self.grbl_refresh_controls()

    def on_grbl_pause_resume(self, checked):
        if not self.grbl_is_connected():
            self.ui.pause_resume_button.setChecked(False)
            return

        # feed hold and cycle start are realtime commands that the controller does not acknowledge, therefore
        # they are written directly: waiting for an answer that never comes would block for the whole timeout
        # and would take the answers that a job which is being streamed waits for
        if checked is False:
            self.grbl_ser_port.write('~'.encode('utf-8'))
            self.app.inform.emit("%s" % _("GRBL resumed."))
        else:
            self.grbl_ser_port.write('!'.encode('utf-8'))
            self.app.inform.emit("%s" % _("GRBL paused."))

    def probing_gcode(self, storage):
        """
        :param storage:         either a dict of dicts (voronoi) or a list of tuples (bilinear)
        :return:                Probing GCode
        :rtype:                 str
        """

        target_obj = self.app.collection.get_by_name(self.ui.object_combo.get_value())

        p_gcode = ''
        header = ''
        time_str = "{:%A, %d %B %Y at %H:%M}".format(dt.now())

        coords = []
        al_method = self.ui.al_method_radio.get_value()
        if al_method == 'v':
            for id_key, value in storage.items():
                x = value['point'].x
                y = value['point'].y
                coords.append(
                    (
                        self.app.dec_format(x, dec=self.app.decimals),
                        self.app.dec_format(y, dec=self.app.decimals)
                    )
                )
        else:
            for pt in storage:
                x = pt[0]
                y = pt[1]
                coords.append(
                    (
                        self.app.dec_format(x, dec=self.app.decimals),
                        self.app.dec_format(y, dec=self.app.decimals)
                    )
                )

        pr_travel = self.ui.ptravelz_entry.get_value()
        probe_fr = self.ui.feedrate_probe_entry.get_value()
        pr_depth = self.ui.pdepth_entry.get_value()
        controller = self.ui.al_controller_combo.get_value()

        header += '(G-CODE GENERATED BY FLATCAM v%s - www.flatcam.org - Version Date: %s)\n' % \
                  (str(self.app.version), str(self.app.version_date)) + '\n'

        header += '(This is a autolevelling probing GCode.)\n' \
                  '(Make sure that before you start the job you first do a zero for all axis.)\n\n'

        header += '(Name: ' + str(target_obj.obj_options['name']) + ')\n'
        header += '(Type: ' + "Autolevelling Probing GCode " + ')\n'

        header += '(Units: ' + self.units.upper() + ')\n'
        header += '(Created on ' + time_str + ')\n'

        # commands
        if controller == 'MACH3':
            probing_command = 'G31'
            # probing_var = '#2002'
            openfile_command = 'M40'
            closefile_command = 'M41'
        elif controller == 'MACH4':
            probing_command = 'G31'
            # probing_var = '#5063'
            openfile_command = 'M40'
            closefile_command = 'M41'
        elif controller == 'LinuxCNC':
            probing_command = 'G38.2'
            # probing_var = '#5422'
            openfile_command = '(PROBEOPEN a_probing_points_file.txt)'
            closefile_command = '(PROBECLOSE)'
        elif controller == 'GRBL':
            # do nothing here because the Probing GCode for GRBL is obtained differently
            return
        else:
            self.app.log.debug("CNCJobObject.probing_gcode() -> controller not supported")
            return

        # #############################################################################################################
        # ########################### GCODE construction ##############################################################
        # #############################################################################################################

        # header
        p_gcode += header + '\n'
        # supplementary message for LinuxCNC
        if controller == 'LinuxCNC':
            p_gcode += "The file with the stored probing points can be found\n" \
                       "in the configuration folder for LinuxCNC.\n" \
                       "The name of the file is: a_probing_points_file.txt.\n"
        # units
        p_gcode += 'G21\n' if self.units == 'MM' else 'G20\n'
        # reference mode = absolute
        p_gcode += 'G90\n'
        # open a new file
        p_gcode += openfile_command + '\n'
        # move to safe height (probe travel Z)
        p_gcode += 'G0 Z%s\n' % str(self.app.dec_format(pr_travel, target_obj.coords_decimals))

        # probing points
        for idx, xy_tuple in enumerate(coords, 1):  # index starts from 1
            x = xy_tuple[0]
            y = xy_tuple[1]
            # move to probing point
            p_gcode += "G0 X%sY%s\n" % (
                str(self.app.dec_format(x, target_obj.coords_decimals)),
                str(self.app.dec_format(y, target_obj.coords_decimals))
            )
            # do the probing
            p_gcode += "%s Z%s F%s\n" % (
                probing_command,
                str(self.app.dec_format(pr_depth, target_obj.coords_decimals)),
                str(self.app.dec_format(probe_fr, target_obj.fr_decimals)),
            )
            # store in a global numeric variable the value of the detected probe Z
            # I offset the global numeric variable by 500 so, it does not conflict with something else
            # temp_var = int(idx + 500)
            # p_gcode += "#%d = %s\n" % (temp_var, probing_var)

            # move to safe height (probe travel Z)
            p_gcode += 'G0 Z%s\n' % str(self.app.dec_format(pr_travel, target_obj.coords_decimals))

        # close the file
        p_gcode += closefile_command + '\n'
        # finish the GCode
        p_gcode += 'M2'

        return p_gcode

    def on_save_probing_gcode(self):
        lines = StringIO(self.probing_gcode_text)

        _filter_ = self.app.options['cncjob_save_filters']
        name = "probing_gcode"
        try:
            dir_file_to_save = self.app.get_last_save_folder() + '/' + str(name)
            filename, _f = FCFileSaveDialog.get_saved_filename(
                caption=_("Export Code ..."),
                directory=dir_file_to_save,
                ext_filter=_filter_
            )
        except TypeError:
            filename, _f = FCFileSaveDialog.get_saved_filename(
                caption=_("Export Code ..."),
                ext_filter=_filter_)

        if filename == '':
            self.app.inform.emit('[WARNING_NOTCL] %s' % _("Export cancelled ..."))
            return
        else:
            try:
                force_windows_line_endings = self.app.options['cncjob_line_ending']
                if force_windows_line_endings and sys.platform != 'win32':
                    with open(filename, 'w', newline='\r\n') as f:
                        for line in lines:
                            f.write(line)
                else:
                    with open(filename, 'w') as f:
                        for line in lines:
                            f.write(line)
            except FileNotFoundError:
                self.app.inform.emit('[WARNING_NOTCL] %s' % _("No such file or directory"))
                return
            except PermissionError:
                self.app.inform.emit(
                    '[WARNING] %s' % _("Permission denied, saving not possible.\n"
                                       "Most likely another app is holding the file open and not accessible.")
                )
                return 'fail'

    def on_edit_probing_gcode(self):
        self.app.proc_container.view.set_busy('%s...' % _("Loading"))

        gco = self.probing_gcode_text
        if gco is None or gco == '':
            self.app.inform.emit('[WARNING_NOTCL] %s...' % _('There is nothing to view'))
            return

        self.gcode_viewer_tab = AppTextEditor(app=self.app, plain_text=True)

        # add the tab if it was closed
        self.app.ui.plot_tab_area.addTab(self.gcode_viewer_tab, '%s' % _("Code Viewer"))
        self.gcode_viewer_tab.setObjectName('code_viewer_tab')

        # delete the absolute and relative position and messages in the infobar
        self.app.ui.position_label.setText("")
        self.app.ui.rel_position_label.setText("")

        self.gcode_viewer_tab.code_editor.completer_enable = False
        self.gcode_viewer_tab.buttonRun.hide()

        # Switch plot_area to CNCJob tab
        self.app.ui.plot_tab_area.setCurrentWidget(self.gcode_viewer_tab)

        self.gcode_viewer_tab.t_frame.hide()
        # then append the text from GCode to the text editor
        try:
            self.gcode_viewer_tab.load_text(gco, move_to_start=True, clear_text=True)
        except Exception as e:
            self.app.log.error('FlatCAMCNCJob.on_edit_probing_gcode() -->%s' % str(e))
            return

        self.gcode_viewer_tab.t_frame.show()
        self.app.proc_container.view.set_idle()

        self.gcode_viewer_tab.buttonSave.hide()
        self.gcode_viewer_tab.buttonOpen.hide()
        self.gcode_viewer_tab.buttonPrint.hide()
        self.gcode_viewer_tab.buttonPreview.hide()
        self.gcode_viewer_tab.buttonReplace.hide()
        self.gcode_viewer_tab.sel_all_cb.hide()
        self.gcode_viewer_tab.entryReplace.hide()

        self.gcode_viewer_tab.button_update_code.show()

        # self.gcode_viewer_tab.code_editor.setReadOnly(True)

        self.gcode_viewer_tab.button_update_code.clicked.connect(self.on_update_probing_gcode)

        self.app.inform.emit('[success] %s...' % _('Loaded Machine Code into Code Viewer'))

    def on_update_probing_gcode(self):
        self.probing_gcode_text = self.gcode_viewer_tab.code_editor.toPlainText()

    def on_import_height_map(self):
        """
        Import the height map file into the app
        :return:
        :rtype:
        """

        _filter_ = "Text File .txt (*.txt);;All Files (*.*)"
        try:
            filename, _f = QtWidgets.QFileDialog.getOpenFileName(caption=_("Import Height Map"),
                                                                 directory=self.app.get_last_folder(),
                                                                 filter=_filter_)
        except TypeError:
            filename, _f = QtWidgets.QFileDialog.getOpenFileName(caption=_("Import Height Map"),
                                                                 filter=_filter_)

        filename = str(filename)

        if filename == '':
            self.app.inform.emit('[WARNING_NOTCL] %s' % _("Cancelled."))
        else:
            self.app.worker_task.emit({'fcn': self.import_height_map, 'params': [filename]})

    def import_height_map(self, filename):
        """

        :param filename:
        :type filename:
        :return:
        :rtype:
        """

        try:
            if filename:
                with open(filename, 'r') as f:
                    stream = f.readlines()
            else:
                return
        except IOError:
            self.app.log.error("Failed to open height map file: %s" % filename)
            self.app.inform.emit('[ERROR_NOTCL] %s: %s' % (_("Failed to open height map file"), filename))
            return

        if not stream:
            self.app.inform.emit('[ERROR_NOTCL] %s: %s' % (_("Empty file"), filename))
            return

        probed_points = []
        for line in stream:
            if not line.strip():
                continue
            parts = [p for p in line.replace(' ', ',').replace('\n', '').split(',') if p != '']
            try:
                probed_points.append((float(parts[0]), float(parts[1]), float(parts[2])))
            except (IndexError, ValueError):
                self.app.inform.emit('[ERROR_NOTCL] %s: %s' % (_("Failed to parse height map file"), filename))
                return

        if not probed_points:
            self.app.inform.emit('[ERROR_NOTCL] %s: %s' % (_("Empty file"), filename))
            return

        self.store_probed_points(probed_points)
        self.build_al_table_sig.emit()
        self.autolevell_gcode()

    def store_probed_points(self, probed_points):
        """
        Store the probed heights in both the Voronoi and the bilinear storage. The Voronoi cells that were already
        calculated are kept.

        :param probed_points:   list of (x, y, height) tuples, in the order in which the points were probed
        :type probed_points:    list
        """
        self.al_bilinear_geo_storage = [(x, y, height) for x, y, height in probed_points]

        for idx, (x, y, height) in enumerate(probed_points, 1):
            if idx not in self.al_voronoi_geo_storage:
                self.al_voronoi_geo_storage[idx] = {'point': Point((x, y)), 'geo': None}
            self.al_voronoi_geo_storage[idx]['height'] = height
            if 'point' not in self.al_voronoi_geo_storage[idx]:
                self.al_voronoi_geo_storage[idx]['point'] = Point((x, y))

    def on_grbl_autolevel(self):
        # show the Shell Dock
        self.app.ui.shell_dock.show()

        # the probing reads the answers of the controller; nothing else may do it meanwhile
        self.grbl_busy += 1
        self.grbl_refresh_controls()

        def worker_task():
            try:
                probe_and_level()
            finally:
                self.grbl_busy_done_sig.emit()

        def probe_and_level():
            with self.app.proc_container.new('%s...' % _("Sending")):
                self.grbl_probe_result = ''
                pr_travelz = str(self.ui.ptravelz_entry.get_value())
                probe_fr = str(self.ui.feedrate_probe_entry.get_value())
                pr_depth = str(self.ui.pdepth_entry.get_value())

                cmd = {'IN': 'G20', 'MM': 'G21'}[self.units]
                self.send_grbl_command(command=cmd)
                cmd = 'G90'
                self.send_grbl_command(command=cmd)

                # GRBL reports the probed point in machine coordinates and the job is in work coordinates; a
                # machine Z taken for a height moves every cut by the distance of the work Z zero to the machine
                # zero, tens of mm down into the material
                wco = self.grbl_read_work_offset()
                if wco is None:
                    self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller did not report the work "
                                                                "offset. Nothing was probed."))
                    return

                probed_points = []
                for x, y in self.probe_points():
                    cmd = 'G0 Z%s' % pr_travelz
                    self.send_grbl_command(command=cmd)
                    cmd = 'G0 X%s Y%s' % (str(x), str(y))
                    self.send_grbl_command(command=cmd)
                    cmd = 'G38.2 Z%s F%s' % (pr_depth, probe_fr)
                    output = self.send_grbl_command(command=cmd, timeout=120.0)

                    self.grbl_probe_result += output + '\n'

                    machine_z = self.parse_probe_answer(output)
                    if machine_z is None:
                        self.app.inform.emit(
                            '[ERROR_NOTCL] %s: %s, %s' % (_("Probing failed in point"), str(x), str(y))
                        )
                        return
                    probed_points.append((x, y, self.grbl_work_height(machine_z, wco[2], self.units)))

                cmd = 'G0 Z%s' % pr_travelz
                self.send_grbl_command(command=cmd)
                # no program end here: GRBL resets every modal state to its default when a program ends, which
                # among other things selects G54 again and stops the spindle, and the probing is only a step
                # before the job itself is sent over the same connection
                self.app.inform.emit('%s' % _("Finished probing. Doing the autolevelling."))

                self.store_probed_points(probed_points)
                self.build_al_table_sig.emit()

                # apply autolevelling here
                self.on_grbl_apply_autolevel()

        self.app.inform.emit('%s' % _("Sending probing GCode to the GRBL controller."))
        self.app.worker_task.emit({'fcn': worker_task, 'params': []})

    def grbl_read_work_offset(self, timeout=10.0):
        """
        The offset of the work coordinates from the machine coordinates, as the controller reports it now. GRBL
        includes it in a status report only every few reports, therefore the one that was kept from earlier is
        dropped and a new one is waited for.

        :param timeout: for how long to wait for the report, in seconds
        :type timeout:  float
        :return:        the X, Y and Z offset in mm, or None when the controller did not report it
        :rtype:         list or None
        """
        self.grbl_status['wco'] = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.grbl_read_status()
            if self.grbl_status['wco'] is not None:
                return list(self.grbl_status['wco'])
        return None

    @staticmethod
    def grbl_work_height(machine_z, wco_z, units):
        """
        The height of a probed point in the work coordinates the job uses.

        :param machine_z:   the Z of the [PRB:...] answer, in machine coordinates, in mm
        :type machine_z:    float
        :param wco_z:       the Z of the work offset, in mm
        :type wco_z:        float
        :param units:       the units of the job, 'MM' or 'IN'
        :type units:        str
        :return:            the height relative to the work Z zero, in the units of the job
        :rtype:             float
        """
        height = machine_z - wco_z
        return height / 25.4 if units == 'IN' else height

    def on_grbl_heightmap_save(self):
        if self.grbl_probe_result != '':
            _filter_ = "Text File .txt (*.txt);;All Files (*.*)"
            name = "probing_gcode"
            try:
                dir_file_to_save = self.app.get_last_save_folder() + '/' + str(name)
                filename, _f = FCFileSaveDialog.get_saved_filename(
                    caption=_("Export Code ..."),
                    directory=dir_file_to_save,
                    ext_filter=_filter_
                )
            except TypeError:
                filename, _f = FCFileSaveDialog.get_saved_filename(
                    caption=_("Export Code ..."),
                    ext_filter=_filter_)

            if filename == '':
                self.app.inform.emit('[WARNING_NOTCL] %s' % _("Export cancelled ..."))
                return
            else:
                try:
                    force_windows_line_endings = self.app.options['cncjob_line_ending']
                    if force_windows_line_endings and sys.platform != 'win32':
                        with open(filename, 'w', newline='\r\n') as f:
                            for line in self.grbl_probe_result:
                                f.write(line)
                    else:
                        with open(filename, 'w') as f:
                            for line in self.grbl_probe_result:
                                f.write(line)
                except FileNotFoundError:
                    self.app.inform.emit('[WARNING_NOTCL] %s' % _("No such file or directory"))
                    return
                except PermissionError:
                    self.app.inform.emit(
                        '[WARNING] %s' % _("Permission denied, saving not possible.\n"
                                           "Most likely another app is holding the file open and not accessible.")
                    )
                    return 'fail'
        else:
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("Empty GRBL heightmap."))

    def on_grbl_apply_autolevel(self):
        self.autolevell_gcode()

    def on_grbl_send_job(self, start_line=1):
        """
        Send the GCode of the selected CNCJob object to the GRBL controller.

        :param start_line:  the number of the first line to send
        :type start_line:   int
        """
        target_obj = self.app.collection.get_by_name(self.ui.object_combo.get_value())
        if target_obj is None:
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("Could not retrieve object"))
            return

        try:
            job_gcode = target_obj.source_file
        except AttributeError:
            job_gcode = ''

        if not job_gcode:
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("There is no GCode to send."))
            return

        if not self.grbl_is_connected():
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller is not connected."))
            return

        if self.grbl_job_running or self.grbl_busy:
            return

        # a controller in an alarm refuses the first line of the job; after a power up that is the homing lock
        if self.grbl_check_manual() is None:
            return
        if self.grbl_limits_suspended:
            if any(axis in self.grbl_status['pins'] for axis in 'XYZ'):
                self.app.inform.emit('[ERROR_NOTCL] %s' % _("A limit switch is pressed, jog away from it first."))
                return
            if not self.grbl_restore_hard_limits():
                return
        if not self.grbl_job_fits(job_gcode):
            return

        # show the Shell Dock
        self.app.ui.shell_dock.show()

        self.grbl_job_stop = False
        self.grbl_job_waiting = False
        self.grbl_job_running = True
        self.grbl_refresh_controls()

        def worker_task():
            with self.app.proc_container.new('%s...' % _("Sending")):
                self.grbl_stream_gcode(job_gcode, start_line=start_line)

        self.app.worker_task.emit({'fcn': worker_task, 'params': []})

    @staticmethod
    def grbl_job_top_z(job_gcode):
        """
        The highest Z a job moves to, in work coordinates.

        :param job_gcode:   the GCode of the job
        :type job_gcode:    str
        :return:            the Z, or None when the job has no Z or uses relative moves, inch or offsets, which
                            make it not known without running it
        :rtype:             float or None
        """
        heights = []
        for line in job_gcode.splitlines():
            code = re.sub(r"\(.*?\)|;.*", '', line).upper()
            if re.search(r"G0*91(?![.\d])|G0*20(?!\d)|G0*92|G0*5[35-9](?!\d)|G0*10(?!\d)", code):
                return None
            heights += [float(value) for axis, value in ToolLevelling.coord_re.findall(code) if axis == 'Z']
        return max(heights) if heights else None

    def grbl_job_fits(self, job_gcode):
        """
        Find out that the job does not go above the top of the machine, where the limit switch is: hitting it
        ends the job with an alarm.

        :param job_gcode:   the GCode of the job
        :type job_gcode:    str
        :return:            False when the job would run into the switch
        :rtype:             bool
        """
        ceiling = self.grbl_ceilings()[2]
        wco = self.grbl_status['wco']
        top_z = self.grbl_job_top_z(job_gcode)
        if self.units != 'MM' or ceiling is None or wco is None or top_z is None:
            return True
        if top_z + wco[2] <= ceiling + 0.001:
            return True
        self.app.inform.emit('[ERROR_NOTCL] %s' % (
            _("The job moves up to Z %.3f, which is %.3f mm above the top of the machine. Set the work Z zero "
              "lower, or make the travel and end Z of the job smaller.") % (top_z, top_z + wco[2] - ceiling)))
        return False

    def on_grbl_resume_job(self):
        """
        Send the job again, starting with the line the user chose.
        """
        self.on_grbl_send_job(start_line=self.ui.grbl_resume_line_entry.get_value())

    def on_grbl_job_progress(self, text):
        self.ui.grbl_job_progress_label.set_value(text)

    def on_grbl_job_paused(self, message):
        """
        Tell the operator what the machine waits for at a program pause.
        """
        self.ui.grbl_job_progress_label.set_value(message)
        self.app.inform.emit('[WARNING_NOTCL] %s' % message)

        # the machine stands still and the streaming thread is not reading the port, therefore the tool can be
        # changed, the machine jogged and the new tool touched off
        self.grbl_refresh_controls()

    def grbl_refresh_controls(self):
        """
        Make usable what can be used in the state the sender is in, and nothing else.

        The answers of the controller can only be read by one at a time: by the thread that streams a job, by a
        command that runs in the background, or by a button pressed by hand. A job that waits at a program pause
        does not read the port, therefore the machine can be operated by hand meanwhile.
        """
        streaming = self.grbl_job_running and not self.grbl_job_waiting
        busy = self.grbl_busy > 0
        manual = not busy and not streaming

        self.on_grbl_manual_control(manual)
        self.ui.grbl_send_job_button.setDisabled(busy or self.grbl_job_running)
        self.ui.grbl_stop_job_button.setDisabled(not self.grbl_job_running)
        self.ui.grbl_continue_button.setDisabled(busy or not (self.grbl_job_running and self.grbl_job_waiting))
        self.ui.grbl_resume_job_button.setDisabled(busy or self.grbl_job_running or not self.grbl_can_resume)

        if manual and not self.grbl_job_running:
            self.on_levelling_available(self.can_be_levelled(
                self.app.collection.get_by_name(self.ui.object_combo.get_value())))
        else:
            # probing the surface in the middle of a job would take the place of the rest of it
            self.ui.grbl_get_heightmap_button.setDisabled(True)

    def on_grbl_manual_control(self, enabled):
        """
        Everything that moves the machine or reads an answer from it by hand goes through send_grbl_command,
        which reads the replies of the controller. One of those pressed while a job streams would consume an
        acknowledgement that the streaming thread counts on, therefore they are only usable when the machine
        is not being fed.

        Stop, Pause/Resume and the controller Reset are not in this set: they write to the port without
        reading it and have to stay reachable at all times.

        :param enabled: whether the machine may be operated by hand
        :type enabled:  bool
        """
        self.ui.jog_wdg.setDisabled(not enabled)
        self.ui.zero_axs_wdg.setDisabled(not enabled)
        self.ui.probe_z_button.setDisabled(not enabled)
        self.ui.grbl_unlock_button.setDisabled(not enabled)
        self.ui.grbl_command_entry.setDisabled(not enabled)
        self.ui.grbl_send_button.setDisabled(not enabled)
        self.ui.grbl_report_button.setDisabled(not enabled)
        self.ui.grbl_parameter_entry.setDisabled(not enabled)
        self.ui.grbl_get_param_button.setDisabled(not enabled)

        if enabled is False:
            # probing the surface needs the port to itself; it is not enabled again here because whether it is
            # usable at all depends on the selected object
            self.ui.grbl_get_heightmap_button.setDisabled(True)

    def on_grbl_continue_job(self):
        """
        Let a job that stopped at a program pause continue.
        """
        if self.grbl_job_waiting is False:
            return

        # nothing has to be released: the pause was never sent to the controller, therefore the machine is
        # only waiting for the next line
        self.grbl_job_waiting = False
        self.grbl_refresh_controls()
        self.app.inform.emit('%s' % _("The job continues."))

    def on_grbl_job_finished(self):
        self.grbl_job_running = False
        self.grbl_job_waiting = False

        # a job that did not run to its end can be resumed from the line that follows the last one the
        # controller acknowledged
        resume_from = self.grbl_job_last_ack + 1
        self.grbl_can_resume = 1 < resume_from <= len(self.grbl_job_lines)
        if self.grbl_can_resume:
            self.ui.grbl_resume_line_entry.set_value(resume_from)

        # the machine is free again, therefore it can be moved by hand and the levelling of the selected object
        # is available once more
        self.grbl_refresh_controls()

    def on_grbl_stop_job(self):
        """
        Stop the machine and the sending of the job.
        """
        if not self.grbl_job_running:
            return

        self.grbl_job_stop = True
        self.grbl_job_waiting = False
        try:
            # feed hold first: it brings the machine to a controlled stop without losing its position. It is a
            # realtime command that the controller does not acknowledge and reading the port here would take
            # the answers that the thread which streams the job waits for.
            self.grbl_ser_port.write(b'!')
        except Exception as e:
            self.app.log.error("ToolLevelling.on_grbl_stop_job() --> %s" % str(e))
            return

        self.ui.pause_resume_button.setChecked(False)
        self.app.inform.emit('[WARNING_NOTCL] %s' % _("Sending the job was stopped."))
        self.grbl_run_in_background(self.grbl_finish_stop)

    def grbl_finish_stop(self, timeout=30.0):
        """
        Throw away the rest of the job the controller holds. A held controller still has the lines it was given
        and would move on with them at the next cycle start, and it refuses to jog. A soft reset is the only way
        to empty its buffers; once the machine came to a stop it does not cost the position.
        """
        deadline = time.monotonic() + timeout
        # the thread that streams the job is still reading the port for a moment
        time.sleep(0.3)

        while time.monotonic() < deadline:
            state = self.grbl_read_status()
            if state is not None and (state.startswith('hold:0') or state.startswith('idle') or
                                      state.startswith('alarm') or state.startswith('door')):
                break

        self.grbl_resets += 1
        try:
            self.grbl_ser_port.write(b'\x18')
        except Exception as e:
            self.app.log.error("ToolLevelling.grbl_finish_stop() --> %s" % str(e))
            return

        self.grbl_report_after_reset()

    @staticmethod
    def grbl_extract_message(gcode_line):
        """
        The text that a (MSG, ...) comment addresses to the operator.

        :param gcode_line:  a line of GCode
        :type gcode_line:   str
        :return:            the text of the message or None when the line holds no message
        :rtype:             str or None
        """
        found = ToolLevelling.msg_re.search(gcode_line)
        return found.group(1) if found else None

    @staticmethod
    def grbl_prepare_job(job_gcode):
        """
        Turn a GCode program into the lines that are to be sent to a GRBL controller.

        Comments are not sent because GRBL has no use for them, but the text of a (MSG, ...) comment is kept and
        attached to the line that follows it so that it can be shown to the operator. The M6 word is taken out
        because GRBL does not support it and would reject the whole line it is on.

        :param job_gcode:   the GCode program
        :type job_gcode:    str
        :return:            a (line, message, pauses the machine) tuple for every line that is to be sent
        :rtype:             list
        """
        prepared = []
        pending_message = None

        for raw_line in job_gcode.splitlines():
            line = raw_line.strip()
            if not line:
                continue

            message = ToolLevelling.grbl_extract_message(line)
            if message:
                pending_message = message

            line = ToolLevelling.comment_re.sub(' ', line).split(';')[0]
            line = ToolLevelling.m6_re.sub(' ', line)

            # the pause word is not sent either: the sender holds the job itself, which leaves the machine Idle
            # so that it can be jogged and a new tool can be touched off
            pauses = bool(ToolLevelling.pause_re.search(line))
            line = ToolLevelling.pause_re.sub(' ', line)

            line = ' '.join(line.split())
            if not line and not pauses:
                continue

            prepared.append((line, pending_message, pauses))
            pending_message = None

        return prepared

    @staticmethod
    def grbl_resume_preamble(sent_lines, modal_only=False):
        """
        The commands that restore the state of the controller before a job is resumed from somewhere in the
        middle of it. The lines that are skipped carry that state and are scanned for it.

        The tool is lifted to the highest Z the skipped lines used, which is the travel or tool change height,
        so that the first move of the resumed job does not drag the tool through the material.

        :param sent_lines:  the lines of the job that are skipped, as they come out of grbl_prepare_job()
        :type sent_lines:   list
        :param modal_only:  only the modal commands, which is what a job that waited at a program pause needs:
                            it was never interrupted, but the operator may have left the controller in another
                            distance or units mode
        :type modal_only:   bool
        :return:            the commands to send before the job continues
        :rtype:             list
        """
        preamble = []
        upper_lines = [line.upper() for line in sent_lines]

        # a modal command stays in effect until another one of its group replaces it
        for group in ToolLevelling.modal_groups:
            for line in reversed(upper_lines):
                in_line = next((code for code in group if re.search(r"\b%s\b" % code, line)), None)
                if in_line:
                    preamble.append(in_line)
                    break

        if modal_only:
            return preamble

        for line in reversed(upper_lines):
            found = ToolLevelling.feed_re.search(line)
            if found:
                preamble.append('F%s' % found.group(1))
                break

        # the spindle is only started again when it was not stopped by the last command that addressed it
        spindle_dir = None
        for line in reversed(upper_lines):
            found = ToolLevelling.spindle_re.search(line)
            if found:
                spindle_dir = found.group(1)
                break

        if spindle_dir in ('3', '4'):
            speed = None
            for line in reversed(upper_lines):
                found = ToolLevelling.speed_re.search(line)
                if found:
                    speed = found.group(1)
                    break
            preamble.append('M0%s S%s' % (spindle_dir, speed) if speed else 'M0%s' % spindle_dir)

        heights = [float(value) for line in upper_lines
                   for axis, value in ToolLevelling.coord_re.findall(line) if axis == 'Z']
        if heights:
            preamble.append('G00 Z%s' % max(heights))

        return preamble

    def grbl_next_response(self, pending, timeout):
        """
        The next answer line of the controller.

        The answers are buffered because the controller may answer several of the streamed lines at once and
        none of those answers may be lost. An answer that was read before all of it arrived is kept until the
        rest of it does.

        :param pending: buffers the answers that were read from the port but not consumed yet
        :type pending:  list
        :param timeout: for how long to wait for an answer, in seconds
        :type timeout:  float
        :return:        the answer or None when the controller did not answer in time
        :rtype:         str or None
        """
        deadline = time.monotonic() + timeout

        while True:
            while pending:
                response = pending.pop(0).strip()
                if response:
                    return response

            if self.grbl_job_stop is True or time.monotonic() >= deadline:
                return None

            raw = self.grbl_ser_port.readlines()
            if raw:
                raw[0] = self.grbl_rx_partial + raw[0]
                self.grbl_rx_partial = b''
                if not raw[-1].endswith(b'\n'):
                    self.grbl_rx_partial = raw.pop()
                decoded = self.grbl_decode(raw)
                self.grbl_note_lines(decoded)
                pending.extend(decoded)

            if not pending:
                time.sleep(0.01)

    def grbl_collect_ack(self, pending, in_flight):
        """
        Wait for the controller to answer and account for the oldest line that it did not acknowledge yet.

        Everything that is not an acknowledgement, a push message of the controller for instance, is shown in
        the shell and does not count as the answer to a line.

        :param pending:     buffers the answers that were read from the port but not consumed yet
        :type pending:      list
        :param in_flight:   the lines that were sent but not acknowledged yet, as (line number, bytes) pairs
        :type in_flight:    list
        :return:            True when a line was acknowledged
        :rtype:             bool
        """
        while True:
            if self.grbl_job_stop is True:
                return False

            response = self.grbl_next_response(pending, timeout=self.grbl_job_line_timeout)
            if response is None:
                if self.grbl_job_stop is True:
                    # the operator stopped the job; that is not a controller that went silent
                    return False
                self.app.inform.emit(
                    '[ERROR_NOTCL] %s %d' % (_("The GRBL controller did not answer for line"),
                                             in_flight[0][0] if in_flight else 0)
                )
                return False

            low_response = response.lower()

            if low_response.startswith('ok'):
                if in_flight:
                    self.grbl_job_last_ack = in_flight.pop(0)[0]
                return True

            if low_response.startswith('error') or low_response.startswith('alarm'):
                line_nr = in_flight.pop(0)[0] if in_flight else 0
                offender = self.grbl_job_lines[line_nr - 1][0] if 0 < line_nr <= len(self.grbl_job_lines) else ''
                self.app.inform_shell.emit('\t\t\t: %s' % response.upper())
                self.app.inform.emit(
                    '[ERROR_NOTCL] %s %d: %s' % (_("The GRBL controller rejected the line"), line_nr, offender)
                )
                return False

            # the controller has something to say that is not about the line that was sent; the status reports
            # that the position display asks for are kept but not shown
            if not response.startswith('<'):
                self.app.inform_shell.emit('\t\t\t: %s' % response.upper())

    def grbl_read_status(self, timeout=2.0):
        with self.grbl_port_reader():
            return self._grbl_read_status(timeout=timeout)

    def _grbl_read_status(self, timeout=2.0):
        """
        The state that the controller reports. Nothing may be waiting for an acknowledgement when this is
        called because the status reports are read from the same port.

        :param timeout: for how long to wait for the report, in seconds
        :type timeout:  float
        :return:        the state in lower case, 'idle', 'run', 'hold:0' or 'alarm' for instance, or None when
                        the controller did not report in time
        :rtype:         str or None
        """
        deadline = time.monotonic() + timeout
        partial = b''

        # a status report is asked for; it is a realtime command that is not acknowledged with 'ok'
        self.grbl_ser_port.write('?'.encode('utf-8'))

        while time.monotonic() < deadline:
            time.sleep(0.1)

            raw = self.grbl_ser_port.readlines()
            if raw:
                raw[0] = partial + raw[0]
                partial = b''
                if not raw[-1].endswith(b'\n'):
                    partial = raw.pop()

            decoded = self.grbl_decode(raw)
            self.grbl_note_lines(decoded)
            state = None
            for response in decoded:
                stripped = response.strip()
                if stripped.startswith('<'):
                    # the last one counts: the ones before answer what the position display asked for earlier
                    state = stripped[1:].split('|')[0].strip().lower()
                elif stripped and stripped.lower() != 'ok':
                    # what the controller says on its own, a limit alarm for instance
                    self.app.inform_shell.emit('\t\t\t: ' + stripped.upper())
            if state is not None:
                return state

        return None

    def grbl_wait_for_idle(self, timeout=120.0):
        """
        Poll the controller until it stopped moving, which is what it does once it worked through everything
        that was sent to it. An alarm also ends the wait, because a machine in that state will never reach Idle.

        :param timeout: for how long to wait for the machine to come to a stop, in seconds
        :type timeout:  float
        :return:        True when the machine stopped
        :rtype:         bool
        """
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            if self.grbl_job_stop is True:
                return False

            state = self.grbl_read_status()
            if state is not None and (state.startswith('idle') or state.startswith('alarm')):
                return True

        return False

    def grbl_wait_for_operator(self, message=None):
        """
        Stop streaming until the operator acted on a program pause, a tool change for instance, and pressed the
        Continue button.

        :param message: what the preprocessor asks the operator to do
        :type message:  str or None
        :return:        True when the job may continue
        :rtype:         bool
        """
        self.grbl_wait_for_idle()

        if self.grbl_job_stop is True:
            return False

        self.grbl_job_waiting = True
        self.grbl_job_paused_sig.emit(
            message if message else _("The job is paused. Press Continue when the machine is ready."))

        while self.grbl_job_waiting is True and self.grbl_job_stop is False:
            time.sleep(0.2)

        self.grbl_job_waiting = False
        return self.grbl_job_stop is False

    def grbl_resume_after_pause(self, sent_lines, pending):
        """
        Put the controller back into the state the job needs after the operator worked at the machine during a
        program pause.

        :param sent_lines:  the lines of the job that were sent so far
        :type sent_lines:   list
        :param pending:     buffers the answers that were read from the port but not consumed yet
        :type pending:      list
        :return:            True when the job may continue
        :rtype:             bool
        """
        # a jog, a zeroing or a probe that the operator ran during the pause left answers in the port, and
        # none of them may be taken for the acknowledgement of a streamed line
        try:
            self.grbl_ser_port.reset_input_buffer()
        except Exception as e:
            self.app.log.error("ToolLevelling.grbl_resume_after_pause() --> %s" % str(e))
        del pending[:]
        self.grbl_rx_partial = b''

        # the operator may have left another distance or units mode in effect, a failed probe leaves G91 for
        # instance, which would turn the rest of the job into relative moves
        for command in self.grbl_resume_preamble(sent_lines, modal_only=True):
            if self.grbl_job_stop is True:
                return False
            if not self.send_grbl_command(command=command, timeout=self.grbl_job_line_timeout):
                self.app.inform.emit(
                    '[ERROR_NOTCL] %s: %s' % (_("The GRBL controller rejected the line"), command))
                return False

        return True

    def grbl_stream_gcode(self, job_gcode, start_line=1):
        """
        Send a GCode program to the GRBL controller.

        The lines are streamed with the character counting protocol: as many of them are sent as fit in the
        receive buffer of the controller, so that its planner is kept full and a job that is made of many short
        segments is not slowed down by the round trip of every single line. A program pause stops the streaming
        until the operator acted on it.

        :param job_gcode:   the GCode program
        :type job_gcode:    str
        :param start_line:  the number of the first line to send; the lines before it are only scanned for the
                            state that has to be restored before the job continues
        :type start_line:   int
        """
        prepared = self.grbl_prepare_job(job_gcode)
        self.grbl_job_lines = prepared
        self.grbl_job_last_ack = start_line - 1
        self.grbl_rx_partial = b''

        total = len(prepared)
        if start_line > total:
            self.app.inform.emit('[ERROR_NOTCL] %s: %d' % (_("The job has no line"), start_line))
            self.grbl_job_finished_sig.emit()
            return

        # whatever is left in the port from an earlier command would be taken for an acknowledgement
        self.grbl_ser_port.reset_input_buffer()

        # the answers that were read from the port but not consumed yet
        pending = []
        # the lines that were sent but not acknowledged yet, as (line number, length in bytes) pairs
        in_flight = []

        try:
            if start_line > 1:
                self.app.inform.emit('%s: %d' % (_("Resuming the job from line"), start_line))
                for command in self.grbl_resume_preamble([line for line, _msg, _p in prepared[:start_line - 1]]):
                    if self.grbl_job_stop is True:
                        return
                    if not self.send_grbl_command(command=command, timeout=self.grbl_job_line_timeout):
                        self.app.inform.emit(
                            '[ERROR_NOTCL] %s: %s' % (_("The GRBL controller rejected the line"), command))
                        return
            else:
                self.app.inform.emit('%s: %d' % (_("Sending the job to the GRBL controller. Lines"), total))

            for idx, (line, message, pauses) in enumerate(prepared, 1):
                if idx < start_line:
                    continue

                if self.grbl_job_stop is True:
                    return

                # a paused machine is given time to be resumed before the next line is sent
                while self.ui.pause_resume_button.isChecked() and self.grbl_job_stop is False:
                    time.sleep(0.2)

                if self.grbl_job_stop is True:
                    return

                if message:
                    # what the preprocessor has to say to the operator
                    self.app.inform_shell.emit('%s: %s' % (_("Message"), message))
                    self.grbl_job_progress_sig.emit(message)

                if not line:
                    # the line held nothing but a program pause, which is handled below
                    pass
                elif line.startswith('$'):
                    # a system command answers with more than an acknowledgement and it is not queued by the
                    # controller, therefore it is sent on its own and its whole answer is read
                    while in_flight:
                        if self.grbl_collect_ack(pending, in_flight) is False:
                            return
                    if not self.send_grbl_command(command=line, timeout=self.grbl_job_line_timeout):
                        self.app.inform.emit(
                            '[ERROR_NOTCL] %s %d: %s' % (_("The GRBL controller rejected the line"), idx, line))
                        return
                    self.grbl_job_last_ack = idx
                else:
                    sent_bytes = len(line) + 1
                    # only as many lines are sent as the receive buffer of the controller can still hold
                    while in_flight and sum(count for _nr, count in in_flight) + sent_bytes > self.grbl_rx_buffer:
                        if self.grbl_collect_ack(pending, in_flight) is False:
                            return

                    self.grbl_ser_port.write((line + '\n').encode('utf-8'))
                    in_flight.append((idx, sent_bytes))

                # a job has many lines therefore the progress is not reported for every one of them
                if idx == total or idx % 25 == 0:
                    self.grbl_job_progress_sig.emit('%s: %d / %d' % (_("Line"), idx, total))

                if pauses:
                    # the machine only comes to a stop once it worked through everything that was sent
                    while in_flight:
                        if self.grbl_collect_ack(pending, in_flight) is False:
                            return
                    self.grbl_job_last_ack = idx

                    while True:
                        if self.grbl_wait_for_operator(message) is False:
                            return

                        # a job cannot be continued from an alarm: the machine lost its position and every
                        # move it is given would be refused
                        state = self.grbl_read_status()
                        if state is None or not state.startswith('alarm'):
                            break

                        self.app.inform.emit(
                            '[ERROR_NOTCL] %s' % _("The controller is in an alarm state. Unlock it with $X, "
                                                   "set the Z origin again, then continue.")
                        )

                    sent_lines = [sent for sent, _msg, _p in prepared[:idx]]
                    if self.grbl_resume_after_pause(sent_lines, pending) is False:
                        return

            # the job is only done once the controller acknowledged the last line of it
            while in_flight:
                if self.grbl_collect_ack(pending, in_flight) is False:
                    return

            self.app.inform.emit('[success] %s' % _("The job was sent to the GRBL controller."))
        except Exception as e:
            self.app.log.error("ToolLevelling.grbl_stream_gcode() --> %s" % str(e))
            self.app.inform.emit('[ERROR_NOTCL] %s: %s' % (_("Sending the job failed"), str(e)))
        finally:
            self.grbl_job_finished_sig.emit()

    def probe_points(self):
        """
        The (x, y) points that are to be probed, taken from the storage of the active levelling method.

        :return:    list of (x, y) tuples
        :rtype:     list
        """
        if self.ui.al_method_radio.get_value() == 'b':
            return [(pt[0], pt[1]) for pt in self.al_bilinear_geo_storage]
        return [
            (val['point'].x, val['point'].y) for val in self.al_voronoi_geo_storage.values()
        ]

    def parse_probe_answer(self, answer):
        """
        Extract the probed height from the answer of the GRBL controller. A probe that did not touch the material is
        reported by GRBL with a success flag of zero.

        :param answer:  the answer of the controller to a G38.2 command
        :type answer:   str
        :return:        the probed height or None if the probe did not trigger
        :rtype:         float
        """
        match = self.prb_re.search(answer or '')
        if not match or match.group(4) == '0':
            return None
        return float(match.group(3))

    def ui_connect(self):
        self.ui.al_add_button.clicked.connect(self.on_add_al_probepoints)
        self.ui.show_al_table.stateChanged.connect(self.on_show_al_table)

    def ui_disconnect(self):
        try:
            self.ui.al_add_button.clicked.disconnect()
        except (TypeError, AttributeError):
            pass

        try:
            self.ui.show_al_table.stateChanged.disconnect()
        except (TypeError, AttributeError):
            pass

    def reset_fields(self):
        self.ui.object_combo.setRootModelIndex(self.app.collection.index(0, 0, QtCore.QModelIndex()))


class LevelUI:
    pluginName = _("Levelling")

    def __init__(self, layout, app):
        self.app = app
        self.decimals = self.app.decimals
        self.layout = layout

        self.tools_frame = QtWidgets.QFrame()
        self.tools_frame.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.tools_frame)
        self.tools_box = QtWidgets.QVBoxLayout()
        self.tools_box.setContentsMargins(0, 0, 0, 0)
        self.tools_frame.setLayout(self.tools_box)

        self.title_box = QtWidgets.QHBoxLayout()
        self.tools_box.addLayout(self.title_box)

        # ## Title
        title_label = FCLabel("%s" % self.pluginName, size=16, bold=True)
        title_label.setToolTip(
            _("Generate CNC Code with auto-levelled paths.")
        )

        self.title_box.addWidget(title_label)

        # App Level label
        self.level = QtWidgets.QToolButton()
        self.level.setToolTip(
            _(
                "Beginner Mode - many parameters are hidden.\n"
                "Advanced Mode - full control.\n"
                "Permanent change is done in 'Preferences' menu."
            )
        )
        # self.level.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.level.setCheckable(True)
        self.title_box.addWidget(self.level)

        self.obj_combo_label = FCLabel('%s' % _("Source Object"), color='darkorange', bold=True)
        self.obj_combo_label.setToolTip(
            _("CNCJob source object to be levelled.")
        )

        self.tools_box.addWidget(self.obj_combo_label)

        # #############################################################################################################
        # ################################ The object to be Auto-levelled ##############################################
        # #############################################################################################################
        self.object_combo = FCComboBox()
        self.object_combo.setModel(self.app.collection)
        self.object_combo.setRootModelIndex(self.app.collection.index(3, 0, QtCore.QModelIndex()))
        # self.object_combo.setCurrentIndex(1)
        self.object_combo.is_last = True

        self.tools_box.addWidget(self.object_combo)

        # separator_line = QtWidgets.QFrame()
        # separator_line.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        # separator_line.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        # self.tools_box.addWidget(separator_line)

        # Autolevelling
        self.al_frame = QtWidgets.QFrame()
        self.al_frame.setContentsMargins(0, 0, 0, 0)
        self.tools_box.addWidget(self.al_frame)

        self.al_box = QtWidgets.QVBoxLayout()
        self.al_box.setContentsMargins(0, 0, 0, 0)
        self.al_frame.setLayout(self.al_box)
        self.al_frame.setDisabled(True)

        grid0 = GLay(v_spacing=5, h_spacing=3)
        self.al_box.addLayout(grid0)

        self.al_title = FCLabel('%s' % _("Probe Points Table"), bold=True)
        self.al_title.setToolTip(_("Generate GCode that will obtain the height map"))

        self.show_al_table = FCCheckBox(_("Show"))
        self.show_al_table.setToolTip(_("Toggle the display of the Probe Points table."))
        self.show_al_table.setChecked(True)

        hor_lay = QtWidgets.QHBoxLayout()
        hor_lay.addWidget(self.al_title)
        hor_lay.addStretch()
        hor_lay.addWidget(self.show_al_table, alignment=QtCore.Qt.AlignmentFlag.AlignRight)

        grid0.addLayout(hor_lay, 0, 0, 1, 2)

        # #############################################################################################################
        # Tool Table Frame
        # #############################################################################################################
        tt_frame = FCFrame()
        self.al_box.addWidget(tt_frame)

        # Grid Layout
        tool_grid = GLay(v_spacing=5, h_spacing=3, c_stretch=[0, 1])
        tt_frame.setLayout(tool_grid)

        # Probe Points table
        self.al_probe_points_table = FCTable()
        self.al_probe_points_table.setColumnCount(3)
        self.al_probe_points_table.setColumnWidth(0, 20)
        self.al_probe_points_table.setHorizontalHeaderLabels(['#', _('X-Y Coordinates'), _('Height')])

        tool_grid.addWidget(self.al_probe_points_table, 0, 0, 1, 2)

        # Plot Probe Points
        self.plot_probing_pts_cb = FCCheckBox(_("Plot probing points"))
        self.plot_probing_pts_cb.setToolTip(
            _("Plot the probing points in the table.\n"
              "If a Voronoi method is used then\n"
              "the Voronoi areas are also plotted.")
        )
        tool_grid.addWidget(self.plot_probing_pts_cb, 2, 0, 1, 2)

        # Avoid Excellon holes
        self.avoid_exc_holes_cb = FCCheckBox(_("Avoid Excellon holes"))
        self.avoid_exc_holes_cb.setToolTip(
            _("When active, the user cannot add probe points over a drill hole.")
        )
        tool_grid.addWidget(self.avoid_exc_holes_cb, 4, 0, 1, 2)

        # Avoid Excellon holes Size
        self.avoid_exc_holes_size_label = FCLabel('%s:' % _("Avoid Step"))
        self.avoid_exc_holes_size_label.setToolTip(
            _("The incremental size to move to the side, to avoid an Excellon hole.")
        )

        self.avoid_exc_holes_size_entry = FCDoubleSpinner()
        self.avoid_exc_holes_size_entry.set_precision(self.decimals)
        self.avoid_exc_holes_size_entry.set_range(0.0000, 99999.0000)

        tool_grid.addWidget(self.avoid_exc_holes_size_label, 6, 0)
        tool_grid.addWidget(self.avoid_exc_holes_size_entry, 6, 1, 1, 1)

        # #############################################################################################################
        # ############### Probe GCode Generation ######################################################################
        # #############################################################################################################
        self.probe_gc_label = FCLabel('%s' % _("Parameters"), color='blue', bold=True)
        self.probe_gc_label.setToolTip(
            _("Will create a GCode which will be sent to the controller,\n"
              "either through a file or directly, with the intent to get the height map\n"
              "that is to modify the original GCode to level the cutting height.")
        )
        self.al_box.addWidget(self.probe_gc_label)

        tp_frame = FCFrame()
        self.al_box.addWidget(tp_frame)

        # Grid Layout
        param_grid = GLay(v_spacing=5, h_spacing=3)
        tp_frame.setLayout(param_grid)

        # Probe Diameter
        self.probe_tip_dia_label = FCLabel('%s:' % _("Probe Tip Dia"))
        self.probe_tip_dia_label.setToolTip(
            _("The probe tip diameter.")
        )
        self.probe_tip_dia_entry = FCDoubleSpinner()
        self.probe_tip_dia_entry.set_precision(self.decimals)
        self.probe_tip_dia_entry.set_range(0.0000, 10.0000)

        param_grid.addWidget(self.probe_tip_dia_label, 0, 0)
        param_grid.addWidget(self.probe_tip_dia_entry, 0, 1)

        # Travel Z Probe
        self.ptravelz_label = FCLabel('%s:' % _("Probe Z travel"))
        self.ptravelz_label.setToolTip(
            _("The safe Z for probe travelling between probe points.")
        )
        self.ptravelz_entry = FCDoubleSpinner()
        self.ptravelz_entry.set_precision(self.decimals)
        self.ptravelz_entry.set_range(0.0000, 10000.0000)

        param_grid.addWidget(self.ptravelz_label, 2, 0)
        param_grid.addWidget(self.ptravelz_entry, 2, 1)

        # Probe depth
        self.pdepth_label = FCLabel('%s:' % _("Probe Z depth"))
        self.pdepth_label.setToolTip(
            _("The maximum depth that the probe is allowed\n"
              "to probe. Negative value, in current units.")
        )
        self.pdepth_entry = FCDoubleSpinner()
        self.pdepth_entry.set_precision(self.decimals)
        self.pdepth_entry.set_range(-910000.0000, 0.0000)

        param_grid.addWidget(self.pdepth_label, 4, 0)
        param_grid.addWidget(self.pdepth_entry, 4, 1)

        # Probe feedrate
        self.feedrate_probe_label = FCLabel('%s:' % _("Probe Feedrate"))
        self.feedrate_probe_label.setToolTip(
           _("The feedrate used while the probe is probing.")
        )
        self.feedrate_probe_entry = FCDoubleSpinner()
        self.feedrate_probe_entry.set_precision(self.decimals)
        self.feedrate_probe_entry.set_range(0, 910000.0000)

        param_grid.addWidget(self.feedrate_probe_label, 6, 0)
        param_grid.addWidget(self.feedrate_probe_entry, 6, 1)

        separator_line = QtWidgets.QFrame()
        separator_line.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        separator_line.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        param_grid.addWidget(separator_line, 8, 0, 1, 2)

        # AUTOLEVELL MODE
        al_mode_lbl = FCLabel('%s' % _("Mode"), bold=True)
        al_mode_lbl.setToolTip(_("Choose a mode for height map generation.\n"
                                 "- Manual: will pick a selection of probe points by clicking on canvas\n"
                                 "- Grid: will automatically generate a grid of probe points"))

        self.al_mode_radio = RadioSet(
            [
                {'label': _('Manual'), 'value': 'manual'},
                {'label': _('Grid'), 'value': 'grid'}
            ])
        param_grid.addWidget(al_mode_lbl, 10, 0)
        param_grid.addWidget(self.al_mode_radio, 10, 1)

        # AUTOLEVELL METHOD
        self.al_method_lbl = FCLabel('%s:' % _("Method"))
        self.al_method_lbl.setToolTip(_("Choose a method for approximation of heights from autolevelling data.\n"
                                        "- Voronoi: will generate a Voronoi diagram\n"
                                        "- Bilinear: will use bilinear interpolation. Usable only for grid mode."))

        self.al_method_radio = RadioSet(
            [
                {'label': _('Voronoi'), 'value': 'v'},
                {'label': _('Bilinear'), 'value': 'b'}
            ])
        self.al_method_lbl.setDisabled(True)
        self.al_method_radio.setDisabled(True)
        self.al_method_radio.set_value('v')

        param_grid.addWidget(self.al_method_lbl, 12, 0)
        param_grid.addWidget(self.al_method_radio, 12, 1)

        # ## Columns
        self.al_columns_entry = FCSpinner()
        self.al_columns_entry.setMinimum(2)

        self.al_columns_label = FCLabel('%s:' % _("Columns"))
        self.al_columns_label.setToolTip(
            _("The number of grid columns.")
        )
        param_grid.addWidget(self.al_columns_label, 14, 0)
        param_grid.addWidget(self.al_columns_entry, 14, 1)

        # ## Rows
        self.al_rows_entry = FCSpinner()
        self.al_rows_entry.setMinimum(2)

        self.al_rows_label = FCLabel('%s:' % _("Rows"))
        self.al_rows_label.setToolTip(
            _("The number of grid rows.")
        )
        param_grid.addWidget(self.al_rows_label, 16, 0)
        param_grid.addWidget(self.al_rows_entry, 16, 1)

        self.al_add_button = FCButton(_("Add Probe Points"))
        self.al_box.addWidget(self.al_add_button)

        # #############################################################################################################
        # Controller Frame
        # #############################################################################################################
        self.al_controller_label = FCLabel('%s' % _("Controller"), color='red', bold=True)
        self.al_controller_label.setToolTip(
            _("The kind of controller for which to generate\n"
              "height map gcode.")
        )
        # the controller section is not part of self.al_frame: connecting to the machine, jogging it and sending
        # a job work for any kind of CNC job, not only for one that can be levelled
        self.tools_box.addWidget(self.al_controller_label)

        self.c_frame = FCFrame()
        self.tools_box.addWidget(self.c_frame)

        ctrl_grid = GLay(v_spacing=5, h_spacing=3)
        self.c_frame.setLayout(ctrl_grid)

        self.al_controller_combo = FCComboBox()
        self.al_controller_combo.addItems(["MACH3", "MACH4", "LinuxCNC", "GRBL"])
        ctrl_grid.addWidget(self.al_controller_combo, 0, 0, 1, 2)

        # #############################################################################################################
        # ########################## GRBL frame #######################################################################
        # #############################################################################################################
        self.grbl_frame = QtWidgets.QFrame()
        self.grbl_frame.setContentsMargins(0, 0, 0, 0)
        ctrl_grid.addWidget(self.grbl_frame, 2, 0, 1, 2)

        self.grbl_box = QtWidgets.QVBoxLayout()
        self.grbl_box.setContentsMargins(0, 0, 0, 0)
        self.grbl_frame.setLayout(self.grbl_box)

        # #############################################################################################################
        # ########################## GRBL TOOLBAR #####################################################################
        # #############################################################################################################
        self.al_toolbar = FCDetachableTab(protect=True)
        self.al_toolbar.setTabsClosable(False)
        self.al_toolbar.useOldIndex(True)
        self.al_toolbar.set_detachable(val=False)
        self.grbl_box.addWidget(self.al_toolbar)

        # GRBL Connect TAB
        self.gr_conn_tab = QtWidgets.QWidget()
        self.gr_conn_tab.setObjectName("connect_tab")
        self.gr_conn_tab_layout = QtWidgets.QVBoxLayout(self.gr_conn_tab)
        self.gr_conn_tab_layout.setContentsMargins(2, 2, 2, 2)
        # self.gr_conn_scroll_area = VerticalScrollArea()
        # self.gr_conn_tab_layout.addWidget(self.gr_conn_scroll_area)
        self.al_toolbar.addTab(self.gr_conn_tab, _("Connect"))

        # GRBL Control TAB
        self.gr_ctrl_tab = QtWidgets.QWidget()
        self.gr_ctrl_tab.setObjectName("connect_tab")
        self.gr_ctrl_tab_layout = QtWidgets.QVBoxLayout(self.gr_ctrl_tab)
        self.gr_ctrl_tab_layout.setContentsMargins(2, 2, 2, 2)

        # self.gr_ctrl_scroll_area = VerticalScrollArea()
        # self.gr_ctrl_tab_layout.addWidget(self.gr_ctrl_scroll_area)
        self.al_toolbar.addTab(self.gr_ctrl_tab, _("Control"))

        # GRBL Sender TAB
        self.gr_send_tab = QtWidgets.QWidget()
        self.gr_send_tab.setObjectName("connect_tab")
        self.gr_send_tab_layout = QtWidgets.QVBoxLayout(self.gr_send_tab)
        self.gr_send_tab_layout.setContentsMargins(2, 2, 2, 2)

        # self.gr_send_scroll_area = VerticalScrollArea()
        # self.gr_send_tab_layout.addWidget(self.gr_send_scroll_area)
        self.al_toolbar.addTab(self.gr_send_tab, _("Sender"))

        for idx in range(self.al_toolbar.count()):
            if self.al_toolbar.tabText(idx) == _("Connect"):
                self.al_toolbar.tabBar.setTabTextColor(idx, QtGui.QColor('red'))
            if self.al_toolbar.tabText(idx) == _("Control"):
                self.al_toolbar.tabBar.setTabEnabled(idx, False)
            if self.al_toolbar.tabText(idx) == _("Sender"):
                self.al_toolbar.tabBar.setTabEnabled(idx, False)
        # #############################################################################################################

        # #############################################################################################################
        # GRBL CONNECT
        # #############################################################################################################
        self.connect_frame = FCFrame()
        self.gr_conn_tab_layout.addWidget(self.connect_frame)

        grbl_conn_grid = GLay(v_spacing=5, h_spacing=3, c_stretch=[0, 1, 0])
        self.connect_frame.setLayout(grbl_conn_grid)

        # COM list
        self.com_list_label = FCLabel('%s:' % _("COM list"))
        self.com_list_label.setToolTip(
            _("Lists the available serial ports.")
        )

        self.com_list_combo = FCComboBox()
        self.com_search_button = FCButton(_("Search"))
        self.com_search_button.setToolTip(
            _("Search for the available serial ports.")
        )
        grbl_conn_grid.addWidget(self.com_list_label, 2, 0)
        grbl_conn_grid.addWidget(self.com_list_combo, 2, 1)
        grbl_conn_grid.addWidget(self.com_search_button, 2, 2)

        # BAUDRATES list
        self.baudrates_list_label = FCLabel('%s:' % _("Baud rates"))
        self.baudrates_list_label.setToolTip(
            _("Lists the available serial ports.")
        )

        self.baudrates_list_combo = FCComboBox()
        cb_model = QtCore.QStringListModel()
        self.baudrates_list_combo.setModel(cb_model)
        self.baudrates_list_combo.addItems(
            ['9600', '19200', '38400', '57600', '115200', '230400', '460800', '500000', '576000', '921600', '1000000',
             '1152000', '1500000', '2000000'])
        self.baudrates_list_combo.setCurrentText('115200')

        grbl_conn_grid.addWidget(self.baudrates_list_label, 4, 0)
        grbl_conn_grid.addWidget(self.baudrates_list_combo, 4, 1)

        # New baudrate
        self.new_bd_label = FCLabel('%s:' % _("New"))
        self.new_bd_label.setToolTip(
            _("New, custom baudrate.")
        )

        self.new_baudrate_entry = FCSpinner()
        self.new_baudrate_entry.set_range(40, 9999999)

        self.add_bd_button = FCButton(_("Add"))
        self.add_bd_button.setToolTip(
            _("Add the specified custom baudrate to the list.")
        )
        grbl_conn_grid.addWidget(self.new_bd_label, 6, 0)
        grbl_conn_grid.addWidget(self.new_baudrate_entry, 6, 1)
        grbl_conn_grid.addWidget(self.add_bd_button, 6, 2)

        self.del_bd_button = FCButton(_("Delete selected baudrate"))
        grbl_conn_grid.addWidget(self.del_bd_button, 8, 0, 1, 3)

        ctrl_h_lay = QtWidgets.QHBoxLayout()
        self.controller_reset_button = FCButton(_("Reset"))
        self.controller_reset_button.setToolTip(
            _("Software reset of the controller.")
        )
        self.controller_reset_button.setDisabled(True)
        ctrl_h_lay.addWidget(self.controller_reset_button)

        self.com_connect_button = FCButton()
        self.com_connect_button.setText(_("Disconnected"))
        self.com_connect_button.setToolTip(
            _("Connect to the selected port with the selected baud rate.")
        )
        self.com_connect_button.setStyleSheet("QPushButton {background-color: red;}")
        ctrl_h_lay.addWidget(self.com_connect_button)

        grbl_conn_grid.addWidget(FCLabel(""), 9, 0, 1, 3)
        grbl_conn_grid.setRowStretch(9, 1)
        grbl_conn_grid.addLayout(ctrl_h_lay, 10, 0, 1, 3)

        # #############################################################################################################
        # GRBL CONTROL
        # #############################################################################################################
        self.pos_grbl_frame = FCFrame()
        self.gr_ctrl_tab_layout.addWidget(self.pos_grbl_frame)
        grbl_pos_grid = GLay(v_spacing=3, h_spacing=6, c_stretch=[0, 1, 1, 1])
        self.pos_grbl_frame.setLayout(grbl_pos_grid)

        state_title_label = FCLabel('%s:' % _("State"), bold=True)
        self.grbl_state_label = FCLabel('-')
        self.grbl_unlock_button = FCButton(_("Unlock"))
        self.grbl_unlock_button.setToolTip(
            _("Clear an alarm of the controller ($X).\n"
              "After a limit switch was hit the controller is reset first.\n"
              "Do Home also clears an alarm and makes the position known again.")
        )
        state_h_lay = QtWidgets.QHBoxLayout()
        state_h_lay.addWidget(self.grbl_state_label, stretch=1)
        state_h_lay.addWidget(self.grbl_unlock_button)
        grbl_pos_grid.addWidget(state_title_label, 0, 0)
        grbl_pos_grid.addLayout(state_h_lay, 0, 1, 1, 3)

        for col, axis in enumerate(('X', 'Y', 'Z'), start=1):
            grbl_pos_grid.addWidget(FCLabel(axis, bold=True), 1, col)

        position_rows = (
            (_("Work"), _("The position in work coordinates, relative to the work origin set with Zero Axes.")),
            (_("Machine"), _("The position in machine coordinates, relative to where the machine homes.")),
            (_("Origin"), _("Where the work origin is, in machine coordinates.")),
        )
        position_labels = []
        for row, (title, tip) in enumerate(position_rows, start=2):
            title_label = FCLabel('%s:' % title)
            title_label.setToolTip(tip)
            grbl_pos_grid.addWidget(title_label, row, 0)
            labels = []
            for col in range(1, 4):
                value_label = FCLabel('-')
                value_label.setToolTip(tip)
                grbl_pos_grid.addWidget(value_label, row, col)
                labels.append(value_label)
            position_labels.append(labels)
        self.grbl_wpos_labels, self.grbl_mpos_labels, self.grbl_wco_labels = position_labels

        self.grbl_origin_label = FCLabel('')
        self.grbl_origin_label.setWordWrap(True)
        self.grbl_origin_label.setToolTip(
            _("Where Go to origin moves: up to the travel height first, then over the work origin.")
        )
        grbl_pos_grid.addWidget(self.grbl_origin_label, 5, 0, 1, 4)

        self.ctrl_grbl_frame = FCFrame()
        self.gr_ctrl_tab_layout.addWidget(self.ctrl_grbl_frame)
        grbl_ctrl_grid = GLay(v_spacing=5, h_spacing=3, c_stretch=[0, 1, 0])
        self.ctrl_grbl_frame.setLayout(grbl_ctrl_grid)

        self.ctrl_grbl_frame2 = FCFrame()
        self.gr_ctrl_tab_layout.addWidget(self.ctrl_grbl_frame2)
        grbl_ctrl2_grid = GLay(v_spacing=5, h_spacing=3)
        self.ctrl_grbl_frame2.setLayout(grbl_ctrl2_grid)

        self.gr_ctrl_tab_layout.addStretch(1)

        jog_title_label = FCLabel(_("Jog"), bold=True)

        zero_title_label = FCLabel(_("Zero Axes"), bold=True)
        # zero_title_label.setStyleSheet("""
        #                         FCLabel
        #                         {
        #                             font-weight: bold;
        #                         }
        #                         """)

        grbl_ctrl_grid.addWidget(jog_title_label, 0, 0)
        grbl_ctrl_grid.addWidget(zero_title_label, 0, 2)

        self.jog_wdg = FCJog(self.app)
        self.jog_wdg.setStyleSheet("""
                            FCJog
                            {
                                border: 1px solid lightgray;
                                border-radius: 5px;
                            }
                            """)

        self.zero_axs_wdg = FCZeroAxes(self.app)
        self.zero_axs_wdg.setStyleSheet("""
                            FCZeroAxes
                            {
                                border: 1px solid lightgray;
                                border-radius: 5px
                            }
                            """)
        grbl_ctrl_grid.addWidget(self.jog_wdg, 2, 0)
        grbl_ctrl_grid.addWidget(self.zero_axs_wdg, 2, 2)

        self.pause_resume_button = RotatedToolButton()
        self.pause_resume_button.setSizePolicy(QtWidgets.QSizePolicy.Policy.Minimum,
                                               QtWidgets.QSizePolicy.Policy.Expanding)
        self.pause_resume_button.setText(_("Pause/Resume"))
        self.pause_resume_button.setCheckable(True)
        self.pause_resume_button.setStyleSheet("""
                            RotatedToolButton:checked
                            {
                                background-color: red;
                                color: white;
                                border: none;
                            }
                            """)

        pause_frame = QtWidgets.QFrame()
        pause_frame.setContentsMargins(0, 0, 0, 0)
        pause_frame.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Expanding)
        pause_h_lay = QtWidgets.QHBoxLayout()
        pause_h_lay.setContentsMargins(0, 0, 0, 0)

        pause_h_lay.addWidget(self.pause_resume_button)
        pause_frame.setLayout(pause_h_lay)
        grbl_ctrl_grid.addWidget(pause_frame, 2, 1)

        # JOG Step
        self.jog_step_label = FCLabel('%s:' % _("Step"))
        self.jog_step_label.setToolTip(
            _("Each jog action will move the axes with this value.")
        )

        self.jog_step_entry = FCSliderWithDoubleSpinner()
        self.jog_step_entry.set_precision(self.decimals)
        self.jog_step_entry.setSingleStep(0.1)
        self.jog_step_entry.set_range(0, 500)

        grbl_ctrl2_grid.addWidget(self.jog_step_label, 0, 0)
        grbl_ctrl2_grid.addWidget(self.jog_step_entry, 0, 1)

        # JOG Feedrate
        self.jog_fr_label = FCLabel('%s:' % _("Feedrate"))
        self.jog_fr_label.setToolTip(
            _("Feedrate when jogging.")
        )

        self.jog_fr_entry = FCSliderWithDoubleSpinner()
        self.jog_fr_entry.set_precision(self.decimals)
        self.jog_fr_entry.setSingleStep(10)
        self.jog_fr_entry.set_range(0, 10000)

        grbl_ctrl2_grid.addWidget(self.jog_fr_label, 1, 0)
        grbl_ctrl2_grid.addWidget(self.jog_fr_entry, 1, 1)

        # #############################################################################################################
        # Z TOUCH OFF
        # #############################################################################################################
        # PROBE TRAVEL
        self.probe_travel_label = FCLabel('%s:' % _("Probe travel"))
        self.probe_travel_label.setToolTip(
            _("How far down the probe is allowed to travel\n"
              "looking for the surface, starting from where\n"
              "the tool is now.")
        )

        self.probe_travel_entry = FCDoubleSpinner()
        self.probe_travel_entry.set_precision(self.decimals)
        self.probe_travel_entry.set_range(0.0000, 10000.0000)

        grbl_ctrl2_grid.addWidget(self.probe_travel_label, 2, 0)
        grbl_ctrl2_grid.addWidget(self.probe_travel_entry, 2, 1)

        # PROBE FEEDRATE
        self.probe_fr_label = FCLabel('%s:' % _("Probe feedrate"))
        self.probe_fr_label.setToolTip(
            _("Feedrate of the downward probing move.\n"
              "Slow is accurate.")
        )

        self.probe_fr_entry = FCDoubleSpinner()
        self.probe_fr_entry.set_precision(self.decimals)
        self.probe_fr_entry.set_range(0.0000, 10000.0000)

        grbl_ctrl2_grid.addWidget(self.probe_fr_label, 3, 0)
        grbl_ctrl2_grid.addWidget(self.probe_fr_entry, 3, 1)

        # PLATE THICKNESS
        self.plate_thickness_label = FCLabel('%s:' % _("Plate thickness"))
        self.plate_thickness_label.setToolTip(
            _("The Z that the touch point is given.\n"
              "It is the thickness of the touch plate, or zero\n"
              "when the probe clip is on the tool and the\n"
              "material itself is the contact.")
        )

        self.plate_thickness_entry = FCDoubleSpinner()
        self.plate_thickness_entry.set_precision(self.decimals)
        self.plate_thickness_entry.set_range(0.0000, 100.0000)

        grbl_ctrl2_grid.addWidget(self.plate_thickness_label, 4, 0)
        grbl_ctrl2_grid.addWidget(self.plate_thickness_entry, 4, 1)

        # PROBE Z
        self.probe_z_button = FCButton(_("Probe Z"))
        self.probe_z_button.setToolTip(
            _("Will probe down for the surface and make the\n"
              "touch point the Z origin of the work coordinate\n"
              "system, which sets the length of the tool.")
        )
        grbl_ctrl2_grid.addWidget(self.probe_z_button, 5, 0, 1, 2)

        # #############################################################################################################
        # GRBL SENDER
        # #############################################################################################################
        self.sender_frame = FCFrame()
        self.gr_send_tab_layout.addWidget(self.sender_frame)

        grbl_send_grid = GLay(v_spacing=5, h_spacing=3, c_stretch=[1, 0])
        self.sender_frame.setLayout(grbl_send_grid)

        # Send CUSTOM COMMAND
        self.grbl_command_label = FCLabel('%s:' % _("Send Command"))
        self.grbl_command_label.setToolTip(
            _("Send a custom command to GRBL.")
        )
        grbl_send_grid.addWidget(self.grbl_command_label, 2, 0, 1, 2)

        self.grbl_command_entry = FCEntry()
        self.grbl_command_entry.setPlaceholderText(_("Type GRBL command ..."))

        self.grbl_send_button = QtWidgets.QToolButton()
        self.grbl_send_button.setText(_("Send"))
        self.grbl_send_button.setToolTip(
            _("Send a custom command to GRBL.")
        )
        grbl_send_grid.addWidget(self.grbl_command_entry, 4, 0)
        grbl_send_grid.addWidget(self.grbl_send_button, 4, 1)

        # Get Parameter
        self.grbl_get_param_label = FCLabel('%s:' % _("Get Config parameter"))
        self.grbl_get_param_label.setToolTip(
            _("A GRBL configuration parameter.")
        )
        grbl_send_grid.addWidget(self.grbl_get_param_label, 6, 0, 1, 2)

        self.grbl_parameter_entry = FCEntry()
        self.grbl_parameter_entry.setPlaceholderText(_("Type GRBL parameter ..."))

        self.grbl_get_param_button = QtWidgets.QToolButton()
        self.grbl_get_param_button.setText(_("Get"))
        self.grbl_get_param_button.setToolTip(
            _("Get the value of a specified GRBL parameter.")
        )
        grbl_send_grid.addWidget(self.grbl_parameter_entry, 8, 0)
        grbl_send_grid.addWidget(self.grbl_get_param_button, 8, 1)

        grbl_send_grid.setRowStretch(9, 1)

        # GET Report
        self.grbl_report_button = FCButton(_("Get Report"))
        self.grbl_report_button.setToolTip(
            _("Print in shell the GRBL report.")
        )
        grbl_send_grid.addWidget(self.grbl_report_button, 10, 0, 1, 2)

        # SEND THE JOB
        self.grbl_send_job_button = FCButton(_("Send Job"))
        self.grbl_send_job_button.setToolTip(
            _("Will send the GCode of the selected CNCJob object\n"
              "to the GRBL controller. A tool change stops the job\n"
              "until it is continued.")
        )
        grbl_send_grid.addWidget(self.grbl_send_job_button, 11, 0)

        self.grbl_stop_job_button = QtWidgets.QToolButton()
        self.grbl_stop_job_button.setText(_("Stop"))
        self.grbl_stop_job_button.setToolTip(
            _("Will hold the machine and stop sending the job.")
        )
        self.grbl_stop_job_button.setDisabled(True)
        grbl_send_grid.addWidget(self.grbl_stop_job_button, 11, 1)

        # CONTINUE A PAUSED JOB
        self.grbl_continue_button = FCButton(_("Continue"))
        self.grbl_continue_button.setToolTip(
            _("Will let a job that stopped at a program pause continue.\n"
              "A tool change stops the job until the tool was changed\n"
              "and its length was set.")
        )
        self.grbl_continue_button.setDisabled(True)
        grbl_send_grid.addWidget(self.grbl_continue_button, 12, 0, 1, 2)

        # RESUME A STOPPED JOB
        self.grbl_resume_job_button = FCButton(_("Resume from line"))
        self.grbl_resume_job_button.setToolTip(
            _("Will send the job again, starting with the line on the right.\n"
              "The units, the distance mode, the feedrate and the spindle of\n"
              "the skipped lines are restored first and the tool is lifted to\n"
              "a safe height before the first move.")
        )
        self.grbl_resume_job_button.setDisabled(True)
        grbl_send_grid.addWidget(self.grbl_resume_job_button, 13, 0)

        self.grbl_resume_line_entry = FCSpinner()
        self.grbl_resume_line_entry.set_range(1, 9999999)
        self.grbl_resume_line_entry.setToolTip(
            _("The number of the line to resume the job from.\n"
              "Only the lines that are sent to the controller are counted,\n"
              "the comments and the empty lines are not.")
        )
        grbl_send_grid.addWidget(self.grbl_resume_line_entry, 13, 1)

        self.grbl_job_progress_label = FCLabel('')
        grbl_send_grid.addWidget(self.grbl_job_progress_label, 14, 0, 1, 2)

        hm_lay = QtWidgets.QHBoxLayout()
        # GET HEIGHT MAP
        self.grbl_get_heightmap_button = FCButton(_("Apply AutoLevelling"))
        self.grbl_get_heightmap_button.setToolTip(
            _("Will send the probing GCode to the GRBL controller,\n"
              "wait for the Z probing data and then apply this data\n"
              "over the original GCode therefore doing autolevelling.")
        )
        hm_lay.addWidget(self.grbl_get_heightmap_button, stretch=1)

        self.grbl_save_height_map_button = QtWidgets.QToolButton()
        self.grbl_save_height_map_button.setIcon(QtGui.QIcon(self.app.resource_location + '/save_as.png'))
        self.grbl_save_height_map_button.setToolTip(
            _("Will save the GRBL height map.")
        )
        hm_lay.addWidget(self.grbl_save_height_map_button, stretch=0, alignment=Qt.AlignmentFlag.AlignRight)

        grbl_send_grid.addLayout(hm_lay, 15, 0, 1, 2)

        self.grbl_frame.hide()
        # #############################################################################################################

        # the levelling steps that follow the controller section; enabled together with self.al_frame
        self.al_frame2 = QtWidgets.QFrame()
        self.al_frame2.setContentsMargins(0, 0, 0, 0)
        self.tools_box.addWidget(self.al_frame2)

        self.al_box2 = QtWidgets.QVBoxLayout()
        self.al_box2.setContentsMargins(0, 0, 0, 0)
        self.al_frame2.setLayout(self.al_box2)
        self.al_frame2.setDisabled(True)

        height_lay = QtWidgets.QHBoxLayout()
        self.h_gcode_button = FCButton(_("Save Probing GCode"))
        self.h_gcode_button.setToolTip(
            _("Will save the probing GCode.")
        )
        self.h_gcode_button.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,
                                          QtWidgets.QSizePolicy.Policy.MinimumExpanding)

        height_lay.addWidget(self.h_gcode_button)
        self.view_h_gcode_button = QtWidgets.QToolButton()
        self.view_h_gcode_button.setIcon(QtGui.QIcon(self.app.resource_location + '/edit_file32.png'))
        # self.view_h_gcode_button.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
        #                                        QtWidgets.QSizePolicy.Policy.Ignored)
        self.view_h_gcode_button.setToolTip(
            _("View/Edit the probing GCode.")
        )
        # height_lay.addStretch()
        height_lay.addWidget(self.view_h_gcode_button)

        self.al_box2.addLayout(height_lay)

        self.import_heights_button = FCButton(_("Import Height Map"))
        self.import_heights_button.setToolTip(
            _("Import the file that has the Z heights\n"
              "obtained through probing and then apply this data\n"
              "over the original GCode therefore\n"
              "doing autolevelling.")
        )
        self.al_box2.addWidget(self.import_heights_button)

        # self.h_gcode_button.hide()
        # self.import_heights_button.hide()

        # separator_line = QtWidgets.QFrame()
        # separator_line.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        # separator_line.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        # grid0.addWidget(separator_line, 35, 0, 1, 2)

        self.tools_box.addStretch(1)

        # ## Reset Tool
        self.reset_button = FCButton(_("Reset Tool"), bold=True)
        self.reset_button.setIcon(QtGui.QIcon(self.app.resource_location + '/reset32.png'))
        self.reset_button.setToolTip(
            _("Will reset the tool parameters.")
        )
        self.tools_box.addWidget(self.reset_button)
        # ############################ FINISHED GUI ###################################
        # #############################################################################

        GLay.set_common_column_size([tool_grid, param_grid], 0)

        self.plot_probing_pts_cb.stateChanged.connect(self.on_plot_points_changed)
        self.avoid_exc_holes_cb.stateChanged.connect(self.on_avoid_exc_holes_changed)

        # the touch off values are not stored on the object, therefore they are kept in the app options
        self.probe_travel_entry.valueChanged.connect(
            lambda val: self.app.options.__setitem__("tools_al_grbl_probe_travel", val))
        self.probe_fr_entry.valueChanged.connect(
            lambda val: self.app.options.__setitem__("tools_al_grbl_probe_fr", val))
        self.plate_thickness_entry.valueChanged.connect(
            lambda val: self.app.options.__setitem__("tools_al_grbl_plate_thickness", val))

    def confirmation_message(self, accepted, minval, maxval):
        if accepted is False:
            self.app.inform[str, bool].emit('[WARNING_NOTCL] %s: [%.*f, %.*f]' % (_("Edited value is out of range"),
                                                                                  self.decimals,
                                                                                  minval,
                                                                                  self.decimals,
                                                                                  maxval), False)
        else:
            self.app.inform[str, bool].emit('[success] %s' % _("Edited value is within limits."), False)

    def confirmation_message_int(self, accepted, minval, maxval):
        if accepted is False:
            self.app.inform[str, bool].emit('[WARNING_NOTCL] %s: [%d, %d]' %
                                            (_("Edited value is out of range"), minval, maxval), False)
        else:
            self.app.inform[str, bool].emit('[success] %s' % _("Edited value is within limits."), False)

    def on_plot_points_changed(self, state):
        self.app.options["tools_al_plot_points"] = False if not state else True

    def on_avoid_exc_holes_changed(self, state):
        self.app.options["tools_al_avoid_exc_holes"] = False if not state else True
