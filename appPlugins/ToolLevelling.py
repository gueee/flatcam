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
        self.build_al_table_sig.connect(self.build_al_table)
        self.grbl_job_progress_sig.connect(self.on_grbl_job_progress)
        self.grbl_job_finished_sig.connect(self.on_grbl_job_finished)
        self.grbl_job_paused_sig.connect(self.on_grbl_job_paused)
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

        # Sender
        self.ui.grbl_report_button.clicked.connect(lambda: self.send_grbl_command(command='?'))
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
            x = value['point'].x
            y = value['point'].y
            xy_coords = self.app.dec_format(x, dec=self.app.decimals), self.app.dec_format(y, dec=self.app.decimals)
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

    def on_grbl_connect(self):
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
                    self.ui.com_connect_button.setStyleSheet("QPushButton {background-color: seagreen;}")
                    self.ui.com_connect_button.setText(_("Connected"))
                    self.ui.controller_reset_button.setDisabled(False)

                    for idx in range(self.ui.al_toolbar.count()):
                        if self.ui.al_toolbar.tabText(idx) == _("Connect"):
                            self.ui.al_toolbar.tabBar.setTabTextColor(idx, QtGui.QColor('seagreen'))
                        if self.ui.al_toolbar.tabText(idx) == _("Control"):
                            self.ui.al_toolbar.tabBar.setTabEnabled(idx, True)
                        if self.ui.al_toolbar.tabText(idx) == _("Sender"):
                            self.ui.al_toolbar.tabBar.setTabEnabled(idx, True)

                    self.app.inform.emit("%s: %s" % (_("Port connected"), port_name))
                    return

            self.grbl_ser_port.close()
            self.app.inform.emit("[ERROR_NOTCL] %s: %s" % (_("Could not connect to GRBL on port"), port_name))

        except serial.SerialException:
            self.grbl_ser_port = serial.Serial()
            self.grbl_ser_port.port = port_name
            self.grbl_ser_port.close()
            self.ui.com_connect_button.setStyleSheet("QPushButton {background-color: red;}")
            self.ui.com_connect_button.setText(_("Disconnected"))
            self.ui.controller_reset_button.setDisabled(True)

            for idx in range(self.ui.al_toolbar.count()):
                if self.ui.al_toolbar.tabText(idx) == _("Connect"):
                    self.ui.al_toolbar.tabBar.setTabTextColor(idx, QtGui.QColor('red'))
                if self.ui.al_toolbar.tabText(idx) == _("Control"):
                    self.ui.al_toolbar.tabBar.setTabEnabled(idx, False)
                if self.ui.al_toolbar.tabText(idx) == _("Sender"):
                    self.ui.al_toolbar.tabBar.setTabEnabled(idx, False)
            self.app.inform.emit("%s: %s" % (_("Port is connected. Disconnecting"), port_name))
        except Exception:
            self.app.inform.emit("[ERROR_NOTCL] %s: %s" % (_("Could not connect to port"), port_name))

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

    def grbl_read_reply(self, timeout=30.0):
        """
        Read the answer of the GRBL controller until it acknowledges the command with 'ok' or reports a problem.
        The serial port has a short read timeout therefore reading is repeated until the deadline.

        :param timeout: for how long to wait for an acknowledgement, in seconds
        :type timeout:  float
        :return:        the answer lines of the controller and whether it acknowledged the command
        :rtype:         tuple
        """
        deadline = time.monotonic() + timeout
        answer = []

        while time.monotonic() < deadline:
            new_lines = self.grbl_decode(self.grbl_ser_port.readlines())
            answer += [line for line in new_lines if line != '']

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

        # show the Shell Dock
        self.app.ui.shell_dock.show()

        def worker_task():
            with self.app.proc_container.new('%s...' % _("Sending")):
                self.send_grbl_command(command=cmd)

        self.app.worker_task.emit({'fcn': worker_task, 'params': []})

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
        answer, acknowledged = self.grbl_read_reply(timeout=timeout)

        if not answer:
            self.app.inform_shell[str, bool].emit('\t\t\t: No answer\n', False)

        if echo:
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

    def on_grbl_get_parameter(self, param):
        if '$' in param:
            param = param.replace('$', '')

        snd = '$$\n'
        self.grbl_ser_port.write(snd.encode('utf-8'))
        answer, _ack = self.grbl_read_reply()
        for decoded_line in answer:
            par = '$%s' % str(param)
            if par in decoded_line:
                result = float(decoded_line.rpartition('=')[2])
                self.app.shell_message("GRBL Parameter: %s = %s" % (str(param), str(result)), show=True)
                return result

    def on_grbl_jog(self, direction=None):
        if direction is None:
            return
        cmd = ''

        step = self.ui.jog_step_entry.get_value(),
        feedrate = self.ui.jog_fr_entry.get_value()
        travelz = float(self.app.options["tools_al_grbl_travelz"])

        if direction == 'xplus':
            cmd = "$J=G91 %s X%s F%s" % ({'IN': 'G20', 'MM': 'G21'}[self.units], str(step), str(feedrate))
        if direction == 'xminus':
            cmd = "$J=G91 %s X-%s F%s" % ({'IN': 'G20', 'MM': 'G21'}[self.units], str(step), str(feedrate))
        if direction == 'yplus':
            cmd = "$J=G91 %s Y%s F%s" % ({'IN': 'G20', 'MM': 'G21'}[self.units], str(step), str(feedrate))
        if direction == 'yminus':
            cmd = "$J=G91 %s Y-%s F%s" % ({'IN': 'G20', 'MM': 'G21'}[self.units], str(step), str(feedrate))

        if direction == 'zplus':
            cmd = "$J=G91 %s Z%s F%s" % ({'IN': 'G20', 'MM': 'G21'}[self.units], str(step), str(feedrate))
        if direction == 'zminus':
            cmd = "$J=G91 %s Z-%s F%s" % ({'IN': 'G20', 'MM': 'G21'}[self.units], str(step), str(feedrate))

        if direction == 'origin':
            cmd = "$J=G90 %s Z%s F%s" % ({'IN': 'G20', 'MM': 'G21'}[self.units], str(travelz), str(feedrate))
            self.send_grbl_command(command=cmd, echo=False)
            cmd = "$J=G90 %s X0.0 Y0.0 F%s" % ({'IN': 'G20', 'MM': 'G21'}[self.units], str(feedrate))
            self.send_grbl_command(command=cmd, echo=False)
            return

        self.send_grbl_command(command=cmd, echo=False)

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
        current_mode = self.on_grbl_get_parameter('10')
        if current_mode is None:
            return

        cmd = '$10=0'
        self.send_grbl_command(command=cmd, echo=False)

        self.send_grbl_command(command=self.grbl_zero_command(axis), echo=False)

        # restore previous mode
        cmd = '$10=%d' % int(current_mode)
        self.send_grbl_command(command=cmd, echo=False)

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
        if self.grbl_ser_port is None or not self.grbl_ser_port.is_open:
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller is not connected."))
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

        def probe_task():
            self.app.inform.emit('%s' % _("Probing for the surface."))

            for command in commands:
                output = self.send_grbl_command(command=command, timeout=120.0)
                if not output:
                    self.app.inform.emit('[ERROR_NOTCL] %s: %s' % (_("The GRBL controller rejected the line"),
                                                                   command))
                    return

                if command.startswith('G91') and self.parse_probe_answer(output) is None:
                    # the probe did not touch anything; the controller is in an alarm state and the distance
                    # mode is still relative, therefore nothing more is sent
                    self.app.inform.emit(
                        '[ERROR_NOTCL] %s' % _("The probe did not touch the surface. Unlock the controller "
                                               "with $X and set the distance mode back with G90.")
                    )
                    return

            self.app.inform.emit('[success] %s' % _("The tool length was set."))

        # a thread of its own and not a worker task: the worker that streams a job holds one of the few threads
        # of the pool, and a probe that is queued there would only run after the job was continued
        threading.Thread(target=probe_task, daemon=True).start()

    def on_grbl_homing(self):
        cmd = '$H'
        self.app.inform.emit("%s" % _("GRBL is doing a home cycle."))
        self.on_grbl_wake()
        self.send_grbl_command(command=cmd)

    def on_grbl_reset(self):
        cmd = '\x18'
        self.app.inform.emit("%s" % _("GRBL software reset was sent."))
        self.on_grbl_wake()
        self.send_grbl_command(command=cmd)

    def on_grbl_pause_resume(self, checked):
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

        def worker_task():
            with self.app.proc_container.new('%s...' % _("Sending")):
                self.grbl_probe_result = ''
                pr_travelz = str(self.ui.ptravelz_entry.get_value())
                probe_fr = str(self.ui.feedrate_probe_entry.get_value())
                pr_depth = str(self.ui.pdepth_entry.get_value())

                cmd = {'IN': 'G20', 'MM': 'G21'}[self.units]
                self.send_grbl_command(command=cmd)
                cmd = 'G90'
                self.send_grbl_command(command=cmd)

                probed_points = []
                for x, y in self.probe_points():
                    cmd = 'G0 Z%s' % pr_travelz
                    self.send_grbl_command(command=cmd)
                    cmd = 'G0 X%s Y%s' % (str(x), str(y))
                    self.send_grbl_command(command=cmd)
                    cmd = 'G38.2 Z%s F%s' % (pr_depth, probe_fr)
                    output = self.send_grbl_command(command=cmd, timeout=120.0)

                    self.grbl_probe_result += output + '\n'

                    height = self.parse_probe_answer(output)
                    if height is None:
                        self.app.inform.emit(
                            '[ERROR_NOTCL] %s: %s, %s' % (_("Probing failed in point"), str(x), str(y))
                        )
                        return
                    probed_points.append((x, y, height))

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

        if self.grbl_ser_port is None or not self.grbl_ser_port.is_open:
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("The GRBL controller is not connected."))
            return

        # show the Shell Dock
        self.app.ui.shell_dock.show()

        self.grbl_job_stop = False
        self.grbl_job_waiting = False
        self.ui.grbl_send_job_button.setDisabled(True)
        self.ui.grbl_stop_job_button.setDisabled(False)
        self.ui.grbl_continue_button.setDisabled(True)
        self.ui.grbl_resume_job_button.setDisabled(True)
        self.on_grbl_manual_control(False)

        def worker_task():
            with self.app.proc_container.new('%s...' % _("Sending")):
                self.grbl_stream_gcode(job_gcode, start_line=start_line)

        self.app.worker_task.emit({'fcn': worker_task, 'params': []})

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
        self.ui.grbl_continue_button.setDisabled(False)
        self.ui.grbl_job_progress_label.set_value(message)
        self.app.inform.emit('[WARNING_NOTCL] %s' % message)

        # the machine stands still and the streaming thread is not reading the port, therefore the tool can be
        # changed, the machine jogged and the new tool touched off
        self.on_grbl_manual_control(True)

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
        self.ui.grbl_continue_button.setDisabled(True)
        self.on_grbl_manual_control(False)
        self.grbl_job_waiting = False
        self.app.inform.emit('%s' % _("The job continues."))

    def on_grbl_job_finished(self):
        self.ui.grbl_send_job_button.setDisabled(False)
        self.ui.grbl_stop_job_button.setDisabled(True)
        self.ui.grbl_continue_button.setDisabled(True)

        # the machine is free again, therefore it can be moved by hand and the levelling of the selected object
        # is available once more
        self.on_grbl_manual_control(True)
        self.on_levelling_available(self.can_be_levelled(
            self.app.collection.get_by_name(self.ui.object_combo.get_value())))

        # a job that did not run to its end can be resumed from the line that follows the last one the
        # controller acknowledged
        resume_from = self.grbl_job_last_ack + 1
        can_resume = 1 < resume_from <= len(self.grbl_job_lines)
        self.ui.grbl_resume_job_button.setDisabled(not can_resume)
        if can_resume:
            self.ui.grbl_resume_line_entry.set_value(resume_from)

    def on_grbl_stop_job(self):
        """
        Hold the machine and stop sending the job.
        """
        self.grbl_job_stop = True
        self.grbl_job_waiting = False
        try:
            # feed hold; the job is not resumed, the remaining lines are not sent. It is a realtime command that
            # the controller does not acknowledge and reading the port here would take the answers that the
            # thread which streams the job waits for.
            self.grbl_ser_port.write('!'.encode('utf-8'))
        except Exception as e:
            self.app.log.error("ToolLevelling.on_grbl_stop_job() --> %s" % str(e))
        self.app.inform.emit('[WARNING_NOTCL] %s' % _("Sending the job was stopped."))

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
                pending.extend(self.grbl_decode(raw))

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

            # the controller has something to say that is not about the line that was sent
            self.app.inform_shell.emit('\t\t\t: %s' % response.upper())

    def grbl_read_status(self, timeout=2.0):
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

        # a status report is asked for; it is a realtime command that is not acknowledged with 'ok'
        self.grbl_ser_port.write('?'.encode('utf-8'))

        while time.monotonic() < deadline:
            time.sleep(0.1)

            for response in self.grbl_decode(self.grbl_ser_port.readlines()):
                stripped = response.strip()
                if stripped.startswith('<'):
                    return stripped[1:].split('|')[0].strip().lower()

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
