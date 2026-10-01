# ##########################################################
# FlatCAM: 2D Post-processing for Manufacturing            #
# File Modified by: Marius Adrian Stanciu (c)              #
# Date: 3/10/2019                                          #
# MIT Licence                                              #
# ##########################################################

from PyQt6 import QtWidgets, QtCore, QtGui  # noqa

from appPlugins.ToolNCC.NccUI import NccUI
from appTool import AppTool
from appGUI.GUIElements import (
    VerticalScrollArea,
    FCComboBox,
    FCCheckBox,
    RadioSet,
    FCDoubleSpinner,
    FCInputDialogSpinnerButton,
)

import logging
from copy import deepcopy
import numpy as np
import simplejson as json
import sys

from shapely import (
    Polygon,
    MultiPolygon,
)
from shapely.ops import unary_union, nearest_points

import gettext
import appTranslation as fcTranslate
import builtins

from appPlugins.ToolNCC.NccGen import NccGen
from appParsers.ParseGerber import Gerber
from camlib import grace, flatten_shapely_geometry
from matplotlib.backend_bases import KeyEvent as mpl_key_event

fcTranslate.apply_language('strings')
if '_' not in builtins.__dict__:
    _ = gettext.gettext

log = logging.getLogger('base')


class ToolNcc(Gerber, AppTool):

    optimal_found_sig = QtCore.pyqtSignal(float)

    def __init__(self, app):
        self.app = app
        self.decimals = self.app.decimals

        AppTool.__init__(self, app)
        Gerber.__init__(self, steps_per_circle=self.app.options["gerber_circle_steps"], app=app)

        # #############################################################################
        # ######################### Tool GUI ##########################################
        # #############################################################################
        self.ui = NccUI(layout=self.layout, app=self.app)
        self.pluginName = self.ui.pluginName

        self.init_context_menu()

        # #############################################################################
        # ########################## VARIABLES ########################################
        # #############################################################################
        self.units = ''
        self.ncc_tools = {}
        self.tooluid = 0

        # store here the default data for Geometry Data
        self.default_data = {}

        self.grid_status_memory = None

        self.obj_name = ""
        self.ncc_obj = None

        self.areas_to_clear_list: list[Polygon] = []

        self.bound_obj_name = ""
        self.bound_obj = None

        self.has_offset = None
        self.o_name = None
        self.overlap = None
        self.connect = None
        self.contour = None
        self.rest = None

        # store here the tool diameter that is guaranteed to isolate the object
        self.safe_tooldia = None

        self.ncc_dia_list = []
        self.iso_dia_list = []

        self.first_click = False
        self.cursor_pos = None
        self.mouse_is_dragging = False

        # store here the points for the "Polygon" area selection shape
        self.points = []
        # set this as True when in middle of drawing a "Polygon" area selection shape
        # it is made False by first click to signify that the shape is complete
        self.poly_drawn = False

        self.mm = None
        self.mr = None
        self.kp = None

        # disconnect flags
        self.area_sel_disconnect_flag = False

        # store here solid_geometry when there are tool with isolation job
        self.solid_geometry = []

        self.select_method = None
        self.tool_type_item_options = []

        self.circle_steps = int(self.app.options["gerber_circle_steps"])

        self.tooldia = None

        self.form_fields = {
            "tools_ncc_operation":      self.ui.op_radio,
            "tools_ncc_overlap":        self.ui.overlap_entry,
            "tools_ncc_margin":         self.ui.margin_entry,
            "tools_ncc_method":         self.ui.method_combo,
            "tools_ncc_connect":        self.ui.connect_cb,
            "tools_ncc_contour":        self.ui.contour_cb,
            "tools_ncc_offset_choice":  self.ui.offset_choice_cb,
            "tools_ncc_offset_value":   self.ui.offset_entry,
            "tools_ncc_milling_type":   self.ui.milling_type_radio,
            "tools_ncc_check_valid":    self.ui.valid_cb
        }

        self.name2option = {
            "n_operation":      "tools_ncc_operation",
            "n_overlap":        "tools_ncc_overlap",
            "n_margin":         "tools_ncc_margin",
            "n_method":         "tools_ncc_method",
            "n_connect":        "tools_ncc_connect",
            "n_contour":        "tools_ncc_contour",
            "n_offset":         "tools_ncc_offset_choice",
            "n_offset_value":   "tools_ncc_offset_value",
            "n_milling_type":   "tools_ncc_milling_type",
            "n_check":          "tools_ncc_check_valid",
        }

        self.old_tool_dia = None

        self.gen: NccGen | None = None

    def install(self, icon=None, separator=None, **kwargs):
        AppTool.install(self, icon, separator, shortcut='Alt+N', **kwargs)

    def run(self, toggle=True):
        self.app.defaults.report_usage("ToolNonCopperClear()")

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

        # reset those objects on a new run
        self.ncc_obj = None
        self.bound_obj = None
        self.obj_name = ''
        self.bound_obj_name = ''

        self.build_ui()

        # all the tools are selected by default
        # self.ui.tools_table.selectColumn(0)
        self.ui.tools_table.selectAll()

        self.app.ui.notebook.setTabText(2, _("NCC"))

    def clear_context_menu(self):
        self.ui.tools_table.removeContextMenu()

    def init_context_menu(self):

        # #############################################################################
        # ###################### Setup CONTEXT MENU ###################################
        # #############################################################################
        self.ui.tools_table.setupContextMenu()
        self.ui.tools_table.addContextMenu(
            _("Add"), self.on_tool_add_by_key, icon=QtGui.QIcon(self.app.resource_location + "/plus16.png")
        )
        self.ui.tools_table.addContextMenu(
            _("Add from DB"), self.on_tool_add_by_key, icon=QtGui.QIcon(self.app.resource_location + "/plus16.png")
        )
        self.ui.tools_table.addContextMenu(
            _("Delete"), lambda:
            self.on_tool_delete(rows_to_delete=None, all_tools=None),
            icon=QtGui.QIcon(self.app.resource_location + "/delete32.png")
        )

    def connect_signals(self):
        # #############################################################################
        # ############################ SIGNALS ########################################
        # #############################################################################
        try:
            self.ui.level.toggled.disconnect(self.on_level_changed)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.level.toggled.connect(self.on_level_changed)

        try:
            self.ui.find_optimal_button.clicked.disconnect(self.on_find_optimal_tooldia)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.find_optimal_button.clicked.connect(self.on_find_optimal_tooldia)

        # Custom Signal
        try:
            self.optimal_found_sig.disconnect(lambda val: self.ui.new_tooldia_entry.set_value(float(val)))
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.optimal_found_sig.connect(lambda val: self.ui.new_tooldia_entry.set_value(float(val)))

        try:
            self.ui.deltool_btn.clicked.disconnect(self.on_tool_delete)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.deltool_btn.clicked.connect(self.on_tool_delete)

        try:
            self.ui.generate_button.clicked.disconnect(self.gen.on_ncc_click)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.generate_button.clicked.connect(self.gen.on_ncc_click)

        try:
            self.ui.op_radio.activated_custom.disconnect(self.on_operation_change)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.op_radio.activated_custom.connect(self.on_operation_change)

        try:
            self.ui.reference_type_combo.currentIndexChanged.disconnect(self.on_reference_combo_changed)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.reference_type_combo.currentIndexChanged.connect(self.on_reference_combo_changed)

        try:
            self.ui.select_method_combo.currentIndexChanged.disconnect(self.ui.on_selection)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.select_method_combo.currentIndexChanged.connect(self.ui.on_selection)

        try:
            self.ui.rest_cb.stateChanged.disconnect(self.ui.on_rest_machining_check)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.rest_cb.stateChanged.connect(self.ui.on_rest_machining_check)

        try:
            self.ui.order_combo.currentIndexChanged.disconnect(self.on_order_changed)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.order_combo.currentIndexChanged.connect(self.on_order_changed)

        try:
            self.ui.type_obj_radio.activated_custom.disconnect(self.on_type_obj_index_changed)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.type_obj_radio.activated_custom.connect(self.on_type_obj_index_changed)

        try:
            self.ui.apply_param_to_all.clicked.disconnect(self.on_apply_param_to_all_clicked)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.apply_param_to_all.clicked.connect(self.on_apply_param_to_all_clicked)

        # add a new tool Signals
        try:
            self.ui.search_and_add_btn.clicked.disconnect(self.on_tool_add)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.search_and_add_btn.clicked.connect(lambda: self.on_tool_add())

        try:
            self.ui.addtool_from_db_btn.clicked.disconnect(self.on_ncc_tool_add_from_db_clicked)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.addtool_from_db_btn.clicked.connect(self.on_ncc_tool_add_from_db_clicked)

        try:
            self.app.proj_selection_changed.disconnect(self.on_object_selection_changed)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.app.proj_selection_changed.connect(self.on_object_selection_changed)

        try:
            self.ui.reset_button.clicked.disconnect(self.set_tool_ui)
        except (TypeError, RuntimeError, AttributeError):
            pass
        self.ui.reset_button.clicked.connect(self.set_tool_ui)

        try:
            self.app.cleanup.disconnect(self.set_tool_ui)
        except (TypeError, RuntimeError, AttributeError):
            pass
        # Cleanup on Graceful exit (CTRL+ALT+X combo key)
        self.app.cleanup.connect(self.set_tool_ui)

    def set_tool_ui(self):
        self.units = self.app.app_units.upper()
        self.old_tool_dia = self.app.options["tools_ncc_newdia"]

        self.clear_ui(self.layout)
        self.ui = NccUI(layout=self.layout, app=self.app)
        self.pluginName = self.ui.pluginName

        self.clear_context_menu()
        self.init_context_menu()

        self.form_fields = {
            "tools_ncc_operation":      self.ui.op_radio,
            "tools_ncc_overlap":        self.ui.overlap_entry,
            "tools_ncc_margin":         self.ui.margin_entry,
            "tools_ncc_method":         self.ui.method_combo,
            "tools_ncc_connect":        self.ui.connect_cb,
            "tools_ncc_contour":        self.ui.contour_cb,
            "tools_ncc_offset_choice":  self.ui.offset_choice_cb,
            "tools_ncc_offset_value":   self.ui.offset_entry,
            "tools_ncc_milling_type":   self.ui.milling_type_radio,
            "tools_ncc_check_valid":    self.ui.valid_cb
        }

        # reset the value to prepare for another isolation
        self.safe_tooldia = None

        self.ui.tools_frame.show()

        # use the current selected object and make it visible in the NCC object combobox
        sel_list = self.app.collection.get_selected()
        if len(sel_list) == 1:
            active = self.app.collection.get_active()
            kind = active.kind
            if kind == 'gerber':
                self.ui.type_obj_radio.set_value('gerber')
            else:
                self.ui.type_obj_radio.set_value('geometry')

            # run those once so the obj_type attribute is updated for the FCComboboxes
            # so the last loaded object is displayed
            self.on_type_obj_index_changed(val=kind)
            self.on_reference_combo_changed()

            self.ui.obj_combo.set_value(active.obj_options['name'])
        else:
            kind = 'gerber'
            self.ui.type_obj_radio.set_value('gerber')

            # run those once so the obj_type attribute is updated for the FCComboboxes
            # so the last loaded object is displayed
            self.on_type_obj_index_changed(val=kind)
            self.on_reference_combo_changed()

        self.ui.op_radio.set_value(self.app.options["tools_ncc_operation"])
        self.ui.order_combo.set_value(self.app.options["tools_ncc_order"])
        self.ui.overlap_entry.set_value(self.app.options["tools_ncc_overlap"])
        self.ui.margin_entry.set_value(self.app.options["tools_ncc_margin"])
        self.ui.method_combo.set_value(self.app.options["tools_ncc_method"])
        self.ui.connect_cb.set_value(self.app.options["tools_ncc_connect"])
        self.ui.contour_cb.set_value(self.app.options["tools_ncc_contour"])
        self.ui.offset_choice_cb.set_value(self.app.options["tools_ncc_offset_choice"])
        self.ui.offset_entry.set_value(self.app.options["tools_ncc_offset_value"])

        self.ui.rest_cb.set_value(self.app.options["tools_ncc_rest"])
        self.ui.on_rest_machining_check(state=self.app.options["tools_ncc_rest"])

        self.ui.rest_margin_entry.set_value(self.app.options["tools_ncc_margin"])
        self.ui.rest_connect_cb.set_value(self.app.options["tools_ncc_connect"])
        self.ui.rest_contour_cb.set_value(self.app.options["tools_ncc_contour"])
        self.ui.rest_offset_choice_cb.set_value(self.app.options["tools_ncc_offset_choice"])
        self.ui.rest_offset_entry.set_value(self.app.options["tools_ncc_offset_value"])

        self.ui.select_method_combo.set_value(self.app.options["tools_ncc_ref"])
        self.ui.area_shape_radio.set_value(self.app.options["tools_ncc_area_shape"])
        self.ui.valid_cb.set_value(self.app.options["tools_ncc_check_valid"])

        self.ui.milling_type_radio.set_value(self.app.options["tools_ncc_milling_type"])

        self.ui.new_tooldia_entry.set_value(self.app.options["tools_ncc_newdia"])

        # Show/Hide Advanced Options
        app_mode = self.app.options["global_app_level"]
        self.change_level(app_mode)

        # init the working variables
        self.default_data.clear()
        kind = 'geometry'
        for option in self.app.options:
            if option.find(kind + "_") == 0:
                oname = option[len(kind) + 1:]
                self.default_data[oname] = self.app.options[option]

            if option.find('tools_') == 0:
                self.default_data[option] = self.app.options[option]

        try:
            dias = [float(self.app.options["tools_ncc_tools"])]
        except (ValueError, TypeError):
            try:
                dias = [float(eval(dia)) for dia in self.app.options["tools_ncc_tools"].split(",") if dia != '']
            except AttributeError:
                dias = self.app.options["tools_ncc_tools"]
        except Exception:
            dias = []

        self.tooluid = 0

        self.ncc_tools.clear()
        for tool_dia in dias:
            self.on_tool_add(custom_dia=tool_dia)

        self.obj_name = ""
        self.ncc_obj = None
        self.bound_obj_name = ""
        self.bound_obj = None

        self.tool_type_item_options = ["C1", "C2", "C3", "C4", "B", "V", "L"]
        self.units = self.app.app_units.upper()

        self.first_click = False
        self.cursor_pos = None
        self.mouse_is_dragging = False

        prog_plot = True if self.app.options["tools_ncc_plotting"] == 'progressive' else False
        if prog_plot:
            self.temp_shapes.clear(update=True)

        self.areas_to_clear_list = []

        self.ui.tools_table.drag_drop_sig.connect(self.rebuild_ui)

        self.gen = NccGen(self)
        self.connect_signals()

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

            # Add Tool section
            self.ui.add_tool_frame.hide()

            # Tool parameters section
            if self.ncc_tools:
                for tool in self.ncc_tools:
                    tool_data = self.ncc_tools[tool]['data']

                    tool_data['tools_ncc_operation'] = "clear"
                    tool_data['tools_ncc_milling_type'] = "cl"

                    tool_data['tools_ncc_offset_choice'] = False
                    tool_data['tools_ncc_offset_value'] = 0.0
                    tool_data['tools_ncc_rest'] = False

            self.ui.op_label.hide()
            self.ui.op_radio.hide()
            self.ui.milling_type_label.hide()
            self.ui.milling_type_radio.hide()
            self.ui.offset_choice_cb.hide()
            self.ui.offset_entry.hide()

            self.ui.rest_cb.hide()

            # All param section
            self.ui.apply_param_to_all.hide()

            # Context Menu section
            self.ui.tools_table.removeContextMenu()
        else:
            self.ui.level.setText('%s' % _('Advanced'))
            self.ui.level.setStyleSheet("""
                                        QToolButton
                                        {
                                            color: red;
                                        }
                                        """)

            # Add Tool section
            self.ui.add_tool_frame.show()

            # Tool parameters section
            if self.ncc_tools:
                app_defaults = self.app.options
                for tool in self.ncc_tools:
                    tool_data = self.ncc_tools[tool]['data']

                    tool_data['tools_ncc_operation'] = app_defaults['tools_ncc_operation']
                    tool_data['tools_ncc_milling_type'] = app_defaults['tools_ncc_milling_type']

                    tool_data['tools_ncc_offset_choice'] = app_defaults['tools_ncc_offset_choice']
                    tool_data['tools_ncc_offset_value'] = app_defaults['tools_ncc_offset_value']
                    tool_data['tools_ncc_rest'] = app_defaults['tools_ncc_rest']

            self.ui.op_label.show()
            self.ui.op_radio.show()
            self.ui.milling_type_label.show()
            self.ui.milling_type_radio.show()
            self.ui.offset_choice_cb.show()
            self.ui.offset_entry.show()

            self.ui.rest_cb.show()

            # All param section
            self.ui.apply_param_to_all.show()

            # Context Menu section
            self.ui.tools_table.setupContextMenu()

    def on_type_obj_index_changed(self, val):
        obj_type = 0 if val == 'gerber' else 2
        self.ui.obj_combo.setRootModelIndex(self.app.collection.index(obj_type, 0, QtCore.QModelIndex()))
        self.ui.obj_combo.setCurrentIndex(0)
        self.ui.obj_combo.obj_type = {
            "gerber": "Gerber", "geometry": "Geometry"
        }[self.ui.type_obj_radio.get_value()]

    def on_operation_change(self, val):
        self.ui.parameters_ui(val=val)

        current_row = self.ui.tools_table.currentRow()
        try:
            current_uid = int(self.ui.tools_table.item(current_row, 3).text())
            self.ncc_tools[current_uid]['data']['tools_ncc_operation'] = val
        except (AttributeError, KeyError):
            # there is no row selected or the tool of that row is gone
            return

    def on_object_selection_changed(self, current):
        found_idx = None
        for tab_idx in range(self.app.ui.notebook.count()):
            if self.app.ui.notebook.tabText(tab_idx) == self.ui.pluginName:
                found_idx = True
                break

        if found_idx:
            try:
                name = current.indexes()[0].internalPointer().obj.obj_options['name']
                kind = current.indexes()[0].internalPointer().obj.kind

                if kind in ['gerber', 'geometry']:
                    self.ui.type_obj_radio.set_value(kind)

                self.ui.obj_combo.set_value(name)
            except Exception:
                pass

    def on_toggle_all_rows(self):
        """
        will toggle the selection of all rows in Tools table

        :return:
        """
        sel_model = self.ui.tools_table.selectionModel()
        sel_indexes = sel_model.selectedIndexes()

        # it will iterate over all indexes which means all items in all columns too, but I'm interested only on rows
        sel_rows = set()
        for idx in sel_indexes:
            sel_rows.add(idx.row())

        if len(sel_rows) == self.ui.tools_table.rowCount():
            self.ui.tools_table.clearSelection()
            self.ui.tool_data_label.setText(
                "<b>%s: <font color='#0000FF'>%s</font></b>" % (_('Parameters for'), _("No Tool Selected"))
            )
        else:
            self.ui.tools_table.selectAll()
            self.ui.tool_data_label.setText(
                "<b>%s: <font color='#0000FF'>%s</font></b>" % (_('Parameters for'), _("Multiple Tools"))
            )

    def on_row_selection_change(self):
        sel_model = self.ui.tools_table.selectionModel()
        sel_indexes = sel_model.selectedIndexes()

        # it will iterate over all indexes which means all items in all columns too, but I'm interested only on rows
        sel_rows = set()
        for idx in sel_indexes:
            sel_rows.add(idx.row())

        # update UI only if only one row is selected otherwise having multiple rows selected will deform information
        # for the rows other that the current one (first selected)
        if len(sel_rows) == 1:
            self.update_ui()

    def update_ui(self):
        self.blockSignals(True)

        sel_rows = set()
        table_items = self.ui.tools_table.selectedItems()
        if table_items:
            for it in table_items:
                sel_rows.add(it.row())
            # sel_rows = sorted(set(index.row() for index in self.ui.tools_table.selectedIndexes()))

        if not sel_rows or len(sel_rows) == 0:
            self.ui.generate_button.setDisabled(True)
            self.ui.tool_data_label.setText(
                "<b>%s: <font color='#0000FF'>%s</font></b>" % (_('Parameters for'), _("No Tool Selected"))
            )
            self.blockSignals(False)
            return
        else:
            self.ui.generate_button.setDisabled(False)

        for current_row in sel_rows:
            # populate the form with the data from the tool associated with the row parameter
            try:
                item = self.ui.tools_table.item(current_row, 3)
                if item is not None:
                    tooluid = int(item.text())
                else:
                    return
            except Exception as e:
                self.app.log.error("Tool missing. Add a tool in the Tool Table. %s" % str(e))
                return

            # update the QLabel that shows for which Tool we have the parameters in the UI form
            if len(sel_rows) == 1:
                cr = current_row + 1
                self.ui.tool_data_label.setText(
                    "<b>%s: <font color='#0000FF'>%s %d</font></b>" % (_('Parameters for'), _("Tool"), cr)
                )
                try:
                    # set the form with data from the newly selected tool
                    for tooluid_key, tooluid_value in list(self.ncc_tools.items()):
                        if int(tooluid_key) == tooluid:
                            for key, value in tooluid_value.items():
                                if key == 'data':
                                    self.storage_to_form(tooluid_value['data'])
                except Exception as e:
                    self.app.log.error("ToolNcc ---> update_ui() " + str(e))
            else:
                self.ui.tool_data_label.setText(
                    "<b>%s: <font color='#0000FF'>%s</font></b>" % (_('Parameters for'), _("Multiple Tools"))
                )

        self.blockSignals(False)

    def storage_to_form(self, dict_storage):
        for form_key in self.form_fields:
            for storage_key in dict_storage:
                if form_key == storage_key:
                    try:
                        self.form_fields[form_key].set_value(dict_storage[form_key])
                    except Exception as e:
                        self.app.log.error("ToolNcc.storage_to_form() --> %s" % str(e))
                        pass

    def form_to_storage(self):
        if self.ui.tools_table.rowCount() == 0:
            # there is no tool in tool table, so we can't save the GUI elements values to storage
            return

        self.blockSignals(True)

        widget_changed = self.sender()
        wdg_objname = widget_changed.objectName()
        option_changed = self.name2option[wdg_objname]

        # row = self.ui.tools_table.currentRow()
        rows = sorted(set(index.row() for index in self.ui.tools_table.selectedIndexes()))
        for row in rows:
            if row < 0:
                row = 0
            tooluid_item = int(self.ui.tools_table.item(row, 3).text())

            for tooluid_key, tooluid_val in self.ncc_tools.items():
                if int(tooluid_key) == tooluid_item:
                    new_option_value = self.form_fields[option_changed].get_value()
                    if option_changed in tooluid_val:
                        tooluid_val[option_changed] = new_option_value
                    if option_changed in tooluid_val['data']:
                        tooluid_val['data'][option_changed] = new_option_value

        self.blockSignals(False)

    def on_apply_param_to_all_clicked(self):
        if self.ui.tools_table.rowCount() == 0:
            # there is no tool in tool table, so we can't save the GUI elements values to storage
            self.app.log.debug("ToolNcc.on_apply_param_to_all_clicked() --> no tool in Tools Table, aborting.")
            return

        self.blockSignals(True)

        row = self.ui.tools_table.currentRow()
        if row < 0:
            row = 0

        tooluid_item = int(self.ui.tools_table.item(row, 3).text())
        temp_tool_data = {}

        for tooluid_key, tooluid_val in self.ncc_tools.items():
            if int(tooluid_key) == tooluid_item:
                # this will hold the 'data' key of the self.tools[tool] dictionary that corresponds to
                # the current row in the tool table
                temp_tool_data = tooluid_val['data']
                break

        for tooluid_key, tooluid_val in self.ncc_tools.items():
            tooluid_val['data'] = deepcopy(temp_tool_data)

        # store all the data associated with the row parameter to the self.tools storage
        # tooldia_item = float(self.ui.tools_table.item(row, 1).text())
        # type_item = self.ui.tools_table.cellWidget(row, 2).currentText()
        # operation_type_item = self.ui.tools_table.cellWidget(row, 4).currentText()
        #
        # nccoffset_item = self.ui.offset_choice_cb.get_value()
        # nccoffset_value_item = float(self.ui.offset_entry.get_value())

        # this new dict will hold the actual useful data, another dict that is the value of key 'data'
        # temp_tools = {}
        # temp_dia = {}
        # temp_data = {}
        #
        # for tooluid_key, tooluid_value in self.ncc_tools.items():
        #     for key, value in tooluid_value.items():
        #         if key == 'data':
        #             # update the 'data' section
        #             for data_key in tooluid_value[key].keys():
        #                 for form_key, form_value in self.form_fields.items():
        #                     if form_key == data_key:
        #                         temp_data[data_key] = form_value.get_value()
        #                 # make sure we make a copy of the keys not in the form (we may use 'data' keys that are
        #                 # updated from self.app.options
        #                 if data_key not in self.form_fields:
        #                     temp_data[data_key] = value[data_key]
        #             temp_dia[key] = deepcopy(temp_data)
        #             temp_data.clear()
        #
        #         elif key == 'solid_geometry':
        #             temp_dia[key] = deepcopy(self.tools[tooluid_key]['solid_geometry'])
        #         else:
        #             temp_dia[key] = deepcopy(value)
        #
        #         temp_tools[tooluid_key] = deepcopy(temp_dia)
        #
        # self.ncc_tools.clear()
        # self.ncc_tools = deepcopy(temp_tools)
        # temp_tools.clear()

        self.app.inform.emit('[success] %s' % _("Current Tool parameters were applied to all tools."))

        self.blockSignals(False)

    def rebuild_ui(self):
        # read the table tools uid
        current_uid_list = []
        for row in range(self.ui.tools_table.rowCount()):
            uid = int(self.ui.tools_table.item(row, 3).text())
            current_uid_list.append(uid)

        new_tools = {}
        new_uid = 1

        for current_uid in current_uid_list:
            new_tools[new_uid] = deepcopy(self.ncc_tools[current_uid])
            new_uid += 1

        self.ncc_tools = new_tools

        # the tools table changed therefore we need to rebuild it
        QtCore.QTimer.singleShot(20, self.build_ui)

    def build_ui(self):
        self.ui_disconnect()

        # updated units
        self.units = self.app.app_units.upper()

        sorted_tools = []
        for k, v in self.ncc_tools.items():
            if self.units == "IN":
                sorted_tools.append(float('%.*f' % (self.decimals, float(v['tooldia']))))
            else:
                sorted_tools.append(float('%.*f' % (self.decimals, float(v['tooldia']))))

        order = self.ui.order_combo.get_value()
        if order == 1:  # "Forward"
            sorted_tools.sort(reverse=False)
        elif order == 2:    # "Reverse"
            sorted_tools.sort(reverse=True)
        else:
            pass

        n = len(sorted_tools)
        self.ui.tools_table.setRowCount(n)
        tool_id = 0

        selected_enabled_flag = QtCore.Qt.ItemFlag.ItemIsSelectable | QtCore.Qt.ItemFlag.ItemIsEnabled  # noqa
        enabled_flag = QtCore.Qt.ItemFlag.ItemIsEnabled
        selected_enabled_editable_flag = (
                QtCore.Qt.ItemFlag.ItemIsEditable
                | QtCore.Qt.ItemFlag.ItemIsSelectable   # noqa
                | QtCore.Qt.ItemFlag.ItemIsEnabled
        )

        for tool_sorted in sorted_tools:
            for tooluid_key, tooluid_value in self.ncc_tools.items():
                if float('%.*f' % (self.decimals, tooluid_value['tooldia'])) == tool_sorted:
                    tool_id += 1

                    # ------------------------ Tool ID ----------------------------------------------------------------
                    id_ = QtWidgets.QTableWidgetItem('%d' % int(tool_id))
                    id_.setFlags(selected_enabled_flag)
                    row_no = tool_id - 1
                    self.ui.tools_table.setItem(row_no, 0, id_)  # Tool name/id

                    # ------------------------ Tool Diameter ----------------------------------------------------------
                    # Make sure that the drill diameter when in MM is with no more than self.decimals decimals
                    dia = QtWidgets.QTableWidgetItem('%.*f' % (self.decimals, tooluid_value['tooldia']))
                    dia.setFlags(enabled_flag)
                    self.ui.tools_table.setItem(row_no, 1, dia)  # Diameter

                    # ------------------------ Tool Shape -------------------------------------------------------------
                    tool_type_item = FCComboBox()
                    tool_type_item.addItems(self.tool_type_item_options)
                    idx = int(tooluid_value['data']['tools_mill_tool_shape'])
                    tool_type_item.setCurrentIndex(idx)
                    self.ui.tools_table.setCellWidget(row_no, 2, tool_type_item)

                    # ------------------------ Tool UID - NOT Visible -------------------------------------------------
                    tool_uid_item = QtWidgets.QTableWidgetItem(str(int(tooluid_key)))
                    # ## REMEMBER: THIS COLUMN IS HIDDEN IN OBJECTUI.PY # ##
                    self.ui.tools_table.setItem(row_no, 3, tool_uid_item)  # Tool unique ID

        # make the diameter column editable
        for row in range(tool_id):
            self.ui.tools_table.item(row, 1).setFlags(selected_enabled_editable_flag)

        self.ui.tools_table.resizeColumnsToContents()
        self.ui.tools_table.resizeRowsToContents()

        vertical_header = self.ui.tools_table.verticalHeader()
        vertical_header.hide()
        self.ui.tools_table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        horizontal_header = self.ui.tools_table.horizontalHeader()
        horizontal_header.setMinimumSectionSize(10)
        horizontal_header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeMode.Fixed)
        horizontal_header.resizeSection(0, 20)
        horizontal_header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeMode.Stretch)

        self.ui.tools_table.setMinimumHeight(self.ui.tools_table.getHeight())
        self.ui.tools_table.setMaximumHeight(self.ui.tools_table.getHeight())

        self.ui_connect()

        # set the text on tool_data_label after loading the object
        sel_rows = set()
        sel_items = self.ui.tools_table.selectedItems()
        for it in sel_items:
            sel_rows.add(it.row())
        if len(sel_rows) > 1:
            self.ui.tool_data_label.setText(
                "<b>%s: <font color='#0000FF'>%s</font></b>" % (_('Parameters for'), _("Multiple Tools"))
            )

    def ui_connect(self):
        self.ui.tools_table.itemChanged.connect(self.on_tool_edit)

        # rows selected
        self.ui.tools_table.clicked.connect(self.on_row_selection_change)
        self.ui.tools_table.horizontalHeader().sectionClicked.connect(self.on_toggle_all_rows)

        for row in range(self.ui.tools_table.rowCount()):
            try:
                self.ui.tools_table.cellWidget(row, 2).currentIndexChanged.connect(self.on_tooltable_cellwidget_change)
            except AttributeError:
                pass

        for opt in self.form_fields:
            current_widget = self.form_fields[opt]
            if isinstance(current_widget, FCCheckBox):
                current_widget.stateChanged.connect(self.form_to_storage)
            if isinstance(current_widget, RadioSet):
                current_widget.activated_custom.connect(self.form_to_storage)
            elif isinstance(current_widget, FCDoubleSpinner):
                current_widget.returnPressed.connect(self.form_to_storage)
            elif isinstance(current_widget, FCComboBox):
                current_widget.currentIndexChanged.connect(self.form_to_storage)

        self.ui.rest_cb.stateChanged.connect(self.ui.on_rest_machining_check)
        self.ui.order_combo.currentIndexChanged.connect(self.on_order_changed)

    def ui_disconnect(self):

        try:
            # if connected, disconnect the signal from the slot on item_changed as it creates issues
            self.ui.tools_table.itemChanged.disconnect()
        except (TypeError, AttributeError):
            pass

        for row in range(self.ui.tools_table.rowCount()):

            try:
                self.ui.tools_table.cellWidget(row, 2).currentIndexChanged.disconnect()
            except (TypeError, AttributeError):
                pass

        for opt in self.form_fields:
            current_widget = self.form_fields[opt]
            if isinstance(current_widget, FCCheckBox):
                try:
                    current_widget.stateChanged.disconnect(self.form_to_storage)
                except (TypeError, ValueError):
                    pass
            if isinstance(current_widget, RadioSet):
                try:
                    current_widget.activated_custom.disconnect(self.form_to_storage)
                except (TypeError, ValueError):
                    pass
            elif isinstance(current_widget, FCDoubleSpinner):
                try:
                    current_widget.returnPressed.disconnect(self.form_to_storage)
                except (TypeError, ValueError):
                    pass
            elif isinstance(current_widget, FCComboBox):
                try:
                    current_widget.currentIndexChanged.disconnect(self.form_to_storage)
                except (TypeError, ValueError):
                    pass

        try:
            self.ui.rest_cb.stateChanged.disconnect(self.ui.on_rest_machining_check)
        except (TypeError, ValueError):
            pass
        try:
            self.ui.order_combo.currentIndexChanged.disconnect(self.on_order_changed)
        except (TypeError, ValueError):
            pass

        # rows selected
        try:
            self.ui.tools_table.clicked.disconnect()
        except (TypeError, AttributeError):
            pass
        try:
            self.ui.tools_table.horizontalHeader().sectionClicked.disconnect()
        except (TypeError, AttributeError):
            pass

    def on_reference_combo_changed(self):
        obj_type = self.ui.reference_type_combo.currentIndex()
        self.ui.reference_combo.setRootModelIndex(self.app.collection.index(obj_type, 0, QtCore.QModelIndex()))
        self.ui.reference_combo.setCurrentIndex(0)
        self.ui.reference_combo.obj_type = {0: "Gerber", 1: "Excellon", 2: "Geometry"}[obj_type]

    def on_order_changed(self, order):
        if order != 0:  # "Default"
            self.build_ui()

    def on_tooltable_cellwidget_change(self):
        cw = self.sender()
        assert isinstance(cw, QtWidgets.QComboBox),\
            "Expected a QtWidgets.QComboBox, got %s" % isinstance(cw, QtWidgets.QComboBox)

        cw_index = self.ui.tools_table.indexAt(cw.pos())
        cw_row = cw_index.row()
        cw_col = cw_index.column()

        current_uid = int(self.ui.tools_table.item(cw_row, 3).text())

        # if the sender is in the column with index 2 then we update the tool_type key
        if cw_col == 2:
            tt = cw.currentText()
            typ = 'Iso' if tt == 'V' else 'Rough'

            self.ncc_tools[current_uid].update({
                'type': typ,
                'tool_type': tt,
            })

    def on_find_optimal_tooldia(self):
        self.find_safe_tooldia_worker()

    @staticmethod
    def find_optim_mp(aperture_storage, decimals):
        msg = 'ok'
        total_geo = []

        for ap in list(aperture_storage.keys()):
            if 'geometry' in aperture_storage[ap]:
                for geo_el in aperture_storage[ap]['geometry']:
                    if 'solid' in geo_el and geo_el['solid'] is not None:
                        buff_geo = geo_el['solid'].buffer(0.0000001)
                        if buff_geo.is_valid:
                            total_geo.append(buff_geo)

        total_geo = unary_union(total_geo)
        total_geo = flatten_shapely_geometry(total_geo)

        if len(total_geo) <= 1:
            msg = ('[ERROR_NOTCL] %s' % _("Too few polygons in the Gerber object to determine distances."))
            return msg, np.inf

        min_dict = {}
        idx = 1
        for geo in total_geo:
            for s_geo in total_geo[idx:]:
                # minimize the number of distances by not taking into considerations
                # those that are too small
                dist = geo.distance(s_geo)
                dist = float('%.*f' % (decimals, dist))
                loc_1, loc_2 = nearest_points(geo, s_geo)

                proc_loc = (
                    (float('%.*f' % (decimals, loc_1.x)), float('%.*f' % (decimals, loc_1.y))),
                    (float('%.*f' % (decimals, loc_2.x)), float('%.*f' % (decimals, loc_2.y)))
                )

                if dist in min_dict:
                    min_dict[dist].append(proc_loc)
                else:
                    min_dict[dist] = [proc_loc]

            idx += 1

        min_list = list(min_dict.keys())
        min_dist = min(min_list)
        min_dist -= 10**-decimals  # make sure that this works for isolation case

        return msg, min_dist

    # multiprocessing variant
    def find_safe_tooldia_multiprocessing(self):
        tools_available = self.ui.tools_table.rowCount()
        if tools_available == 0:
            self.app.inform.emit(f'[ERROR_NOTCL] {_("There are no tools in the Tool Table.")}')
            return
        if tools_available > 1:
            sel_table_items = self.ui.tools_table.selectedItems()
            if not sel_table_items:
                self.app.inform.emit(f'[ERROR_NOTCL] {_("There are no tools selected in the Tool Table.")}')
                return
            sel_rows = {t.row() for t in sel_table_items}
            if not sel_rows:
                self.app.inform.emit(f'[ERROR_NOTCL] {_("There are no tools selected in the Tool Table.")}')
                return
        else:
            sel_rows = {0}

        self.units = self.app.app_units.upper()

        # Get source object.
        obj_name = self.ui.obj_combo.currentText()
        try:
            fcobj = self.app.collection.get_by_name(obj_name)
            if fcobj is None:
                self.app.inform.emit('[ERROR_NOTCL] %s: %s' % (_("Object not found"), str(obj_name)))
                return
        except Exception:
            self.app.inform.emit('[ERROR_NOTCL] %s: %s' % (_("Could not retrieve object"), str(obj_name)))
            return

        self.app.inform.emit(_("Checking tools for validity."))

        # find the selected tool ID's
        sorted_tools = []
        for row in sel_rows:
            tid = int(self.ui.tools_table.item(row, 3).text())
            sorted_tools.append(tid)

        def job_thread(app_obj):
            with self.app.proc_container.new(_("Checking ...")):

                ap_storage = fcobj.tools
                p = app_obj.pool.apply_async(self.find_optim_mp, args=(ap_storage, self.decimals))
                res = p.get()

                if res[0] != 'ok':
                    app_obj.inform.emit(res[0])
                    return 'fail'

                min_dist = res[1]

                try:
                    min_dist_truncated = self.app.dec_format(float(min_dist), self.decimals)
                    self.safe_tooldia = min_dist_truncated

                    if not sorted_tools:
                        msg = _("There are no tools selected in the Tool Table.")
                        self.app.inform.emit('[ERROR_NOTCL] %s' % msg)
                        return 'fail'

                    if not self.safe_tooldia:
                        msg = _("Could not find a safe tool diameter.")
                        self.app.log.error(msg)
                        self.app.inform.emit(f'[ERROR_NOTCL] {_("Failed.")}')
                        return 'fail'

                    # check if the tools diameters are less than the safe tool diameter
                    suitable_tools = []
                    for tool in sorted_tools:
                        tool_dia = float(self.ncc_tools[tool]['tooldia'])
                        if tool_dia <= self.safe_tooldia:
                            suitable_tools.append(tool_dia)

                    if not suitable_tools:
                        msg = _("Incomplete isolation. None of the selected tools could do a complete isolation.")
                        self.app.inform.emit('[WARNING] %s' % msg)
                    else:
                        msg = _("At least one of the selected tools can do a complete isolation.")
                        self.app.inform.emit('[success] %s' % msg)

                    # reset the value to prepare for another isolation
                    self.safe_tooldia = None
                except Exception as ee:
                    self.app.log.error(str(ee))
                    return "fail"

                # reset the value to prepare for another isolation
                self.safe_tooldia = None
                self.app.inform.emit("Tool validation passed.")

        self.app.worker_task.emit({'fcn': job_thread, 'params': [self.app]})

    def find_safe_tooldia_worker(self):
        self.app.inform.emit(_("Checking tools for validity."))
        self.units = self.app.app_units.upper()

        obj_name = self.ui.obj_combo.currentText()

        # Get source object.
        try:
            fcobj = self.app.collection.get_by_name(obj_name)
        except Exception:
            self.app.inform.emit('[ERROR_NOTCL] %s: %s' % (_("Could not retrieve object"), str(obj_name)))
            return

        if fcobj is None:
            self.app.inform.emit('[ERROR_NOTCL] %s: %s' % (_("Object not found"), str(obj_name)))
            return

        def job_thread(app_obj):
            with self.app.proc_container.new(_("Checking ...")):
                try:
                    old_disp_number = 0
                    pol_nr = 0
                    app_obj.proc_container.update_view_text(' %d%%' % 0)
                    total_geo = []

                    for ap in list(fcobj.tools.keys()):
                        if 'geometry' in fcobj.tools[ap]:
                            for geo_el in fcobj.tools[ap]['geometry']:
                                if self.app.abort_flag:
                                    # graceful abort requested by the user
                                    raise grace

                                if 'solid' in geo_el and geo_el['solid'] is not None and geo_el['solid'].is_valid:
                                    total_geo.append(geo_el['solid'])

                    total_geo = MultiPolygon(total_geo)
                    total_geo = total_geo.buffer(0)
                    total_geo = flatten_shapely_geometry(total_geo)

                    geo_len = len(total_geo)
                    if geo_len == 1:
                        app_obj.inform.emit(
                            '[ERROR_NOTCL] %s' % _("The Gerber object has one Polygon as geometry.\n"
                                                   "There are no distances between geometry elements to be found."))
                        return 'fail'

                    geo_len = (geo_len * (geo_len - 1)) / 2

                    min_dict = {}
                    idx = 1
                    for geo in total_geo:
                        for s_geo in total_geo[idx:]:
                            if self.app.abort_flag:
                                # graceful abort requested by the user
                                raise grace

                            # minimize the number of distances by not taking into considerations
                            # those that are too small
                            dist = geo.distance(s_geo)
                            dist = float('%.*f' % (self.decimals, dist))
                            loc_1, loc_2 = nearest_points(geo, s_geo)

                            proc_loc = (
                                (float('%.*f' % (self.decimals, loc_1.x)), float('%.*f' % (self.decimals, loc_1.y))),
                                (float('%.*f' % (self.decimals, loc_2.x)), float('%.*f' % (self.decimals, loc_2.y)))
                            )

                            if dist in min_dict:
                                min_dict[dist].append(proc_loc)
                            else:
                                min_dict[dist] = [proc_loc]

                            pol_nr += 1
                            disp_number = int(np.interp(pol_nr, [0, geo_len], [0, 100]))

                            if old_disp_number < disp_number <= 100:
                                app_obj.proc_container.update_view_text(' %d%%' % disp_number)
                                old_disp_number = disp_number
                        idx += 1

                    min_list = list(min_dict.keys())
                    min_dist = min(min_list)

                    min_dist_truncated = self.app.dec_format(float(min_dist), self.decimals)
                    self.safe_tooldia = min_dist_truncated

                    self.optimal_found_sig.emit(min_dist_truncated)

                    l_units = self.units.lower()
                    app_obj.inform.emit(
                        f'[success] {_("Optimal tool diameter found")}: {str(min_dist_truncated)} {l_units}')
                except Exception as ee:
                    app_obj.log.error(str(ee))
                    return

        self.app.worker_task.emit({'fcn': job_thread, 'params': [self.app]})

    def on_tool_add(self, custom_dia=None):
        self.blockSignals(True)

        filename = self.app.tools_database_path()

        new_tools_dict = deepcopy(self.default_data)
        updated_tooldia = None

        # construct a list of all 'tooluid' in the self.iso_tools
        tool_uid_list = [int(tooluid_key) for tooluid_key in self.ncc_tools]

        # find maximum from the temp_uid, add 1 and this is the new 'tooluid'
        max_uid = 0 if not tool_uid_list else max(tool_uid_list)
        tooluid = int(max_uid + 1)

        tool_dias = []
        for k, v in self.ncc_tools.items():
            for tool_v in v.keys():
                if tool_v == 'tooldia':
                    tool_dias.append(self.app.dec_format(v[tool_v], self.decimals))

        # determine the new tool diameter
        if custom_dia is None:
            tool_dia = self.ui.new_tooldia_entry.get_value()
        else:
            tool_dia = custom_dia

        if tool_dia is None or tool_dia == 0:
            self.build_ui()
            self.app.inform.emit(
                '[WARNING_NOTCL] %s' % _("Please enter a tool diameter with non-zero value, in Float format.")
            )
            self.blockSignals(False)
            return

        truncated_tooldia = self.app.dec_format(tool_dia, self.decimals)

        # if new tool diameter already in the Tool List then abort
        if truncated_tooldia in tool_dias:
            self.app.inform.emit(
                '[WARNING_NOTCL] %s %s' % (_("Cancelled."), _("Tool already in Tool Table."))
            )
            self.blockSignals(False)
            return

        # load the database tools from the file
        try:
            with open(filename) as f:
                tools = f.read()
        except IOError:
            self.app.log.error("Could not load tools DB file.")
            self.app.inform.emit('[ERROR] %s' % _("Could not load the file."))
            self.blockSignals(False)
            self.on_tool_default_add(dia=tool_dia)
            return

        try:
            # store here the tools from Tools Database when searching in Tools Database
            tools_db_dict = json.loads(tools)
        except Exception:
            e = sys.exc_info()[0]
            self.app.log.error(str(e))
            self.app.inform.emit('[ERROR] %s' % _("Failed to parse Tools DB file."))
            self.blockSignals(False)
            self.on_tool_default_add(dia=tool_dia)

            return

        tool_found = 0

        # look in database tools
        for db_tool, db_tool_val in tools_db_dict.items():
            db_tooldia = db_tool_val['tooldia']
            low_limit = float(db_tool_val['data']['tol_min'])
            high_limit = float(db_tool_val['data']['tol_max'])

            # we need only tool marked for Isolation Tool
            if db_tool_val['data']['tool_target'] != _('NCC'):
                continue

            # if we find a tool with the same diameter in the Tools DB just update its data
            if truncated_tooldia == db_tooldia:
                tool_found += 1
                for d in db_tool_val['data']:
                    if d.find('tools_ncc_') == 0:
                        new_tools_dict[d] = db_tool_val['data'][d]
                    elif d.find('tools_') == 0:
                        # don't need data for other App Tools; this tests after 'tools_ncc_'
                        continue
                    else:
                        new_tools_dict[d] = db_tool_val['data'][d]
            # search for a tool that has a tolerance that the tool fits in
            elif high_limit >= truncated_tooldia >= low_limit:
                tool_found += 1
                updated_tooldia = db_tooldia
                for d in db_tool_val['data']:
                    if d.find('tools_ncc_') == 0:
                        new_tools_dict[d] = db_tool_val['data'][d]
                    elif d.find('tools_') == 0:
                        # don't need data for other App Tools; this tests after 'tools_ncc_'
                        continue
                    else:
                        new_tools_dict[d] = db_tool_val['data'][d]

        # test we found a suitable tool in Tools Database or if multiple ones
        if tool_found == 0:
            self.app.inform.emit(
                '[WARNING_NOTCL] %s' % _("Tool not in Tools Database. Adding a default tool.")
            )
            self.on_tool_default_add(dia=tool_dia)
            self.blockSignals(False)
            return

        if tool_found > 1:
            self.app.inform.emit(
                f'[WARNING_NOTCL] {_("Cancelled.\nMultiple tools for one tool diameter found in Tools Database.")}')
            self.blockSignals(False)
            return

        # if new tool diameter found in Tools Database already in the Tool List then abort
        if updated_tooldia is not None and updated_tooldia in tool_dias:
            self.app.inform.emit('[WARNING_NOTCL] %s %s' % (_("Cancelled."), _("Tool already in Tool Table.")))
            self.blockSignals(False)
            return

        new_tdia = deepcopy(updated_tooldia) if updated_tooldia is not None else deepcopy(truncated_tooldia)
        self.ncc_tools.update({
            tooluid: {
                'tooldia':          new_tdia,
                'data':             deepcopy(new_tools_dict),
                'solid_geometry':   []
            }
        })
        self.blockSignals(False)
        self.build_ui()

        # select the tool just added
        for row in range(self.ui.tools_table.rowCount()):
            if int(self.ui.tools_table.item(row, 3).text()) == tooluid:
                self.ui.tools_table.selectRow(row)
                break

        # update the UI form
        self.update_ui()

        self.app.inform.emit(
            '[success] %s' % _("New tool added to Tool Table from Tools Database.")
        )

    def on_tool_default_add(self, dia=None, muted=None):
        self.blockSignals(True)
        self.units = self.app.app_units.upper()

        if dia:
            tool_dia = dia
        else:
            tool_dia = self.ui.new_tooldia_entry.get_value()

        if tool_dia is None or tool_dia == 0:
            self.build_ui()
            self.app.inform.emit(
                '[WARNING_NOTCL] %s' % _("Please enter a tool diameter with non-zero value, in Float format.")
            )
            self.blockSignals(False)
            return

        # construct a list of all 'tooluid' in the self.tools
        tool_uid_list = [int(tooluid_key) for tooluid_key in self.ncc_tools]

        # find maximum from the temp_uid, add 1 and this is the new 'tooluid'
        max_uid = 0 if not tool_uid_list else max(tool_uid_list)
        self.tooluid = int(max_uid + 1)

        tool_dias = []
        for k, v in self.ncc_tools.items():
            for tool_v in v.keys():
                if tool_v == 'tooldia':
                    tool_dias.append(float('%.*f' % (self.decimals, (v[tool_v]))))

        truncated_tooldia = self.app.dec_format(tool_dia, self.decimals)
        if truncated_tooldia in tool_dias:
            if muted is None:
                self.app.inform.emit(
                    '[WARNING_NOTCL] %s %s' % (_("Cancelled."), _("Tool already in Tool Table."))
                )
            # self.ui.tools_table.itemChanged.connect(self.on_tool_edit)
            self.blockSignals(False)
            return

        self.ncc_tools.update({
            int(self.tooluid): {
                'tooldia':          truncated_tooldia,
                'data':             deepcopy(self.default_data),
                'solid_geometry':   []
            }
        })

        self.blockSignals(False)
        self.build_ui()

        # select the tool just added
        for row in range(self.ui.tools_table.rowCount()):
            if int(self.ui.tools_table.item(row, 3).text()) == self.tooluid:
                self.ui.tools_table.selectRow(row)
                break

        # update the UI form
        self.update_ui()

        if muted is None:
            self.app.inform.emit('[success] %s' % _("Default tool added to Tool Table."))

    def on_tool_add_by_key(self):
        # tool_add_popup = FCInputDialog(title='%s...' % _("New Tool"),
        #                                text='%s:' % _('Enter a Tool Diameter'),
        #                                min=0.0001, max=10000.0000, decimals=self.decimals)
        btn_icon = QtGui.QIcon(self.app.resource_location + '/open_excellon32.png')

        tool_add_popup = FCInputDialogSpinnerButton(
            title='%s...' % _("New Tool"),
            text='%s:' % _('Enter a Tool Diameter'),
            min=0.0001, max=10000.0000, decimals=self.decimals,
            button_icon=btn_icon,
            callback=self.on_find_optimal_tooldia,
            parent=self.app.ui,
        )
        tool_add_popup.setWindowIcon(QtGui.QIcon(self.app.resource_location + '/letter_t_32.png'))

        def find_optimal(valor):
            tool_add_popup.set_value(float(valor))

        self.optimal_found_sig.connect(find_optimal)

        val, ok = tool_add_popup.get_results()
        if ok:
            if float(val) == 0:
                self.app.inform.emit(
                    '[WARNING_NOTCL] %s' % _("Please enter a tool diameter with non-zero value, in Float format.")
                )
                self.optimal_found_sig.disconnect(find_optimal)
                return
            self.on_tool_add(custom_dia=float(val))
        else:
            self.app.inform.emit(
                '[WARNING_NOTCL] %s...' % _("Adding Tool cancelled")
            )
        self.optimal_found_sig.disconnect(find_optimal)

    def on_tool_edit(self, item):
        self.blockSignals(True)

        edited_row = item.row()
        editeduid = int(self.ui.tools_table.item(edited_row, 3).text())
        tool_dias = []

        try:
            new_tool_dia = float(self.ui.tools_table.item(edited_row, 1).text())
        except ValueError:
            # try to convert comma to decimal point. if it's still not working error message and return
            try:
                new_tool_dia = float(self.ui.tools_table.item(edited_row, 1).text().replace(',', '.'))
            except ValueError:
                self.app.inform.emit(
                    '[ERROR_NOTCL]  %s' % _("Wrong value format entered, use a number.")
                )
                self.blockSignals(False)
                return

        for v in self.ncc_tools.values():
            tool_dias = [
                float('%.*f' % (self.decimals, v[tool_v]))
                for tool_v in v.keys() if tool_v == 'tooldia'
            ]

        # identify the tool that was edited and get it's tooluid
        if new_tool_dia not in tool_dias:
            self.ncc_tools[editeduid]['tooldia'] = deepcopy(float('%.*f' % (self.decimals, new_tool_dia)))
            self.app.inform.emit(
                '[success] %s' % _("Tool from Tool Table was edited.")
            )
            self.blockSignals(False)
            self.build_ui()
            return

        # identify the old tool_dia and restore the text in tool table
        for k, v in self.ncc_tools.items():
            if k == editeduid:
                old_tool_dia = v['tooldia']
                restore_dia_item = self.ui.tools_table.item(edited_row, 1)
                restore_dia_item.setText(str(old_tool_dia))
                break

        self.app.inform.emit(
            '[WARNING_NOTCL] %s' % _("Cancelled. New diameter value is already in the Tool Table.")
        )
        self.blockSignals(False)
        self.build_ui()

    def on_tool_delete(self, rows_to_delete=None, all_tools=None):
        """
        Will delete a tool in the tool table

        :param rows_to_delete:  which rows to delete; can be a list
        :param all_tools:       delete all tools in the tool table
        :return:
        """
        self.blockSignals(True)

        deleted_tools_list = []

        if all_tools:
            self.ncc_tools.clear()
            self.blockSignals(False)
            self.build_ui()
            return

        if rows_to_delete:
            try:
                for row in rows_to_delete:
                    tooluid_del = int(self.ui.tools_table.item(row, 3).text())
                    deleted_tools_list.append(tooluid_del)
            except TypeError:
                tooluid_del = int(self.ui.tools_table.item(rows_to_delete, 3).text())
                deleted_tools_list.append(tooluid_del)

            for t in deleted_tools_list:
                self.ncc_tools.pop(t, None)

            self.blockSignals(False)
            self.build_ui()
            return

        try:
            if self.ui.tools_table.selectedItems():
                for row_sel in self.ui.tools_table.selectedItems():
                    row = row_sel.row()
                    if row < 0:
                        continue
                    tooluid_del = int(self.ui.tools_table.item(row, 3).text())
                    deleted_tools_list.append(tooluid_del)

                for t in deleted_tools_list:
                    self.ncc_tools.pop(t, None)

        except AttributeError:
            self.app.inform.emit('[WARNING_NOTCL] %s' % _("Delete failed. Select a tool to delete."))
            self.blockSignals(False)
            return
        except Exception as e:
            self.app.log.error(str(e))

        self.app.inform.emit('[success] %s' % _("Tools deleted from Tool Table."))
        self.blockSignals(False)
        self.build_ui()

    # To be called after clicking on the plot.
    def on_mouse_release(self, event):
        if self.app.use_3d_engine:
            event_pos = event.pos
            # event_is_dragging = event.is_dragging
            right_button = 2
        else:
            event_pos = (event.xdata, event.ydata)
            # event_is_dragging = self.app.plotcanvas.is_dragging
            right_button = 3

        event_pos = self.app.plotcanvas.translate_coords(event_pos)
        if self.app.grid_status():
            curr_pos = self.app.geo_editor.snap(event_pos[0], event_pos[1])
        else:
            curr_pos = (event_pos[0], event_pos[1])

        x1, y1 = curr_pos[0], curr_pos[1]

        shape_type = self.ui.area_shape_radio.get_value()

        # do clear area only for left mouse clicks
        if event.button == 1:
            if shape_type == "square":
                if self.first_click is False:
                    self.first_click = True
                    self.app.inform.emit('[WARNING_NOTCL] %s' % _("Click the end point of the area."))

                    self.cursor_pos = (curr_pos[0], curr_pos[1])
                else:
                    self.app.inform.emit(_("Zone added. Click to start adding next zone or right click to finish."))
                    self.app.delete_selection_shape()

                    x0, y0 = self.cursor_pos[0], self.cursor_pos[1]

                    pt1 = (x0, y0)
                    pt2 = (x1, y0)
                    pt3 = (x1, y1)
                    pt4 = (x0, y1)

                    new_rectangle = Polygon(
                        [
                            pt1, pt2, pt3, pt4
                        ]
                    )
                    self.areas_to_clear_list.append(new_rectangle)

                    # add a temporary shape on canvas
                    self.draw_tool_selection_shape(old_coords=(x0, y0), coords=(x1, y1))

                    self.first_click = False
                    return
            else:
                self.points.append((x1, y1))

                if len(self.points) > 1:
                    self.poly_drawn = True
                    self.app.inform.emit(_("Click on next Point or click right mouse button to complete ..."))

                return ""
        elif event.button == right_button and self.mouse_is_dragging is False:

            shape_type = self.ui.area_shape_radio.get_value()

            if shape_type == "square":
                self.first_click = False
            else:
                # if we finish to add a polygon
                if self.poly_drawn is True:
                    try:
                        # try to add the point where we last clicked if it is not already in the self.points
                        last_pt = (x1, y1)
                        if last_pt != self.points[-1]:
                            self.points.append(last_pt)
                    except IndexError:
                        pass

                    # we need to add a Polygon and a Polygon can be made only from at least 3 points
                    if len(self.points) > 2:
                        self.delete_moving_selection_shape()
                        pol = Polygon(self.points)
                        # do not add invalid polygons even if they are drawn by utility geometry
                        if pol.is_valid:
                            self.areas_to_clear_list.append(pol)
                            self.draw_selection_shape_polygon(points=self.points)
                            self.app.inform.emit(
                                _("Zone added. Click to start adding next zone or right click to finish.")
                            )

                    self.points = []
                    self.poly_drawn = False
                    return

            self.delete_tool_selection_shape()

            if self.app.use_3d_engine:
                self.app.plotcanvas.graph_event_disconnect('mouse_release', self.on_mouse_release)
                self.app.plotcanvas.graph_event_disconnect('mouse_move', self.on_mouse_move)
                self.app.plotcanvas.graph_event_disconnect('key_press', self.on_key_press)
            else:
                self.app.plotcanvas.graph_event_disconnect(self.mr)
                self.app.plotcanvas.graph_event_disconnect(self.mm)
                self.app.plotcanvas.graph_event_disconnect(self.kp)

            self.app.mp = self.app.plotcanvas.graph_event_connect(
                'mouse_press',
                self.app.on_mouse_click_over_plot,
            )
            self.app.mm = self.app.plotcanvas.graph_event_connect(
                'mouse_move',
                self.app.on_mouse_move_over_plot,
            )
            self.app.mr = self.app.plotcanvas.graph_event_connect(
                'mouse_release',
                self.app.on_mouse_click_release_over_plot,
            )

            # disconnect flags
            self.area_sel_disconnect_flag = False

            if len(self.areas_to_clear_list) == 0:
                return

            self.gen.clear_ncc_area_selection_option(self.areas_to_clear_list)

            self.app.ui.notebook.setDisabled(False)

    # called on mouse move
    def on_mouse_move(self, event):
        shape_type = self.ui.area_shape_radio.get_value()

        if self.app.use_3d_engine:
            event_pos = event.pos
            event_is_dragging = event.is_dragging
        else:
            event_pos = (event.xdata, event.ydata)
            event_is_dragging = self.app.plotcanvas.is_dragging

        curr_pos = self.app.plotcanvas.translate_coords(event_pos)

        # detect mouse dragging motion
        if event_is_dragging is True:
            self.mouse_is_dragging = True
        else:
            self.mouse_is_dragging = False

        # update the cursor position
        if self.app.grid_status():
            # Update cursor
            curr_pos = self.app.geo_editor.snap(curr_pos[0], curr_pos[1])

            self.app.app_cursor.set_data(
                np.asarray([(curr_pos[0], curr_pos[1])]),
                symbol='++', edge_color=self.app.plotcanvas.cursor_color,
                edge_width=self.app.options["global_cursor_width"],
                size=self.app.options["global_cursor_size"],
            )

        if self.cursor_pos is None:
            self.cursor_pos = (0, 0)

        self.app.dx = curr_pos[0] - float(self.cursor_pos[0])
        self.app.dy = curr_pos[1] - float(self.cursor_pos[1])

        # # update the positions on status bar
        # self.app.ui.position_label.setText("&nbsp;<b>X</b>: %.4f&nbsp;&nbsp;   "
        #                                    "<b>Y</b>: %.4f&nbsp;" % (curr_pos[0], curr_pos[1]))
        # self.app.ui.rel_position_label.setText("<b>Dx</b>: %.4f&nbsp;&nbsp;  <b>Dy</b>: "
        #                                        "%.4f&nbsp;&nbsp;&nbsp;&nbsp;" % (self.app.dx, self.app.dy))
        self.app.ui.update_location_labels(
            self.app.dx,
            self.app.dy,
            curr_pos[0],
            curr_pos[1],
        )

        # units = self.app.app_units.lower()
        # self.app.plotcanvas.text_hud.text = \
        #     'Dx:\t{:<.4f} [{:s}]\nDy:\t{:<.4f} [{:s}]\n\nX:  \t{:<.4f} [{:s}]\nY:  \t{:<.4f} [{:s}]'.format(
        #         self.app.dx, units, self.app.dy, units, curr_pos[0], units, curr_pos[1], units)
        self.app.plotcanvas.on_update_text_hud(
            self.app.dx,
            self.app.dy,
            curr_pos[0],
            curr_pos[1],
        )

        # draw the utility geometry
        if shape_type == "square":
            if self.first_click:
                self.app.delete_selection_shape()
                self.app.draw_moving_selection_shape(
                    old_coords=(self.cursor_pos[0], self.cursor_pos[1]),
                    coords=(curr_pos[0], curr_pos[1]),
                )
        else:
            self.delete_moving_selection_shape()
            self.draw_moving_selection_shape_poly(
                points=self.points,
                data=(curr_pos[0], curr_pos[1]),
            )

    def on_key_press(self, event):
        # modifiers = QtWidgets.QApplication.keyboardModifiers()
        # matplotlib_key_flag = False

        # events out of the self.app.collection view (it's about Project Tab) are of type int
        if type(event) is int:
            key = event
        # events from the GUI are of type QKeyEvent
        elif type(event) == QtGui.QKeyEvent:
            key = event.key()
        elif isinstance(event, mpl_key_event):  # MatPlotLib key events are trickier to interpret than the rest
            # matplotlib_key_flag = True

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

        if key == QtCore.Qt.Key.Key_Escape or key == 'Escape':

            if self.area_sel_disconnect_flag is True:
                if self.app.use_3d_engine:
                    self.app.plotcanvas.graph_event_disconnect(
                        'mouse_release',
                        self.on_mouse_release,
                    )
                    self.app.plotcanvas.graph_event_disconnect(
                        'mouse_move',
                        self.on_mouse_move,
                    )
                    self.app.plotcanvas.graph_event_disconnect(
                        'key_press',
                        self.on_key_press,
                    )
                else:
                    self.app.plotcanvas.graph_event_disconnect(self.mr)
                    self.app.plotcanvas.graph_event_disconnect(self.mm)
                    self.app.plotcanvas.graph_event_disconnect(self.kp)

                try:
                    # restore the Grid snapping if it was active before
                    if self.grid_status_memory is True:
                        self.app.ui.grid_snap_btn.trigger()
                    self.app.tool_shapes.clear(update=True)
                except Exception as e:
                    self.app.log.error("ToolNCC.on_key_press() _2 --> %s" % str(e))

                self.app.mp = self.app.plotcanvas.graph_event_connect(
                    'mouse_press',
                    self.app.on_mouse_click_over_plot,
                )
                self.app.mm = self.app.plotcanvas.graph_event_connect(
                    'mouse_move',
                    self.app.on_mouse_move_over_plot,
                )
                self.app.mr = self.app.plotcanvas.graph_event_connect(
                    'mouse_release',
                    self.app.on_mouse_click_release_over_plot,
                )

                self.app.ui.notebook.setDisabled(False)

            self.points = []
            self.poly_drawn = False

            self.delete_moving_selection_shape()
            self.delete_tool_selection_shape()

    def on_ncc_tool_add_from_db_executed(self, tool):
        """
        Here add the tool from DB  in the selected geometry object
        :return:
        """
        tool_from_db = deepcopy(tool)

        if tool['data']['tool_target'] not in [0, 5]:   # [General, NCC]
            for idx in range(self.app.ui.plot_tab_area.count()):
                if self.app.ui.plot_tab_area.tabText(idx) == _("Tools Database"):
                    wdg = self.app.ui.plot_tab_area.widget(idx)
                    wdg.deleteLater()
                    self.app.ui.plot_tab_area.removeTab(idx)
            self.app.inform.emit('[ERROR_NOTCL] %s' % _("Selected tool can't be used here. Pick another."))
            return

        res = self.on_ncc_tool_from_db_inserted(tool=tool_from_db)

        for idx in range(self.app.ui.plot_tab_area.count()):
            if self.app.ui.plot_tab_area.tabText(idx) == _("Tools Database"):
                wdg = self.app.ui.plot_tab_area.widget(idx)
                wdg.deleteLater()
                self.app.ui.plot_tab_area.removeTab(idx)

        if res == 'fail':
            return
        self.app.inform.emit('[success] %s' % _("Tool from DB added in Tool Table."))

        # select last tool added
        toolid = res
        for row in range(self.ui.tools_table.rowCount()):
            if int(self.ui.tools_table.item(row, 3).text()) == toolid:
                self.ui.tools_table.selectRow(row)
        self.on_row_selection_change()

    def on_ncc_tool_from_db_inserted(self, tool):
        """
        Called from the Tools DB object through an App method when adding a tool from Tools Database
        :param tool: a dict with the tool data
        :return: None
        """

        self.ui_disconnect()
        self.units = self.app.app_units.upper()

        tooldia = float(tool['tooldia'])

        # construct a list of all 'tooluid' in the self.tools
        tool_uid_list = [int(tooluid_key) for tooluid_key in self.ncc_tools]

        # find maximum from the temp_uid, add 1 and this is the new 'tooluid'
        max_uid = 0 if not tool_uid_list else max(tool_uid_list)
        tooluid = max_uid + 1

        tool_dias = []
        for k, v in self.ncc_tools.items():
            for tool_v in v.keys():
                if tool_v == 'tooldia':
                    tool_dias.append(self.app.dec_format(v[tool_v], self.decimals))

        truncated_tooldia = self.app.dec_format(tooldia, self.decimals)
        if truncated_tooldia in tool_dias:
            self.app.inform.emit('[WARNING_NOTCL] %s %s' % (_("Cancelled."), _("Tool already in Tool Table.")))
            self.ui_connect()
            return 'fail'

        self.ncc_tools.update({
            tooluid: {
                'tooldia':          truncated_tooldia,
                'data':             deepcopy(tool['data']),
                'solid_geometry':   []
            }
        })
        self.ncc_tools[tooluid]['data']['name'] = '_ncc'

        self.app.inform.emit('[success] %s' % _("New tool added to Tool Table."))

        self.ui_connect()
        self.build_ui()

        # select the tool just added
        for row in range(self.ui.tools_table.rowCount()):
            if int(self.ui.tools_table.item(row, 3).text()) == self.tooluid:
                self.ui.tools_table.selectRow(row)
                break

    def on_ncc_tool_add_from_db_clicked(self):
        """
        Called when the user wants to add a new tool from Tools Database. It will create the Tools Database object
        and display the Tools Database tab in the form needed for the Tool adding
        :return: None
        """

        # if the Tools Database is already opened focus on it
        for idx in range(self.app.ui.plot_tab_area.count()):
            if self.app.ui.plot_tab_area.tabText(idx) == _("Tools Database"):
                self.app.ui.plot_tab_area.setCurrentWidget(self.app.tools_db_tab)
                break
        ret_val = self.app.on_tools_database(source='ncc')
        if ret_val == 'fail':
            return
        self.app.tools_db_tab.ok_to_add = True
        self.app.tools_db_tab.ui.buttons_frame.hide()
        self.app.tools_db_tab.ui.add_tool_from_db.show()
        self.app.tools_db_tab.ui.cancel_tool_from_db.show()

    def reset_fields(self):
        self.ui.obj_combo.setRootModelIndex(self.app.collection.index(0, 0, QtCore.QModelIndex()))
