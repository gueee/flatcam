# ##########################################################
# FlatCAM: 2D Post-processing for Manufacturing            #
# http://flatcam.org                                       #
# Author: Juan Pablo Caram (c)                             #
# Date: 2/5/2014                                           #
# MIT Licence                                              #
# ##########################################################

from appGUI.GUIElements import FlatCAMActivityView
from PyQt6 import QtCore
import traceback
import weakref

import gettext
import appTranslation as fcTranslate
import builtins

fcTranslate.apply_language('strings')
if '_' not in builtins.__dict__:
    _ = gettext.gettext

# import logging

# log = logging.getLogger('base2')
# #log.setLevel(logging.DEBUG)
# log.setLevel(logging.WARNING)
# #log.setLevel(logging.INFO)
# formatter = logging.Formatter('[%(levelname)s] %(message)s')
# handler = logging.StreamHandler()
# handler.setFormatter(formatter)
# log.addHandler(handler)


class FCProcess(object):

    def __init__(self, descr, app):
        self.app = app
        self.callbacks = {
            "done": []
        }
        self.descr = descr
        self.status = "Active"
        self._done_called = False  # Guard against double callback

    def __del__(self):
        self.done()

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is not None:
            self.app.log.error("Abnormal termination of process!")
            self.app.log.error("".join(traceback.format_exception(exc_type, exc_val, exc_tb)))

        self.done()

    def done(self):
        # Guard against double callback from __exit__ and __del__
        if not self._done_called:
            self._done_called = True
            for fcn in self.callbacks["done"]:
                fcn(self)

    def connect(self, callback, event="done"):
        if callback not in self.callbacks[event]:
            self.callbacks[event].append(callback)

    def disconnect(self, callback, event="done"):
        try:
            self.callbacks[event].remove(callback)
        except ValueError:
            pass

    def set_status(self, status_string):
        self.status = status_string

    def status_msg(self):
        return self.descr


class FCProcessContainer(object):
    """
    This is the process container, or controller (as in MVC)
    of the Process/Activity tracking.

    FCProcessContainer keeps weak references to the FCProcess'es
    such that their __del__ method is called when the user
    looses track of their reference.
    """

    def __init__(self, app):

        self.procs = []
        self.app = app
        self._mutex = QtCore.QMutex()

    def add(self, proc):
        self._mutex.lock()
        try:
            self.procs.append(weakref.ref(proc))
        finally:
            self._mutex.unlock()

    def new(self, descr):
        proc = FCProcess(descr, app=self.app)

        proc.connect(self.on_done, event="done")

        self.add(proc)

        self.on_change(proc)

        return proc

    def on_change(self, proc):
        pass

    def on_done(self, proc):
        self.remove(proc)

    def remove(self, proc):
        # Optimized O(n) removal using list comprehension
        self._mutex.lock()
        try:
            self.procs = [pref for pref in self.procs
                          if pref() is not None and pref() != proc]
        finally:
            self._mutex.unlock()

    def get_procs_count(self):
        """Thread-safe way to get the count of processes."""
        self._mutex.lock()
        try:
            return len(self.procs)
        finally:
            self._mutex.unlock()

    def get_first_proc(self):
        """Thread-safe way to get the first process (if any)."""
        self._mutex.lock()
        try:
            if self.procs:
                return self.procs[0]()
            return None
        finally:
            self._mutex.unlock()


class FCVisibleProcessContainer(FCProcessContainer, QtCore.QObject):
    something_changed = QtCore.pyqtSignal()
    # this will signal that the application is IDLE
    idle_flag = QtCore.pyqtSignal()
    # Private signal for thread-safe update request (marshals worker -> main thread)
    _update_requested = QtCore.pyqtSignal()

    def __init__(self, view, app):
        assert isinstance(view, FlatCAMActivityView), \
            "Expected a FlatCAMActivityView, got %s" % type(view)

        QtCore.QObject.__init__(self)
        FCProcessContainer.__init__(self, app=app)

        self.view = view

        self.text_to_display_in_activity = ''
        self.new_text = ' '

        # Coalescing timer for UI updates - lives on main thread only
        self._update_timer = QtCore.QTimer(self)
        self._update_timer.setSingleShot(True)
        self._update_timer.timeout.connect(self._do_coalesced_update)
        self._update_timer.setInterval(50)  # 50ms coalescing window

        # Signal->slot: marshals from worker thread to main thread via AutoConnection
        self._update_requested.connect(self._handle_update_request)
        self.something_changed.connect(self.update_view)

    def _handle_update_request(self):
        """Slot runs on main thread (via AutoConnection). Safe to touch QTimer."""
        if not self._update_timer.isActive():
            self._update_timer.start()

    def _do_coalesced_update(self):
        """Timer callback on main thread - emits the coalesced signal."""
        self.something_changed.emit()

    def on_done(self, proc):
        # self.app.log.debug("FCVisibleProcessContainer.on_done()")
        super(FCVisibleProcessContainer, self).on_done(proc)

        self._update_requested.emit()

    def on_change(self, proc):
        # self.app.log.debug("FCVisibleProcessContainer.on_change()")
        super(FCVisibleProcessContainer, self).on_change(proc)

        # whenever there is a change update the message on activity
        # Use thread-safe method to get first proc
        first_proc = self.get_first_proc()
        if first_proc:
            self.text_to_display_in_activity = first_proc.status_msg()

        self._update_requested.emit()

    def update_view(self):
        # Use thread-safe method to get procs count
        procs_count = self.get_procs_count()

        if procs_count == 0:
            self.new_text = ''
            self.view.set_idle()
            self.idle_flag.emit()

        elif procs_count == 1:
            self.view.set_busy(self.text_to_display_in_activity + self.new_text)
        else:
            self.view.set_busy("%d %s" % (procs_count, _("processes running.")))

    def update_view_text(self, new_text, clear=False):
        # this has to be called after the method 'new' inherited by this class is called with a string text as param
        self.new_text = new_text
        # Use thread-safe method to get procs count
        if self.get_procs_count() == 1:
            if clear is False:
                self.view.set_busy(self.text_to_display_in_activity + self.new_text, no_movie=True)
            else:
                self.view.set_busy(self.new_text, no_movie=True)
