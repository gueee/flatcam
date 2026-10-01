# PyInstaller spec for a Windows one folder build of FlatCAM EVO.
#
# Build it from the repository root:
#     pyinstaller packaging/flatcam.spec --noconfirm
# or use packaging/build_win.ps1 which also runs Inno Setup.

import os
import sys

from PyInstaller.utils.hooks import collect_data_files, collect_submodules

REPO_ROOT = os.path.abspath(os.path.join(SPECPATH, '..'))

# The application reads these from the folder it was started in. appMain chdir's to sys._MEIPASS when frozen.
datas = [
    (os.path.join(REPO_ROOT, 'assets', 'resources'), os.path.join('assets', 'resources')),
    (os.path.join(REPO_ROOT, 'assets', 'examples'), os.path.join('assets', 'examples')),
    (os.path.join(REPO_ROOT, 'locale'), 'locale'),
    # the preprocessors are loaded as source files at runtime, not imported
    (os.path.join(REPO_ROOT, 'preprocessors'), 'preprocessors'),
    (os.path.join(REPO_ROOT, 'config'), 'config'),
    # VisPy looks for its fonts and its freetype DLL next to the package
    (os.path.join(REPO_ROOT, 'appGUI', 'VisPyData'), os.path.join('appGUI', 'VisPyData')),
    (os.path.join(REPO_ROOT, 'libs', 'qdarktheme', 'themes'), os.path.join('libs', 'qdarktheme', 'themes')),
]

datas += collect_data_files('vispy')
datas += collect_data_files('svglib')
datas += collect_data_files('reportlab')

hiddenimports = [
    'appPreProcessor',
    'shapely',
    'rtree',
    'serial',
    'serial.tools.list_ports',
    'darkdetect',
    'svg.path',
    'svglib',
    'fontTools',
    'ezdxf',
    'dill',
    'simplejson',
    'qrcode',
    'OpenGL',
    'OpenGL.GL',
    'OpenGL.platform.win32',
]

# these packages import their submodules dynamically
for package in ['tclCommands', 'appPlugins', 'appEditors', 'appParsers', 'appObjects', 'appHandlers', 'vispy']:
    hiddenimports += collect_submodules(package)

# Optional packages. They are bundled only when they can be imported in the build environment, because their
# native parts are awkward to install on Windows. The application degrades gracefully without them:
# the Image plugin needs rasterio and the path optimization needs OR-Tools.
for package in ['rasterio', 'osgeo', 'ortools', 'pikepdf', 'svgtrace']:
    try:
        __import__(package)
    except Exception as import_error:
        print('flatcam.spec: skipping optional package %s (%s)' % (package, import_error))
        continue
    print('flatcam.spec: including optional package %s' % package)
    hiddenimports += collect_submodules(package)
    datas += collect_data_files(package)

excludes = [
    'tkinter.test',
    'test',
    'pytest',
    'PyQt6.QtWebEngineCore',
    'PyQt6.QtWebEngineWidgets',
    # a headless browser is not needed and it would pull a Chromium download into the installer
    'pyppeteer',
    'playwright',
]

block_cipher = None

a = Analysis(
    [os.path.join(REPO_ROOT, 'flatcam.py')],
    pathex=[REPO_ROOT],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=excludes,
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='FlatCAM',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=os.path.join(REPO_ROOT, 'assets', 'resources', 'flatcam_icon256.ico'),
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name='FlatCAM',
)
