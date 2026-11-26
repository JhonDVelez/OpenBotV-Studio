# -*- mode: python ; coding: utf-8 -*-

import os

from PyInstaller.utils.hooks import collect_data_files

ROOT = os.path.abspath(os.path.join(SPECPATH, '..'))

# Linux resources are collected independently of Windows DLL handling.
a = Analysis(
    [os.path.join(ROOT, 'src', 'main.py')],
    pathex=[ROOT],
    binaries=[],
    datas=[
        (os.path.join(ROOT, 'src', 'resources'), 'src/resources'),
        (os.path.join(ROOT, 'src', 'services', 'styling', 'theme_stylesheet', 'svg'),
         os.path.join('src', 'services', 'styling', 'theme_stylesheet', 'svg')),
        *collect_data_files('qdarktheme'),
    ],
    hiddenimports=[
        'PyQt6.QtQuick',
        'PyQt6.QtQml',
        'PyQt6.QtQuick3D',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='OpenBotV-Studio',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=os.path.join(ROOT, 'src', 'resources', 'img', 'laser_b.png'),
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='OpenBotV-Studio',
)
