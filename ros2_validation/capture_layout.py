"""Arrange real RViz windows and an actual log terminal on a private X display.

This module changes presentation only. The terminal follows the unmodified
stdout/stderr logs of the live processes, including the auditor's real output.
"""
import ctypes
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import time
import yaml

WIDTH, HEIGHT = 1920, 1080
OVERVIEW = (0, 0, 1050, 1080)
FOLLOW = (1050, 0, 870, 650)
CONSOLE = (1050, 650, 870, 430)


def configure_capture_views(overview_path, follow_path, console=True, follow_scale=35., out_dir=None):
    """Copy to out_dir then adjust, or adjust already-generated per-run files.

    Without out_dir, callers must pass their generated copies, not source RViz
    configs. This lets a runner preserve its scenario-specific displays.
    """
    overview_path, follow_path = Path(overview_path), Path(follow_path)
    if out_dir is not None:
        destination=Path(out_dir);destination.mkdir(parents=True,exist_ok=True)
        copies=[]
        for source in (overview_path,follow_path):
            target=destination/source.name
            if target.resolve()!=source.resolve():shutil.copyfile(source,target)
            copies.append(target)
        overview_path,follow_path=copies
    for path, rectangle in ((overview_path, OVERVIEW), (follow_path, FOLLOW if console else (1050, 0, 870, 1080))):
        data = yaml.safe_load(path.read_text())
        x, y, width, height = rectangle
        data.setdefault('Window Geometry', {}).update({'X':x, 'Y':y, 'Width':width, 'Height':height,
                                                       'Hide Left Dock':True, 'Hide Right Dock':True})
        if path == follow_path:
            data['Visualization Manager']['Views']['Current']['Scale'] = float(follow_scale)
        path.write_text(yaml.safe_dump(data, sort_keys=False))
    return overview_path, follow_path


def resolve_terminal(binary=None):
    """XTERM may point to a portable xterm executable without shell parsing."""
    binary = binary or os.environ.get('XTERM') or shutil.which('xterm')
    if not binary:
        raise RuntimeError('xterm is required for --console-layout; install xterm or set XTERM to its executable')
    candidate = shutil.which(str(binary)) or str(Path(binary).expanduser())
    if not Path(candidate).is_file() or not os.access(candidate, os.X_OK):
        raise FileNotFoundError(f'xterm executable is unavailable: {candidate}')
    return candidate


def start_console(run, out, project_dir, command_hint, xterm=None,
                  log_names=('monitor.log', 'audit_result.log'), mode=None):
    """Launch xterm with a real tail -F of process logs using relative headers.

    `command_hint` must describe the command actually used for this run. It is
    printed as a label, never presented as a shell session that did not occur.
    Run from a neutral project directory if absolute paths printed by ROS must
    be neutral; this helper deliberately does not suppress or rewrite errors.
    """
    out, project_dir = Path(out).resolve(), Path(project_dir)
    logs = out / 'logs'; logs.mkdir(parents=True, exist_ok=True)
    for name in log_names:
        if Path(name).name != name:
            raise ValueError('Log names must be local basenames')
        (logs / name).touch(exist_ok=True)
    script = out / 'capture_console.sh'
    lines = ['#!/usr/bin/env bash', 'set -euo pipefail',
             'export TERM=xterm-256color',
             "printf '%s\\n' 'ROS 2 Humble | ejecución real del simulador'",
             "printf '%s\\n' " + shlex.quote('Directorio del proyecto: ' + str(project_dir)),
             "printf '%s\\n' 'Comando de esta ejecución:' " + shlex.quote(command_hint),
             "printf '%s\\n' " + shlex.quote('Modo: ' + (str(mode) if mode else 'ventanas reales de RViz')),
             "printf '%s\\n' 'Esperando controlador; salida viva de los procesos:' ''",
             'cd -- ' + shlex.quote(str(logs)),
             'exec tail -n 8 -F -- ' + ' '.join(shlex.quote(n) for n in log_names)]
    script.write_text('\n'.join(lines) + '\n')
    command = [resolve_terminal(xterm), '-ut', '-u8', '-T', 'Project-ROS | consola de ejecucion',
               '-n', 'Project-ROS', '-bg', '#071019', '-fg', '#dcf6e6',
               '-fa', 'Monospace', '-fs', '10', '-sb', '-sl', '4000',
               '-geometry', '108x25+1050+650', '-e', '/bin/bash', str(script)]
    return run.start('terminal', command)


def arrange_capture_windows(display_name, console=True):
    """Apply exact pixel rectangles and neutral titles on the owned Xvfb only."""
    if not (display_name.startswith(':') and display_name[1:].isdigit()) or display_name in (':0', ':1'):
        raise ValueError('Layout changes are restricted to a private numbered X display')
    x = ctypes.CDLL('libX11.so.6')
    window = ctypes.c_ulong
    x.XOpenDisplay.argtypes = [ctypes.c_char_p]; x.XOpenDisplay.restype = ctypes.c_void_p
    x.XDefaultRootWindow.argtypes = [ctypes.c_void_p]; x.XDefaultRootWindow.restype = window
    x.XQueryTree.argtypes = [ctypes.c_void_p,window,ctypes.POINTER(window),ctypes.POINTER(window),ctypes.POINTER(ctypes.POINTER(window)),ctypes.POINTER(ctypes.c_uint)]
    x.XFetchName.argtypes = [ctypes.c_void_p,window,ctypes.POINTER(ctypes.c_void_p)]
    x.XGetGeometry.argtypes = [ctypes.c_void_p,window,ctypes.POINTER(window),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_uint),ctypes.POINTER(ctypes.c_uint),ctypes.POINTER(ctypes.c_uint),ctypes.POINTER(ctypes.c_uint)]
    x.XMoveResizeWindow.argtypes = [ctypes.c_void_p,window,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint]
    x.XStoreName.argtypes = [ctypes.c_void_p,window,ctypes.c_char_p]
    x.XRaiseWindow.argtypes = [ctypes.c_void_p,window]
    x.XFlush.argtypes = [ctypes.c_void_p]; x.XFree.argtypes = [ctypes.c_void_p]; x.XCloseDisplay.argtypes = [ctypes.c_void_p]
    handle = x.XOpenDisplay(display_name.encode())
    if not handle: raise RuntimeError('Cannot open the private capture display')
    found = []
    try:
        root = x.XDefaultRootWindow(handle); returned_root = window(); parent = window()
        children = ctypes.POINTER(window)(); count = ctypes.c_uint()
        if not x.XQueryTree(handle,root,ctypes.byref(returned_root),ctypes.byref(parent),ctypes.byref(children),ctypes.byref(count)):
            raise RuntimeError('Cannot query capture display windows')
        ids = [int(children[i]) for i in range(count.value)]
        if children:x.XFree(children)
        for identifier in ids:
            title_pointer = ctypes.c_void_p()
            title = ''
            if x.XFetchName(handle,identifier,ctypes.byref(title_pointer)) and title_pointer.value:
                title = ctypes.string_at(title_pointer).decode('utf-8',errors='replace'); x.XFree(title_pointer)
            px, py = ctypes.c_int(), ctypes.c_int(); w,h,border,depth = [ctypes.c_uint() for _ in range(4)]
            if x.XGetGeometry(handle,identifier,ctypes.byref(returned_root),ctypes.byref(px),ctypes.byref(py),ctypes.byref(w),ctypes.byref(h),ctypes.byref(border),ctypes.byref(depth)) and w.value>200 and h.value>150:
                found.append({'id':identifier,'title':title,'x':px.value,'width':w.value,'height':h.value})
        rviz = sorted([item for item in found if 'rviz' in item['title'].lower()],key=lambda item:item['x'])
        if len(rviz)!=2:raise RuntimeError(f'Expected two real RViz windows, found {len(rviz)}: {found}')
        placements = [(rviz[0],OVERVIEW,'Project-ROS | mapa completo RViz'),
                      (rviz[1],FOLLOW if console else (1050,0,870,1080),'Project-ROS | seguimiento RViz')]
        if console:
            terminal = [item for item in found if 'consola de ejecuci' in item['title']]
            if len(terminal)!=1:raise RuntimeError(f'Expected one real execution terminal: {found}')
            placements.append((terminal[0],CONSOLE,'Project-ROS | consola de ejecucion'))
        result=[]
        for item,rectangle,title in placements:
            px,py,w,h=rectangle
            x.XMoveResizeWindow(handle,item['id'],px,py,w,h)
            x.XStoreName(handle,item['id'],title.encode());x.XRaiseWindow(handle,item['id'])
            result.append({'window_id':item['id'],'title':title,'rectangle':list(rectangle)})
        x.XFlush(handle)
        return result
    finally:x.XCloseDisplay(handle)


def close_capture_docks(display_name):
    """Close the Displays docks at offsets of the owned two-view layout."""
    if not (display_name.startswith(':') and display_name[1:].isdigit()) or display_name in (':0',':1'):
        raise ValueError('Dock changes are restricted to a private X display')
    # Locate and align the actual RViz windows first. A final arrangement can
    # then restore the shorter follow view above the console.
    placements=arrange_capture_windows(display_name,console=False)
    x=ctypes.CDLL('libX11.so.6');t=ctypes.CDLL('libXtst.so.6')
    x.XOpenDisplay.argtypes=[ctypes.c_char_p];x.XOpenDisplay.restype=ctypes.c_void_p
    x.XFlush.argtypes=[ctypes.c_void_p];x.XCloseDisplay.argtypes=[ctypes.c_void_p]
    t.XTestFakeMotionEvent.argtypes=[ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_ulong]
    t.XTestFakeButtonEvent.argtypes=[ctypes.c_void_p,ctypes.c_uint,ctypes.c_int,ctypes.c_ulong]
    handle=x.XOpenDisplay(display_name.encode())
    if not handle:raise RuntimeError('Cannot open the private capture display')
    try:
        for item in placements:
            px,py,_,_=item['rectangle']
            t.XTestFakeMotionEvent(handle,-1,px+330,py+72,0)
            t.XTestFakeButtonEvent(handle,1,1,0);t.XTestFakeButtonEvent(handle,1,0,0)
            x.XFlush(handle);time.sleep(.25)
    finally:x.XCloseDisplay(handle)


def record_audit_result(out, checks_path, returncode):
    """Append a visible verdict derived only from the actual completed audit."""
    try:
        result = json.loads(Path(checks_path).read_text()) if Path(checks_path).is_file() else {}
    except (OSError,json.JSONDecodeError) as error:
        result={'error':f'{type(error).__name__}: {error}'}
    checks = result.get('checks', {})
    passed = returncode == 0 and result.get('passed') is True and bool(checks) and all(checks.values())
    lines=['', 'AUDITORÍA REAL DEL ROSBAG: ' + ('PASS' if passed else 'FAIL'),
           f'Comprobaciones aprobadas: {sum(bool(v) for v in checks.values())}/{len(checks)}']
    if result.get('error'):lines.append(str(result['error']))
    if not passed:
        lines.append(f'Código de salida del auditor: {returncode}')
        if not checks:
            audit_log=Path(out)/'logs/audit.log'
            if audit_log.is_file():lines.extend(audit_log.read_text(errors='replace').splitlines()[-30:])
    for name,vehicle in result.get('vehicles',{}).items():
        count=vehicle.get('actual_lap_count')
        if count is not None:lines.append(f'{name.upper()}: {count} vueltas reales; extras: {vehicle.get("extra_lap_count",0)}')
    failed=[name for name,value in checks.items() if not value]
    if failed:lines.append('Fallos: '+', '.join(failed))
    with (Path(out)/'logs/audit_result.log').open('a') as stream:
        stream.write('\n'.join(lines)+'\n');stream.flush()
    return passed
