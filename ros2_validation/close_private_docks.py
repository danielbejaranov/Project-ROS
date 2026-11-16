#!/usr/bin/env python3
"""Close the two RViz Displays docks on this recording's private X server."""
import ctypes,sys,time,re
name=sys.argv[1]
assert re.fullmatch(r':\d+',name) and name!=':1'
x=ctypes.CDLL('libX11.so.6');t=ctypes.CDLL('libXtst.so.6')
x.XOpenDisplay.argtypes=[ctypes.c_char_p];x.XOpenDisplay.restype=ctypes.c_void_p
x.XFlush.argtypes=[ctypes.c_void_p];x.XCloseDisplay.argtypes=[ctypes.c_void_p]
t.XTestFakeMotionEvent.argtypes=[ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_ulong]
t.XTestFakeButtonEvent.argtypes=[ctypes.c_void_p,ctypes.c_uint,ctypes.c_int,ctypes.c_ulong]
display=x.XOpenDisplay(name.encode());assert display
for px in (330,1411):
 t.XTestFakeMotionEvent(display,-1,px,72,0)
 t.XTestFakeButtonEvent(display,1,1,0);t.XTestFakeButtonEvent(display,1,0,0)
 x.XFlush(display);time.sleep(.3)
x.XCloseDisplay(display)
