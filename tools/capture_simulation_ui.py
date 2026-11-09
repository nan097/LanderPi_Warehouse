"""Capture only the Gazebo/RViz windows to verify simulation rendering."""
from pathlib import Path
import argparse
import re
import struct
import subprocess
import ctypes
import time
import numpy as np
from PIL import Image


def decode_xwd(path, png):
    raw = path.read_bytes()
    header = struct.unpack_from('>25I', raw)
    size, width, height = header[0], header[4], header[5]
    bpp, stride = header[11], header[12]
    colors = header[19]
    if bpp not in (16, 24, 32):
        raise ValueError(f'Unsupported screenshot pixel format: {bpp}')
    endian = '<' if header[7] == 0 else '>'
    if bpp == 24:
        data = np.frombuffer(raw, dtype=np.uint8, offset=size + colors * 12,
                             count=height * stride).reshape(height, stride)[:, :width * 3].reshape(height, width, 3)
        if header[7] != 0:
            data = data[:, :, ::-1]
        pixels = data[:, :, 0].astype(np.uint32) | (data[:, :, 1].astype(np.uint32) << 8) | (data[:, :, 2].astype(np.uint32) << 16)
    else:
        pixels = np.frombuffer(raw, dtype=endian + f'u{bpp // 8}', offset=size + colors * 12,
                               count=height * stride // (bpp // 8)).reshape(height, stride // (bpp // 8))[:, :width]
    channels = []
    for mask in header[14:17]:
        shift = (mask & -mask).bit_length() - 1
        channels.append((((pixels & mask) >> shift) * (255.0 / (mask >> shift))).astype(np.uint8))
    Image.fromarray(np.stack(channels, axis=-1)).save(png)


def activate_window(window):
    # Ask the window manager to bring this app forward before X_GetImage.
    # Capturing an occluded XWayland window can otherwise return covering pixels.
    x11 = ctypes.CDLL('libX11.so.6')
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x11.XInternAtom.restype = ctypes.c_ulong
    class Data(ctypes.Union):
        _fields_ = [('l', ctypes.c_long * 5)]
    class ClientMessage(ctypes.Structure):
        _fields_ = [('type', ctypes.c_int), ('serial', ctypes.c_ulong),
                    ('send_event', ctypes.c_int), ('display', ctypes.c_void_p),
                    ('window', ctypes.c_ulong), ('message_type', ctypes.c_ulong),
                    ('format', ctypes.c_int), ('data', Data)]
    class Event(ctypes.Union):
        _fields_ = [('client', ClientMessage), ('pad', ctypes.c_long * 24)]
    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError('Cannot connect to display')
    event = Event()
    event.client.type = 33
    event.client.display = display
    event.client.window = int(window, 16)
    event.client.message_type = x11.XInternAtom(display, b'_NET_ACTIVE_WINDOW', 0)
    event.client.format = 32
    event.client.data.l[0] = 2
    x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long, ctypes.POINTER(Event)]
    x11.XSendEvent(display, x11.XDefaultRootWindow(display), 0, (1 << 20) | (1 << 19), ctypes.byref(event))
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    x11.XFlush(display)
    time.sleep(0.6)
    x11.XCloseDisplay(display)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gazebo-pid', type=int)
    parser.add_argument('--rviz-pid', type=int)
    args = parser.parse_args()
    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    found = {}
    areas = {}
    for line in tree.splitlines():
        match = re.search(r'(0x[0-9a-f]+) "([^"]+)"', line)
        if match:
            title = match.group(2)
            name = 'gazebo' if 'Gazebo' in title else 'rviz' if ('RViz' in title or 'rviz' in title) else None
            if name is None:
                continue
            expected_pid = args.gazebo_pid if name == 'gazebo' else args.rviz_pid
            if expected_pid is not None:
                property_text = subprocess.check_output(['xprop', '-id', match.group(1), '_NET_WM_PID'], text=True)
                actual_pid = re.search(r'= (\d+)', property_text)
                if not actual_pid or int(actual_pid.group(1)) != expected_pid:
                    continue
            dimensions = re.search(r'(\d+)x(\d+)\+', line)
            area = int(dimensions.group(1)) * int(dimensions.group(2)) if dimensions else 0
            if area <= areas.get(name, 10000):
                continue
            info = subprocess.check_output(['xwininfo', '-id', match.group(1)], text=True)
            if 'Map State: IsViewable' not in info:
                continue
            found[name] = match.group(1)
            areas[name] = area
    output = Path('/tmp/landerpi_simulation_preview')
    output.mkdir(exist_ok=True)
    for name in ('rviz', 'gazebo'):
        if name not in found:
            continue
        window = found[name]
        activate_window(window)
        xwd = output / f'{name}.xwd'
        png = output / f'{name}.png'
        subprocess.run(['xwd', '-silent', '-id', window, '-out', str(xwd)], check=True)
        decode_xwd(xwd, png)
        print(f'{name}: {png}')
    if not {'gazebo', 'rviz'} <= found.keys():
        raise RuntimeError(f'Simulation windows missing: {found}')


if __name__ == '__main__':
    main()
