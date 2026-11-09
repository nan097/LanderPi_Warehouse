"""Validated pinhole projection. Input depth must be aligned and converted to m."""
from math import isfinite


def project_pixel(u, v, depth_m, fx, fy, cx, cy):
    values = (u, v, depth_m, fx, fy, cx, cy)
    if not all(isfinite(value) for value in values):
        raise ValueError('DEPTH_INVALID or CAMERA_INFO_INVALID: nonfinite input')
    if depth_m <= 0:
        raise ValueError('DEPTH_INVALID: depth must be positive')
    if fx <= 0 or fy <= 0:
        raise ValueError('CAMERA_INFO_INVALID: focal lengths must be positive')
    return ((u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m)
