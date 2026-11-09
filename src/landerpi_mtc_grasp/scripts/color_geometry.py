"""RGB-D color segmentation, independent of ROS and Gazebo model coordinates."""
import cv2
import numpy as np

COLORS = {
    'red': [((0, 130, 60), (12, 255, 255)), ((170, 130, 60), (179, 255, 255))],
    'green': [((35, 100, 50), (85, 255, 255))],
    'blue': [((90, 100, 50), (130, 255, 255))],
}


def locate(bgr, depth, encoding, k, color):
    if color not in COLORS:
        raise ValueError('color must be red, green or blue')
    if bgr.shape[:2] != depth.shape:
        raise ValueError('RGB and registered depth dimensions differ')
    if encoding not in ('32FC1', '16UC1'):
        raise ValueError('unsupported depth encoding')
    if len(k) != 9 or not np.isfinite(k).all() or k[0] <= 0 or k[4] <= 0:
        raise ValueError('invalid camera intrinsics')
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(depth.shape, np.uint8)
    for low, high in COLORS[color]:
        mask |= cv2.inRange(hsv, low, high)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError(f'no {color} cube visible')
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 12:
        raise ValueError('color region too small')
    moments = cv2.moments(contour)
    u, v = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
    interior = np.zeros_like(mask)
    cv2.drawContours(interior, [contour], -1, 255, -1)
    interior = cv2.erode(interior, np.ones((3, 3), np.uint8))
    samples = depth[interior > 0].astype(float)
    if encoding == '16UC1':
        samples *= .001
    samples = samples[np.isfinite(samples) & (samples > .05) & (samples < 3.)]
    if len(samples) < 5:
        raise ValueError('insufficient valid target depth')
    z = float(np.median(samples))
    return ((u-k[2])*z/k[0], (v-k[5])*z/k[4], z), (u, v), mask


def colored_cloud(bgr, depth, encoding, k, color):
    """Registered depth samples inside the largest color region, in camera frame."""
    _, pixel, mask = locate(bgr, depth, encoding, k, color)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    selected = np.zeros_like(mask)
    cv2.drawContours(selected, [max(contours, key=cv2.contourArea)], -1, 255, -1)
    selected = cv2.erode(selected, np.ones((3, 3), np.uint8))
    metric = depth.astype(float) * (.001 if encoding == '16UC1' else 1.)
    v, u = np.nonzero((selected > 0) & np.isfinite(metric) & (metric > .05) & (metric < 3.))
    z = metric[v, u]
    return np.column_stack(((u-k[2])*z/k[0], (v-k[5])*z/k[4], z)), pixel


def transform_cloud(points, translation, quaternion):
    """Rigid camera-to-map transform; quaternion ordered x,y,z,w."""
    q = np.asarray(quaternion, dtype=float)
    q /= np.linalg.norm(q)
    xyz, w = q[:3], q[3]
    cross = np.cross(xyz, points)
    return points + 2.*(w*cross + np.cross(xyz, cross)) + np.asarray(translation)


def upright_center(points, height=.05):
    """Recover upright cube center from visible top, ignoring oblique side faces."""
    if len(points) < 12 or not np.isfinite(points).all():
        raise ValueError('insufficient finite cube points')
    top = points[points[:, 2] >= np.percentile(points[:, 2], 85)-.002]
    if len(top) < 5:
        raise ValueError('insufficient visible top surface')
    center = np.array([(np.percentile(top[:, axis], 5)+np.percentile(top[:, axis], 95))/2
                       for axis in (0, 1)] + [np.median(top[:, 2])-height/2])
    if not .01 < center[2] < .04:
        raise ValueError('detected cube is not resting on the ground')
    return center
