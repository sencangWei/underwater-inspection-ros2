"""Deterministic marker-panel baseline; no trained neural model is claimed."""
import cv2
import numpy as np


def detect_panel(bgr, focal_length, panel_height):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # The simulator models wavelength-dependent absorption. A yellow panel
    # becomes green underwater; exclude the blue/cyan water background.
    mask = cv2.inRange(hsv, np.array([12, 65, 45]), np.array([85, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if h >= 14 and w >= 14 and area > 250 and .55 < w / h < 1.8 and area / (w * h) > .45:
            candidates.append((area, x, y, w, h))
    if not candidates:
        return None
    area, x, y, w, h = max(candidates)
    return {'bbox': (x, y, w, h), 'cx': x + w / 2, 'cy': y + h / 2,
            'range': focal_length * panel_height / h, 'area': area}


def visual_velocity(panel, image_shape, focal_length, standoff):
    """Body FRD velocity for a frontal panel, with yaw regulated separately."""
    h, w = image_shape[:2]
    errors = np.array([panel['range'] - standoff,
                       (panel['cx'] - w / 2) * panel['range'] / focal_length,
                       (panel['cy'] - h / 2) * panel['range'] / focal_length])
    return np.clip(.3 * errors, [-.15, -.2, -.12], [.25, .2, .12]), errors
