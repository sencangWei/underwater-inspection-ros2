import math
from pathlib import Path
import cv2
import numpy as np
import pytest
import yaml

from underwater_inspection.perception import detect_panel, visual_velocity
from underwater_inspection.planning import clearance, plan_route


CFG = yaml.safe_load((Path(__file__).parents[1] / 'config/mission.yaml').read_text())


def test_route_detours_and_all_segments_preserve_safety_margin():
    route = plan_route(CFG['spawn'][:2], CFG['approach_gate'][:2], CFG)
    assert route[0] == tuple(CFG['spawn'][:2])
    assert route[-1] == tuple(CFG['approach_gate'][:2])
    assert any(abs(p[1]) > 1.5 for p in route)
    for a, b in zip(route, route[1:]):
        for t in np.linspace(0, 1, 200):
            p = [a[k] + t * (b[k] - a[k]) for k in range(2)]
            assert clearance(p, CFG['obstacles'], CFG['robot_radius']) >= CFG['planner_margin'] - .01


def test_blocked_start_is_rejected():
    with pytest.raises(ValueError, match='Start or goal'):
        plan_route((-1.3, 0), (2.5, 0), CFG)


def test_visual_marker_center_and_range_and_background_rejection():
    image = np.zeros((480, 640, 3), np.uint8)
    image[:] = (120, 55, 10)
    assert detect_panel(image, 400, 1.0) is None
    cv2.rectangle(image, (200, 140), (440, 340), (0, 230, 255), -1)
    cv2.circle(image, (320, 240), 35, (0, 0, 220), -1)
    panel = detect_panel(image, 400, 1.0)
    assert abs(panel['cx'] - 320) < 2
    assert abs(panel['cy'] - 240) < 2
    assert math.isclose(panel['range'], 2.0, abs_tol=.02)


def test_actual_underwater_camera_panel_survives_absorption_colour_shift():
    # Actual run04 image: yellow material becomes green after red absorption.
    image = cv2.imread(str(Path(__file__).parent / 'data/panel_underwater.png'))
    panel = detect_panel(image, 417, 1.0)
    assert panel is not None
    assert 280 < panel['cx'] < 420
    assert 180 < panel['cy'] < 320
    assert 2.0 < panel['range'] < 3.5


def test_frontal_visual_servo_stops_at_goal_and_corrects_right_down_offset():
    goal = {'range': 1.3, 'cx': 320, 'cy': 240}
    velocity, _ = visual_velocity(goal, (480, 640), 417, 1.3)
    assert np.allclose(velocity, 0)
    offset = {'range': 2.8, 'cx': 380, 'cy': 260}
    velocity, _ = visual_velocity(offset, (480, 640), 417, 1.3)
    assert 0 < velocity[0] <= .25
    assert velocity[1] > 0 and velocity[2] > 0
