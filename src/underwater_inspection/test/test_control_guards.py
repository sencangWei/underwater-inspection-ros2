import time
from types import SimpleNamespace
import numpy as np

from underwater_inspection.mission import InspectionMission
from stonefish_control.nodes.hybrid_controller_node import HybridController4DOFNode


def test_losing_panel_resets_continuous_hold_and_commands_stop():
    commands = []
    state = SimpleNamespace(inspect_good=7.0, pose=[3.8, 0, 2.5],
                            panel_wall=time.monotonic(),
                            send_pose=lambda *args: commands.append(args))
    InspectionMission.stop_for_missing_panel(state)
    assert state.inspect_good == 0.0
    assert commands == [([3.8, 0, 2.5], 0.0)]
    # One new second after the interruption cannot satisfy the 8-second gate.
    assert state.inspect_good + 1.0 < 8.0


def test_controller_zeroes_wrench_when_mission_dies_but_odometry_stays_fresh():
    published = []
    state = SimpleNamespace(odom_received=True, cmd_received=True,
                            last_odom_wall=time.monotonic(), last_cmd_wall=time.monotonic() - 1,
                            wrench_pub=SimpleNamespace(publish=published.append))
    HybridController4DOFNode.control_loop(state)
    assert len(published) == 1
    wrench = published[0].wrench
    assert np.allclose([wrench.force.x, wrench.force.y, wrench.force.z,
                        wrench.torque.x, wrench.torque.y, wrench.torque.z], 0)
