"""Track initialization, scoring, and deletion helpers.

Part H supplies lidar-driven existence decisions (docs/HUONG_DAN_KY_THUAT.md §2).
Use tracking parameters for the score window, thresholds, and covariance limit.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from fusion_lab.workspace_support import get_tracking_params

params = get_tracking_params()


def init_track_state_from_meas(meas: Any) -> dict[str, Any]:
    """Initialize track state, covariance, lifecycle state, and score from a measurement.

    Args:
        meas: Lidar measurement with ``z``, ``R``, ``sensor``.

    Returns:
        Dict with keys ``x``, ``P``, ``state``, ``score`` (matrices as ``np.matrix``).
    """
    sens_to_veh = np.asarray(meas.sensor.sens_to_veh)
    pos_sens = np.ones((4, 1))
    pos_sens[:3] = np.asarray(meas.z, dtype=float).reshape(3, 1)
    rotation = sens_to_veh[:3, :3]

    x = np.zeros((params.dim_state, 1))
    x[:3] = (sens_to_veh @ pos_sens)[:3]
    P = np.zeros((params.dim_state, params.dim_state))
    P[:3, :3] = rotation @ np.asarray(meas.R) @ rotation.T
    P[3:, 3:] = np.diag([params.sigma_p44**2, params.sigma_p55**2, params.sigma_p66**2])
    return {
        "x": np.asmatrix(x),
        "P": np.asmatrix(P),
        "state": "initialized",
        "score": 1.0 / params.window,
    }


def update_track_score(track: dict[str, Any], associated: bool) -> dict[str, Any]:
    """Update existence once per lidar frame; camera passes never call this helper.

    A hit adds 1/window, capped at one; an in-FOV miss subtracts 1/window.
    Confirm above confirmed_threshold, and preserve confirmed state after misses.

    Args:
        track: Dict-like track with ``score``, ``state``.
        associated: True for a lidar hit; False for a lidar miss within the lidar FOV.

    Returns:
        Updated track dict.
    """
    step = 1.0 / params.window
    if associated:
        track["score"] = min(1.0, track["score"] + step)
        if track["score"] > params.confirmed_threshold:
            track["state"] = "confirmed"
        elif track["state"] != "confirmed":
            track["state"] = "tentative"
    else:
        track["score"] -= step
    return track


def should_delete_track(track: dict[str, Any]) -> bool:
    """Return whether a lidar lifecycle pass should remove this track.

    Delete if either horizontal variance exceeds max_P, or if a confirmed
    track has score < delete_threshold, or an unconfirmed track has score <= 0.
    Camera passes never trigger deletion.

    Args:
        track: Dict with ``score``, ``state``, ``P``.

    Returns:
        True if track should be removed.
    """
    if track["P"][0, 0] > params.max_P or track["P"][1, 1] > params.max_P:
        return True
    if track["state"] == "confirmed":
        return track["score"] < params.delete_threshold
    return track["score"] <= 0
