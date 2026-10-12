"""Small mapping helpers shared by the XR adapters, without the legacy utility dependencies."""

import numpy as np


def order_ranges(range_dict, output_order):
    """Return dictionary values arranged in the requested joint order."""
    return np.asarray([range_dict[label] for label in output_order])


def map_normalized_to_range(values, minimums, maximums):
    """Map normalized values in [0, 1] into per-axis ranges."""
    values = np.asarray(values, dtype=np.float32).reshape(-1)
    minimums = np.asarray(minimums, dtype=np.float32).reshape(-1)
    maximums = np.asarray(maximums, dtype=np.float32).reshape(-1)
    if not (values.size == minimums.size == maximums.size):
        raise ValueError("Values and limit arrays must have matching lengths.")
    return minimums + np.clip(values, 0.0, 1.0) * (maximums - minimums)


def follower_to_config_space(values):
    """Reorder 10 follower-state axes to the STRAS config-space order."""
    values = np.asarray(values, dtype=np.float32).reshape(-1)
    if values.size < 10:
        raise ValueError("Follower state must provide at least 10 joint values.")
    return np.asarray(
        [values[6], values[5], values[4],
         values[9], values[8], values[7],
         values[0], values[1], values[2], values[3]],
        dtype=np.float32,
    )
