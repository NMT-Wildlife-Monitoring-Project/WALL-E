"""Pure scan-filtering helpers (no ROS imports, so they are unit-testable).

filter_close_returns() removes laser returns that fall inside the robot
footprint: any finite range below min_range is replaced with +inf (treated as
a no-return by downstream consumers like nav2 collision_monitor). Non-finite
values (inf/nan) and ranges at or beyond min_range pass through unchanged.
"""
import math


def filter_close_returns(ranges, min_range):
    """Return a new list with finite ranges < min_range replaced by +inf."""
    out = []
    for r in ranges:
        if math.isfinite(r) and r < min_range:
            out.append(math.inf)
        else:
            out.append(r)
    return out
