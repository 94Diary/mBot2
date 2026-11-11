"""Shared settings and angle helpers."""

import math

MATCH_RADIUS_M = 1.00
PEER_TIMEOUT_SECONDS = 4.0
JUNCTION_REACHED_M = 0.20  # robot this close to its junction has arrived
JUNCTION_PASSED_M = 0.40   # ...or this far out along the branch it took
REASSIGN_MARGIN_M = 0.5  # move a reserved branch only to a robot this much closer
CARDINAL_DIRECTIONS = ('E', 'N', 'W', 'S')  # +X, +Y, -X, -Y in the map frame
DIRECTION_SECTORS = len(CARDINAL_DIRECTIONS)


def direction_key(heading):
    """Turn a world-frame heading into a cardinal branch key."""
    return round((heading % (2 * math.pi)) * DIRECTION_SECTORS /
                 (2 * math.pi)) % DIRECTION_SECTORS
