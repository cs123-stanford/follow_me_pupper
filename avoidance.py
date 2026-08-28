"""Obstacle-avoidance decisions, as pure functions of detections.

Nothing in here talks to ROS or moves the robot: every function takes
detections in and hands a judgement back, which is what makes this file
testable at your desk (``python3 test_avoidance.py``) before the robot
ever takes a step.

The one big idea
----------------
The camera is monocular -- no depth. But everything in this lab stands on the
same floor, and the floor recedes *upward* in the image: the further away
something is, the higher its point of contact with the ground appears. So the
**bottom edge of a bounding box is a distance sensor**: a box whose bottom
edge sits low in the frame is close to the robot; one whose bottom edge sits
up near the horizon is far away. Every decision in this file is built out of
that single observation.

(When does it lie to you? Anything not standing on the floor -- a tabletop, a
cat on a couch -- and anything cut off by the frame edge. Keep that in mind
when the robot does something confusing.)

Coordinate frame
----------------
Detections arrive in a fixed reference frame -- the rectified
(equirectangular) image, rescaled to IMAGE_WIDTH x IMAGE_HEIGHT by the
detector -- so the pixel thresholds below mean the same thing whatever
resolution the camera is running.
"""

# The fixed frame /detections is published in (see viser_camera.py).
IMAGE_WIDTH = 700
IMAGE_HEIGHT = 572

# --- When is something "in the way"? ---------------------------------------
# Being between us and the target is not enough to justify a detour: it also
# has to be close (bottom edge near the image bottom) and roughly ahead
# (near the middle of the frame).
OBSTACLE_BOTTOM_MARGIN = 250   # px: bottom edge must be within this of the image bottom
OBSTACLE_CENTER_BAND = 0.15    # |x/W - 0.5|: how far off-centre still counts as in the way
OBSTACLE_MIN_SCORE = 0.3       # ignore shaky detections; a false detour is expensive

# --- Which way to go around? ------------------------------------------------
# A question about the whole frame, not just the one thing in the way: every
# non-target box is tallied onto the side of the image it sits on, and the
# robot turns toward whichever side carries less. Nearer objects weigh more --
# they are the ones it would walk into first.
SIDE_WEIGHT_FAR = 0.5          # weight of an object up near the horizon
SIDE_WEIGHT_NEAR = 2.0         # ... and of one right at the robot's feet
SIDE_SCORE_TIE = 0.25          # closer than this and the two sides count as even


def detection_class_id(detection):
    """Class id of a Detection2D, or None when it cannot be read."""
    if not detection.results:
        return None
    try:
        return int(detection.results[0].hypothesis.class_id)
    except (TypeError, ValueError):
        return None


def box_bottom(detection):
    """y of the bottom edge of a detection's box, in reference-frame pixels.

    This is the number the whole file runs on: larger = lower in the image
    = closer to the robot (see the module docstring for why).
    """
    # ========== YOUR CODE HERE: geometry: box_bottom ==========
    # TODO: the box message gives you a *center* y and a *height*
    # (detection.bbox.center.position.y, detection.bbox.size_y).
    # Return the y of the bottom edge.
    raise NotImplementedError
    # ==========================================================


def box_offset(detection):
    """How far off-centre a detection is: -0.5 hard left, +0.5 hard right, 0 centred."""
    # ========== YOUR CODE HERE: geometry: box_offset ==========
    # TODO: normalize the box's center x by IMAGE_WIDTH so that the
    # middle of the image comes out as 0. This is the error signal
    # your TRACK controller steers on.
    raise NotImplementedError
    # ==========================================================


def box_left_fraction(detection):
    """Fraction of a box's width that falls in the left half of the image.

    A box straddling the middle really is on both sides, so split it between
    them instead of forcing it onto whichever side its centre happens to land
    in. Returns 1.0 for a box entirely left of centre, 0.0 entirely right.
    """
    # ========== YOUR CODE HERE: geometry: box_left_fraction ==========
    # TODO: from the center x and size_x, work out the box's left and
    # right edges, then how much of that width lies left of
    # IMAGE_WIDTH / 2. Watch the edge cases: a box entirely on one
    # side, and a degenerate zero-width box.
    raise NotImplementedError
    # =================================================================


def obstacle_weight(detection):
    """How much a detection counts against the side of the frame it is on.

    Nearer things weigh more: a box whose bottom edge is at the bottom of the
    frame is worth SIDE_WEIGHT_NEAR, one at the very top SIDE_WEIGHT_FAR, and
    everything in between interpolates linearly.
    """
    # ========== YOUR CODE HERE: geometry: obstacle_weight ==========
    # TODO: turn box_bottom into a 0..1 "nearness" (clamp it!), then
    # interpolate between SIDE_WEIGHT_FAR and SIDE_WEIGHT_NEAR.
    raise NotImplementedError
    # ===============================================================


def measure_crowding(others):
    """How much is in each half of the frame, as a (left, right) pair.

    Every detection that is not the target counts, wherever it is -- not just
    the one in the way. Each box is split between the two halves by how much
    of its width lands in each (box_left_fraction) and weighted by how near it
    looks (obstacle_weight), so a chair at the robot's feet outweighs three of
    them across the room. The lighter side is the one worth detouring into.
    """
    # ========== YOUR CODE HERE: avoidance: measure_crowding ==========
    # TODO: sum obstacle_weight * box_left_fraction into the left
    # tally and the remainder into the right one.
    raise NotImplementedError
    # =================================================================


def find_blocking_obstacle(target, others):
    """Offset of the nearest object standing between us and the target.

    Returns the blocking detection's box_offset, or None when the way is
    clear. Three tests, all of which must pass -- and if several objects
    block, the nearest one wins, because it is the one we would hit first.
    """
    # ========== YOUR CODE HERE: avoidance: find_blocking_obstacle ==========
    # TODO: for each non-target detection, decide whether it is
    # actually in the way. It is NOT in the way if any of these hold:
    #   1. its bottom edge is level with or above the target's
    #      (it is beside or beyond the target, not between);
    #   2. its bottom edge is further than OBSTACLE_BOTTOM_MARGIN
    #      from the image bottom (still far off -- keep walking);
    #   3. its offset is outside OBSTACLE_CENTER_BAND (off to a side,
    #      we walk past it anyway).
    # Of the survivors, return the box_offset of the *nearest* one.
    raise NotImplementedError
    # =======================================================================


def choose_side(crowding, obstacle_pos, target_pos):
    """Which way to detour: +1.0 for left, -1.0 for right, and a reason string.

    The side to take is the one with less in it, counting everything the
    camera can see -- walking around one chair into another is no better than
    standing still. Only when the two halves come out even does the single
    blocking obstacle, and then the target, break the tie.
    """
    # ========== YOUR CODE HERE: avoidance: choose_side ==========
    # TODO, in priority order:
    #   1. if the crowding tallies differ by at least SIDE_SCORE_TIE,
    #      go toward the emptier half;
    #   2. else, if the obstacle sits clearly off-centre, step off its
    #      side of the line (it is right of centre -> we go left);
    #   3. else, go around on the side the target is on -- the shorter
    #      way back to it;
    #   4. else there is nothing to choose between: pick one.
    # Return (direction, why) -- the string shows up in the logs, and
    # you will be very glad of it when the robot surprises you.
    raise NotImplementedError
    # ============================================================
