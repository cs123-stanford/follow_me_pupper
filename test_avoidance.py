#!/usr/bin/env python3
"""Desk-side tests for avoidance.py — no robot, no ROS, no camera.

Run it after every function you implement:

    python3 test_avoidance.py

Each scene is a handful of fake detections laid out like a real frame would
be. If a test fails, the printout tells you what the scene was and what your
code decided — reproduce it in your head against the picture in the
avoidance.py docstring before touching the code.
"""

import avoidance
from avoidance import IMAGE_HEIGHT, IMAGE_WIDTH


class _Position:
    def __init__(self, x, y):
        self.x, self.y = x, y


class _BBox:
    def __init__(self, cx, cy, w, h):
        self.center = type("C", (), {"position": _Position(cx, cy)})()
        self.size_x, self.size_y = w, h


class _Hypothesis:
    def __init__(self, class_id, score):
        self.class_id, self.score = str(class_id), score


class _Result:
    def __init__(self, class_id, score):
        self.hypothesis = _Hypothesis(class_id, score)


class FakeDetection:
    """Quacks like a vision_msgs Detection2D, built from box corners."""

    def __init__(self, x1, y1, x2, y2, class_id=56, score=0.9):
        self.bbox = _BBox((x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1)
        self.results = [_Result(class_id, score)]
        self.id = ""


PASS = FAIL = 0


def check(name, got, want, tol=None):
    global PASS, FAIL
    ok = (abs(got - want) <= tol) if tol is not None else (got == want)
    if ok:
        PASS += 1
        print(f"  ok   {name}")
    else:
        FAIL += 1
        print(f"  FAIL {name}: got {got!r}, wanted {want!r}")


def main():
    W, H = IMAGE_WIDTH, IMAGE_HEIGHT

    print("geometry:")
    d = FakeDetection(300, 200, 400, 500)
    check("box_bottom is the bottom edge", avoidance.box_bottom(d), 500, tol=1e-6)
    centred = FakeDetection(W / 2 - 50, 100, W / 2 + 50, 300)
    check("box_offset of a centred box", avoidance.box_offset(centred), 0.0, tol=1e-6)
    at_right_edge = FakeDetection(W - 20, 100, W, 300)
    check("box_offset near the right edge", avoidance.box_offset(at_right_edge), 0.486, tol=0.01)
    all_left = FakeDetection(10, 100, 200, 300)
    check("left_fraction, box fully left", avoidance.box_left_fraction(all_left), 1.0, tol=1e-6)
    straddling = FakeDetection(W / 2 - 30, 100, W / 2 + 90, 300)
    check("left_fraction, straddling 1/4 left", avoidance.box_left_fraction(straddling), 0.25, tol=1e-6)
    near = FakeDetection(0, H - 60, 100, H)       # bottom edge at the image bottom
    far = FakeDetection(0, 0, 100, 30)            # bottom edge near the top
    check("near object weighs SIDE_WEIGHT_NEAR", avoidance.obstacle_weight(near),
          avoidance.SIDE_WEIGHT_NEAR, tol=1e-6)
    assert avoidance.obstacle_weight(near) > avoidance.obstacle_weight(far), \
        "a nearer object must weigh more than a farther one"

    print("crowding:")
    left_chair = FakeDetection(50, H - 200, 250, H - 20)
    right_chair_far = FakeDetection(W - 250, 60, W - 50, 160)
    L, R = avoidance.measure_crowding([left_chair, right_chair_far])
    check("near-left beats far-right", L > R, True)

    print("blocking:")
    # The person we are chasing: mid-frame, bottom edge well above the margin.
    person = FakeDetection(W / 2 - 60, 100, W / 2 + 60, H - 300, class_id=0)

    beyond = FakeDetection(W / 2 - 40, 80, W / 2 + 40, H - 350)   # farther than the person
    check("clear path (obstacle beyond target)",
          avoidance.find_blocking_obstacle(person, [beyond]), None)

    between_far = FakeDetection(W / 2 - 40, 80, W / 2 + 40, H - 280)  # between, but far
    check("clear path (between, still far)",
          avoidance.find_blocking_obstacle(person, [between_far]), None)

    off_side = FakeDetection(30, 200, 150, H - 40)                 # near, but off to the left
    check("clear path (near, off to a side)",
          avoidance.find_blocking_obstacle(person, [off_side]), None)

    in_the_way = FakeDetection(W / 2 - 50, 200, W / 2 + 30, H - 100)
    got = avoidance.find_blocking_obstacle(person, [in_the_way])
    assert got is not None, "a near, centred obstacle between us and the target must block"
    check("blocking obstacle's offset", got, avoidance.box_offset(in_the_way), tol=1e-6)

    nearer = FakeDetection(W / 2 - 20, 300, W / 2 + 60, H - 40)
    got = avoidance.find_blocking_obstacle(person, [in_the_way, nearer])
    check("nearest blocker wins", got, avoidance.box_offset(nearer), tol=1e-6)

    print("side choice:")
    d, _ = avoidance.choose_side((0.5, 3.0), 0.0, 0.0)
    check("crowded right -> detour left", d, 1.0)
    d, _ = avoidance.choose_side((3.0, 0.5), 0.0, 0.0)
    check("crowded left -> detour right", d, -1.0)
    d, _ = avoidance.choose_side((1.0, 1.0), +0.10, 0.0)
    check("even sides, obstacle right of centre -> left", d, 1.0)
    d, _ = avoidance.choose_side((1.0, 1.0), 0.0, +0.3)
    check("all even, target right -> detour right (shorter way back)", d, -1.0)

    print()
    if FAIL:
        print(f"{FAIL} test(s) failed, {PASS} passed.")
        raise SystemExit(1)
    print(f"All {PASS} tests passed. Take it to the robot.")


if __name__ == "__main__":
    try:
        main()
    except NotImplementedError:
        print()
        print(f"Hit a function you have not implemented yet "
              f"({PASS} test(s) passed before it). Keep going!")
        raise SystemExit(1)
