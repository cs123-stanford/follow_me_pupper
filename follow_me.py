"""Follow Me Pupper: the tracking state machine.

The Pupper picks a target class out of /detections, walks toward it, and steps
around anything in the way. Motion decisions live here; how the robot *sees*
(camera, YOLO, the browser viewer) lives in viser_camera.py, and the geometry
of deciding what is "in the way" lives in avoidance.py.

Run it (see README.md for the full three-terminal setup):

    python3 follow_me.py                            # waits for /tracking_control
    python3 follow_me.py --ros-args -p target:=person   # start tracking at once
"""

from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
import numpy as np
import os

import avoidance
from avoidance import (
    OBSTACLE_MIN_SCORE,
    box_offset,
    detection_class_id,
)

# --- Control constants ------------------------------------------------------
# ========== YOUR CODE HERE: constants: control tuning ==========
TIMEOUT = None            # s without a detection before we go searching
SEARCH_YAW_VEL = None     # rad/s rotation speed while searching
TRACK_FORWARD_VEL = None  # m/s forward speed while tracking
KP = None                 # proportional gain for centering the target
# ===============================================================

# --- Obstacle avoidance -----------------------------------------------------
# Flip to False and the robot walks straight at the target, ignoring anything
# in between. Useful while you are still working on the core tracking.
ENABLE_OBSTACLE_AVOIDANCE = True

# The detour: turn away, walk past, turn back. The two turns are the same
# speed for the same time in opposite directions, so the robot comes out of it
# pointing the way it went in -- displaced sideways by the walk, which is the
# whole point.
# ========== YOUR CODE HERE: constants: detour tuning ==========
AVOID_YAW_VEL = None      # rad/s during the two turns
AVOID_TURN_TIME = None    # s turning away, and again turning back
AVOID_FORWARD_VEL = None  # m/s while passing the obstacle
AVOID_PASS_TIME = None    # s walking forward alongside the obstacle
AVOID_COOLDOWN = None     # s before another detour may start
# ==============================================================

# --- Optional: color re-identification --------------------------------------
# With several people in frame, plain "track the most centred person" happily
# hops from one person to another. Each detection carries the mean color of
# its segmentation mask (in detection.id, as "r,g,b" -- mostly the color of
# the clothes). Memorize the target's color when tracking starts and prefer
# the detection that still matches it, and the robot follows the *same*
# person -- until two people wear the same shirt.
ENABLE_COLOR_REID = False
REID_MAX_DIST = 90.0    # max RGB distance that still counts as "same object"
REID_BLEND = 0.1        # per-frame blend of the memory toward the current color


class State(Enum):
    IDLE = 0          # Stay in place, no tracking
    SEARCH = 1        # Rotate to search for target
    TRACK = 2         # Follow the target
    AVOID_TURN = 3    # Turn away from an object in the way
    AVOID_PASS = 4    # Walk forward past it
    AVOID_RETURN = 5  # Turn back onto the original heading

AVOID_STATES = (State.AVOID_TURN, State.AVOID_PASS, State.AVOID_RETURN)


def load_class_names():
    """COCO class names, in the id order the detectors publish."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'coco.txt')
    with open(path) as f:
        return [line.strip() for line in f if line.strip()]


def detection_color(detection):
    """Mean mask color of a detection as an RGB array, or None.

    viser_camera.py stamps it into detection.id as "r,g,b" when segmentation
    masks are on; an empty id means no color was available for this box.
    """
    if not detection.id:
        return None
    try:
        parts = [float(p) for p in detection.id.split(',')]
    except ValueError:
        return None
    return np.array(parts) if len(parts) == 3 else None


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Subscribe to tracking control to enable/disable tracking
        self.tracking_control_subscription = self.create_subscription(
            String,
            '/tracking_control',
            self.tracking_control_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        # Start in IDLE mode (no tracking until commanded)
        self.state = State.IDLE
        self.tracking_enabled = False

        # /detections carries every class, so pick the target out of it here.
        self.class_names = load_class_names()
        self.class_name_to_id = {name: i for i, name in enumerate(self.class_names)}
        self.target_class_id = 0  # person
        self.target_object = 'person'

        self.target_pos = 0.0
        self.last_detection_time = self.get_clock().now()

        # Where the object blocking our way is, or None when the path is clear.
        self.obstacle_pos = None
        # How much stuff the last frame had in each half of it: (left, right).
        self.crowding = (0.0, 0.0)
        self.avoid_dir = 1.0
        self.phase_start = self.get_clock().now()
        self.avoid_done_time = None

        # Optional color re-id: the remembered mean color of *our* target.
        self.target_color = None

        # Tracking normally starts when something publishes to /tracking_control
        # -- KarelPupper.begin_tracking(), or the robot-foundation-model lab's
        # commander later in the quarter. Pass a target to skip that and start
        # straight away, so this lab stands on its own with nothing else
        # driving it:
        #
        #     python3 follow_me.py --ros-args -p target:=person
        self.declare_parameter('target', '')
        initial_target = self.get_parameter('target').value.strip()

        self.get_logger().info('State Machine Node initialized in IDLE state.')
        if initial_target:
            self.start_tracking(initial_target)
        else:
            self.get_logger().info(
                'Use begin_tracking(object), or -p target:=<class>, to enable tracking.'
            )

    def start_tracking(self, obj_name):
        """Enter tracking mode for a COCO class, from the parameter or the topic."""
        self.tracking_enabled = True
        self.target_color = None  # a new chase memorizes a new color
        if obj_name in self.class_name_to_id:
            self.target_object = obj_name
            self.target_class_id = self.class_name_to_id[obj_name]
        else:
            # Detectors that pre-filter their output still publish something
            # sensible, so keep going rather than refusing to track at all.
            self.get_logger().warning(
                f'⚠️  Unknown object class "{obj_name}" - still chasing {self.target_object}'
            )
        self.get_logger().info(
            f'✅ Tracking enabled for: {self.target_object} '
            f'(class_id={self.target_class_id})'
        )
        self.get_logger().info(f'   State transition: {self.state.name} → SEARCH')
        self.state = State.SEARCH  # Start searching for target

    def stop_tracking(self):
        """Leave tracking mode and stand still."""
        self.tracking_enabled = False
        self.obstacle_pos = None
        self.target_color = None
        self.get_logger().info('⏸️  Tracking disabled - returning to IDLE')
        self.get_logger().info(f'   State transition: {self.state.name} → IDLE')
        self.state = State.IDLE
        # Stop all movement
        cmd = Twist()
        self.command_publisher.publish(cmd)

    def tracking_control_callback(self, msg):
        """Handle tracking control commands."""
        command = msg.data
        self.get_logger().info(f'📥 Received tracking control: "{command}"')

        if command.startswith("start:"):
            self.start_tracking(command.split(":", 1)[1])
        elif command == "stop":
            self.stop_tracking()

    def pick_target(self, targets):
        """Which of the target-class detections is *the* target this frame.

        The base rule is simple: the most centred one -- it is the one we were
        just steering toward. The optional color re-id refines that so the
        robot stays on the same individual (see ENABLE_COLOR_REID above).
        """
        # ========== YOUR CODE HERE: tracking: pick the target ==========
        # TODO: sort by |box_offset| and, when ENABLE_COLOR_REID is
        # off, return the most centred one. (When the basics work,
        # ask yourself: when is "most centred" the wrong choice? That
        # question is what the optional color re-id part answers.)
        raise NotImplementedError
        # ===============================================================

        # --- Optional: color re-id -----------------------------------------
        # ========== OPTIONAL — YOUR CODE HERE: re-id: match the remembered color ==========
        # TODO (optional): among the candidates, prefer the one whose
        # mean mask color (detection_color) is nearest the remembered
        # self.target_color -- but only within REID_MAX_DIST, so a
        # frame with the wrong person alone in it does not steal the
        # lock. Memorize the color on first sight, and blend the
        # memory toward the current match (REID_BLEND) so slow light
        # changes do not shake it off.
        return by_center[0]
        # ==================================================================================

    def detection_callback(self, msg):
        # ========== YOUR CODE HERE: tracking: process a detection frame ==========
        # TODO: split msg.detections into targets (our class) and
        # others (everything else worth OBSTACLE_MIN_SCORE). Use
        # detection_class_id; treat an unreadable id as a target, so a
        # detector that pre-filters to one class still works.
        #
        # Then: measure the crowding of the frame (avoidance.measure_crowding)
        # -- even when the target is not in it -- and, when there IS a
        # target: pick it (self.pick_target), store its box_offset in
        # self.target_pos, stamp self.last_detection_time, and ask
        # avoidance.find_blocking_obstacle whether something is in the
        # way (respect ENABLE_OBSTACLE_AVOIDANCE).
        pass
        # =========================================================================

        # Debug logging
        if self.tracking_enabled:
            self.get_logger().info(
                f'🎯 Target at position: {self.target_pos:.3f} | '
                f'Center X: {closest_det.bbox.center.position.x:.1f}'
            )

    def start_avoiding(self, now):
        """Commit to a side and begin the detour."""
        # ========== YOUR CODE HERE: detour: commit to a side ==========
        # TODO: ask avoidance.choose_side which way to go (it wants the
        # crowding tally, the obstacle offset, and the target offset),
        # store the direction in self.avoid_dir, enter AVOID_TURN, and
        # start the phase clock (self.phase_start).
        raise NotImplementedError
        # ==============================================================
        side = 'left' if self.avoid_dir > 0 else 'right'
        self.get_logger().info(
            f'↪️  Obstacle ahead - going around it on the {side} ({why})'
        )

    def advance_avoidance(self, now):
        """Step the timed turn / pass / turn-back sequence along."""
        # ========== YOUR CODE HERE: detour: phase clock ==========
        # TODO: how long has the current phase been running? Move
        # AVOID_TURN -> AVOID_PASS -> AVOID_RETURN -> TRACK as each
        # phase's time (AVOID_TURN_TIME / AVOID_PASS_TIME /
        # AVOID_TURN_TIME again) elapses, restarting the phase clock at
        # each hop. On re-entering TRACK, clear self.obstacle_pos and
        # note the time in self.avoid_done_time -- the cooldown reads it.
        raise NotImplementedError
        # =========================================================

    def ready_to_avoid(self, now):
        """Should the TRACK state hand over to a detour right now?"""
        # ========== YOUR CODE HERE: detour: when to start one ==========
        # TODO: only when avoidance is enabled, something is actually
        # blocking (self.obstacle_pos), and the last detour ended more
        # than AVOID_COOLDOWN seconds ago. The cooldown matters: right
        # after a detour the obstacle is often still half in frame, and
        # without it the robot detours forever in a circle.
        raise NotImplementedError
        # ===============================================================

    def timer_callback(self):
        # State machine logic
        if not self.tracking_enabled:
            # Not tracking - stay idle and DON'T publish
            # This allows Karel commands to control the robot
            self.state = State.IDLE
            return

        now = self.get_clock().now()

        # ========== YOUR CODE HERE: fsm: transitions ==========
        # TODO: three questions decide the state.
        #   - Mid-detour (state in AVOID_STATES)? Let it run to
        #     completion on its own clock (advance_avoidance).
        #     Detections keep arriving and updating target_pos, so the
        #     chase picks straight back up afterwards.
        #   - Otherwise: how stale is the last detection? Older than
        #     TIMEOUT means the target is lost -> SEARCH.
        #   - Target in view -> TRACK, unless ready_to_avoid says a
        #     detour should start instead (start_avoiding).
        pass
        # ======================================================

        # Execute state behavior
        yaw_command = 0.0
        forward_vel_command = 0.0

        # ========== YOUR CODE HERE: fsm: state behaviors ==========
        # TODO: set yaw_command and forward_vel_command per state.
        #   IDLE          stand still.
        #   SEARCH        rotate in place -- think about *which way*
        #                 (where was the target last seen?).
        #   TRACK         proportional control: steer against
        #                 target_pos with gain KP, walk forward.
        #   AVOID_TURN    turn off the line, direction avoid_dir.
        #   AVOID_PASS    walk straight past the obstacle.
        #   AVOID_RETURN  undo the first turn.
        pass
        # ==========================================================

        cmd = Twist()
        cmd.angular.z = yaw_command
        cmd.linear.x = forward_vel_command
        self.command_publisher.publish(cmd)

        # Debug logging (log state and commands when tracking)
        if self.tracking_enabled:
            self.get_logger().info(f'🤖 State: {self.state.name} | Yaw: {yaw_command:.2f} | Fwd: {forward_vel_command:.2f} | Target: {self.target_pos:.3f}')

def main():
    rclpy.init()
    state_machine_node = StateMachineNode()

    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        zero_cmd = Twist()
        state_machine_node.command_publisher.publish(zero_cmd)

        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
