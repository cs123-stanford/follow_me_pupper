# Follow Me Pupper

The Pupper finds an object with its camera, walks toward it, and steps around
anything in the way. Everything runs on the robot; you watch it in a browser.

This lab is self-contained: no API keys, no LLM, no voice. The `KarelPupper`
tracking API (`begin_tracking()` / `end_tracking()`) is what the robot
foundation model lab will drive later in the quarter.

## Setup (once)

```bash
./scripts/download_model.sh      # fetches yolov8n_seg.hef (~11 MB)
```

## Running

Three terminals on the Pupper. Keeping them separate means you can restart your
state machine without restarting the robot.

```bash
# 1) robot + camera + viewer + detection
ros2 launch follow_me.launch.py

# 2) your tracking state machine
python3 follow_me.py

# 3) pick something to track
python3 test_tracking.py
```

Then open the viewer at `http://<pupper-ip>:8080`. Over SSH, forward the port
from your laptop first and browse to `localhost:8080`:

```bash
ssh -N -L 8080:localhost:8080 pi@<pupper-ip>
```

`scripts/run_tracking.sh` does all three at once for a quick demo.
`scripts/cleanup.sh` stops everything and frees the Hailo accelerator.

To skip terminal 3 and start tracking immediately:

```bash
python3 follow_me.py --ros-args -p target:=person     # any COCO class
```

The avoidance logic can be tested with no robot at all:

```bash
python3 test_avoidance.py       # runs canned scenes through avoidance.py
```

## Viewer controls

Two settings in the browser GUI matter when the link is poor:

- **Lock aspect ratio** (Camera, on by default) — viser stretches the background
  image across the whole window, which at the default 220° FOV squashes the
  picture sideways by ~45%. On, the frame keeps its true proportions and pads
  with bars instead.
- **Drop frames when behind** (Camera, on by default) — on weak wifi, sending
  every frame just grows a queue the viewer has to chew through, so the picture
  drifts further behind real time the longer you watch. On, a client that has not
  kept up simply misses frames. You lose smoothness, never currency. The Status
  folder counts what was dropped.

## Files

| File | What it does |
|---|---|
| `follow_me.py` | **The lab.** State machine: IDLE → SEARCH → TRACK, plus the detour states. |
| `avoidance.py` | **Also the lab.** The obstacle-avoidance decisions, as pure functions of detections. |
| `test_avoidance.py` | Desk-side test harness: runs canned scenes through your `avoidance.py`. |
| `follow_me.launch.py` | Brings up motor control, the camera, and the viser viewer. |
| `viser_camera.py` | Camera → browser. Runs YOLOv8n-seg on the Hailo and publishes `/detections`. |
| `yolo_seg.py` | Hailo YOLOv8n-seg wrapper: inference, NMS, mask decoding, drawing. |
| `fisheye_converter.py` | Fisheye → equirectangular. Detection runs on the undistorted image. |
| `karel.py` | `KarelPupper` robot API: `move_forward()`, `bark()`, `begin_tracking()`, … |
| `test_tracking.py` | Interactive prompt for choosing what to track. |
| `coco.txt` | The 80 COCO class names, in the id order detections use. |
| `camera_params.yaml` | Double-sphere lens model for the fisheye undistortion. |
| `sounds/` | Bark, wiggle, bob, and dance audio. |

## How the pieces talk

```
camera_ros ──/camera/image_raw/compressed──> viser_camera.py ──/detections──> follow_me.py
                                                    ^                            |
                                                    |                         /cmd_vel
                              karel.py ──/tracking_control──┘                     |
                          (test_tracking.py)                                      v
                                                                          neural_controller
```

`follow_me.py` starts in IDLE and only begins tracking when something publishes
to `/tracking_control` — either `KarelPupper.begin_tracking()` or the `target`
parameter above.

## Notes

- Only one process can hold the Hailo accelerator at a time. If you see a
  device-in-use error, run `./scripts/cleanup.sh`.
- Detections are published in a fixed 700×572 equirectangular frame, so the
  pixel thresholds in `avoidance.py` mean the same thing at any camera
  resolution.
- Each detection also carries the mean color of its segmentation mask (in the
  message's `id` field, as `"r,g,b"`) — the raw material for the optional
  color re-identification part of the lab.
