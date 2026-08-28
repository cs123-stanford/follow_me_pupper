#!/bin/bash
# Stop everything from this lab and release the Hailo accelerator.
# Only one process may hold the Hailo at a time, so a stale process is the
# usual cause of "device in use" on the next run.

echo "Stopping lab 7 processes..."
for proc in test_tracking.py follow_me.py viser_camera.py follow_me.launch.py; do
    if pkill -f "$proc" 2>/dev/null; then
        echo "  killed $proc"
    else
        echo "  $proc not running"
    fi
done

sleep 2

# Anything that ignored SIGTERM still holds the accelerator.
REMAINING=$(pgrep -f "viser_camera.py|follow_me.py" | wc -l)
if [ "$REMAINING" -gt 0 ]; then
    echo "  $REMAINING process(es) survived SIGTERM, forcing..."
    pkill -9 -f "viser_camera.py|follow_me.py" 2>/dev/null
    sleep 1
fi

echo "Cleanup complete."
