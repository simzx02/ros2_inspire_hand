import time
import sys
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from inspire_sdkpy import inspire_hand_defaut, inspire_dds

if __name__ == '__main__':
    # Init communication
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    # Publisher & subscriber for right hand
    pubr = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
    pubr.Init()
    subr = ChannelSubscriber("rt/inspire_hand/state/r", inspire_dds.inspire_hand_state)
    subr.Init()

    # Prepare command
    cmd = inspire_hand_defaut.get_inspire_hand_ctrl()

    # Force limits (grams)
    force_limits = [300] * 6
    if hasattr(cmd, "force_set"):
        cmd.force_set = force_limits
    else:
        print("Warning: cmd.force_set not found in inspire_hand_ctrl")

    # Step 1: Open all fingers fully
    cmd.angle_set = [850] * 6
    cmd.mode = 0b0001  # angle mode for opening
    pubr.Write(cmd)
    print("Opening fingers...")
    time.sleep(2.0)

    # Step 2: Progressive per-finger closing
    current_angles = [850] * 6
    min_angle = 250
    step_size = 10

    reached = [False] * 6
    final_forces = [0.0] * 6
    final_angles = [None] * 6

    print("Closing fingers...")

    try:
        while not all(reached) and any(a >= min_angle for a in current_angles):
            prev_angles = current_angles.copy()

            cmd.angle_set = current_angles
            cmd.mode = 0b0101  # angle + force control
            pubr.Write(cmd)

            time.sleep(0.1)

            state_r = subr.Read(0.05)
            if state_r is not None:
                forces = list(state_r.force_act)
                final_forces = forces
                actual_angles = getattr(state_r, "angle_act", current_angles)

                for i in range(6):
                    if not reached[i]:
                        if forces[i] >= force_limits[i]:
                            reached[i] = True
                            final_angles[i] = actual_angles[i]
                        else:
                            if current_angles[i] - step_size >= min_angle:
                                current_angles[i] -= step_size

            if current_angles == prev_angles:
                print("No angle changes detected — motion stopped.")
                break

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected — opening all fingers.")
        cmd.angle_set = [850] * 6
        cmd.mode = 0b0001
        pubr.Write(cmd)
        time.sleep(1.0)
        sys.exit(0)

    # Step 3: Summary
    print("\n=== Per-Finger Progressive Grip Summary (Right Hand) ===")
    for i in range(6):
        status = "Reached" if reached[i] else "Not reached"
        stop_angle = final_angles[i] if final_angles[i] is not None else current_angles[i]
        print(f"Finger {i+1}: {final_forces[i]:.1f} g "
              f"(limit {force_limits[i]} g) -> {status}, stop angle: {stop_angle}")

    # Step 4: Wait until user interrupts, then open hand
    try:
        print("\nPress Ctrl+C to open all fingers and exit.")
        while True:
            time.sleep(0.1)  # idle loop
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected — opening all fingers.")
        cmd.angle_set = [850] * 6
        cmd.mode = 0b0001  # angle mode for opening
        pubr.Write(cmd)
        time.sleep(1.0)
        sys.exit(0)