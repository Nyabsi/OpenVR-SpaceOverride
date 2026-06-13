# OpenVR-SpaceOverride

OpenVR-SpaceOverride aligns SLAM-tracked headsets (Pico, Galaxy XR and similar) with lighthouse-tracked devices, without the drift that plagues traditional playspace calibration. Instead of computing a one-time offset between two tracking systems that slowly slide apart, it uses a Vive tracker rigidly mounted to the headset: after calibration, the driver stops using the headset's own pose and builds it from the tracker instead.

This puts the headset and all your other lighthouse devices on the same tracking system, so there's nothing left to drift against. The SLAM tracking is still there underneath, you just get proper alignment on top of it.

## Requirements

- Lighthouse system (or other equivalent)
- Rigid Tracker (i.e., Vive Tracker 3.0 or equivalent, **not** a Tundra Tracker, see warning above)
- Headset with SLAM or other positional tracking system

## Compatibility

| Streamer | Status | Notes |
| --- | --- | --- |
| PICO Connect | ✅ Works | |
| Virtual Desktop | ✅ Works | |
| ALVR | ✅ Works | |
| Steam Link | ⚠️ Sub-optimal | Steam Link appears to do additional reprojection / pose prediction on-device, and some functionality doesn't work because it uses stage tracking rather than local space tracking (which is needed for yaw correction). |
| Display Port powered SLAM devices | ✅ Works | Such as: Pimax, PSVR2, HP Reverb G2, etc. |

## Calibration Guide

> [!WARNING]
> If you're using Virtual Desktop, please ensure **Stage Tracking** is disabled in the settings, this is required for the software to function in the expected way.

1. Mount a rigid tracker to your headset.
2. Hit **Calibrate**. It goes through a few stages, and the on-screen text tells you what it wants at each one:
    - First it asks you to **move your head around** so it can spot which tracker is the one mounted to your head.
    - Then it calibrates. *Look left, Look center, Look right, Look center, Look up, Look center.*
3. That's it. The profile saves on its own and the override stays running in the background.

> [!TIP]
> **Patience is key.** If the result feels odd or misaligned after calibrating, switch to a **slower calibration speed** and re-try. Slower calibration gives the solver more data to work with and almost always produces a better result.

Once it's calibrated, the headset is driven entirely by the tracker, so if you don't need the SLAM devices you can disable the headset's own tracking too. This also means the override works fine with your headset set to 3DoF mode or with its positional tracking disabled.

## Native Override Mode

> [!CAUTION]
> Please **DO NOT** use this, unless you understand how to do a manual TrackingOverride, this is meant for **advanced** users and not for your average person, you should not have *any* reason to use this, unless you *know* what you are doing.

Native Override feeds the raw tracker data, with the corrected offset applied, directly to the headset. This makes the headset behave as a truly native lighthouse device. The trade-off is yaw misalignment: the tracker's projection is different from your headset's, which is also why local space tracking is required.

To set it up:

1. Enable the **Native Override** option.
2. Run the calibration.
3. Stand up straight and rotate your body in a circle while recentering your **headset's** space (not SteamVR's) until you find the spot where the view lines up.
4. Once aligned, you can optionally turn off the headset's own tracking entirely, and the pose never changes from that point on.

> [!NOTE]
> After a headset restart, you may need to redo the yaw alignment.

## Troubleshooting

### My calibration feels odd / slightly off

Re-run the calibration with a **slower calibration speed**. A rushed calibration is the most common cause of a bad offset, so take your time, move smoothly, and let it finish.

### My view has shifted after leaving the headset unattended for a period of time

You need to re-center your headset tracking while rotating around in a circle until the view matches again, this is a known quirk as of right now.

### Steam Link feels off

This is expected, see [Compatibility](#compatibility). Steam Link seems to apply its own reprojection / pose prediction on-device on top of the overridden pose, and it's stage tracked rather than local space tracked, which breaks yaw correction. For the best experience, use PICO Connect, Virtual Desktop or ALVR.

### The view seems to have odd roll effect

Turn on "Disable Angular Velocity" in the Settings.

## FAQ

### How is this different from TrackingOverride?

This is **not** TrackingOverride. TrackingOverride simply substitutes one device's pose for another's, which leaves you to deal with the offset between the tracker and the headset yourself. SpaceOverride instead **automatically calculates the proper offset** between the mounted tracker and the headset during calibration, then continuously reconstructs the headset pose from the tracker using that offset. The result is an aligned, drift-free pose rather than a raw pose swap.

### Does this conflict with OpenVR Space Calibrator?

No. SpaceOverride does not conflict with Space Calibrator, you can have both installed. They solve the alignment problem differently, but having Space Calibrator present won't break the override.

### Why are Tundra Trackers unusable?

The mounted tracker drives your *entire headset pose*, so the tracker's quality directly becomes your view's quality. Tundra Trackers are known for jittery tracking, and jitter that's merely annoying on a hip or foot becomes nauseating when it's applied to your head. Use a Vive Tracker 3.0 or an equivalently stable device.

### Can wireless latency affect the pose?

Yes. The headset's display pipeline and the lighthouse tracker run on different clocks, so wireless streaming latency (and an unstable connection in general) can introduce a delay between your real head movement and the tracker-driven pose, which shows up as lag or swimming in your view. A solid Wi-Fi setup (or wired connection where possible) keeps this negligible.

## Acknowledgements

This project is a fork of [OpenVR Space Calibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator). Huge thanks to [pushrax](https://github.com/pushrax) for their work, which this project builds on.

## License

Commits up to and including `1cc0583` are MIT (see [`LICENSE.MIT`](LICENSE.MIT)). Everything after is GPLv3 (see [`LICENSE`](LICENSE)).
