# OpenVR-SpaceOverride

OpenVR-SpaceOverride aligns SLAM-tracked headsets (Pico, Galaxy XR and similar) with lighthouse-tracked devices, without the drift that plagues traditional playspace calibration. Instead of computing a one-time offset between two tracking systems that slowly slide apart, it uses a Vive tracker rigidly mounted to the headset: after calibration, the driver stops using the headset's own pose and builds it from the tracker instead.

This puts the headset and all your other lighthouse devices on the same tracking system, so there's nothing left to drift against. The SLAM tracking is still there underneath, you just get proper alignment on top of it.

> [!NOTE]
> If you find bugs or issues, let me know over e-mail at `nyabsi@sovellus.cc` I will be responding to you when I have time. This software will receive updates on irregular basis but each update is guaranteed to improve the software in a way or another, thanks for using it even on it's current state.

## Requirements

- Lighthouse system (or other equivalent)
- Rigid Tracker (i.e., Vive Tracker 3.0 or equivalent)
- Headset with SLAM or other positional tracking system

## Compatibility

| Streamer | Status | Notes |
| --- | --- | --- |
| PICO Connect | ⚠️ Sub-optimal |  v9 has issues, v8 works OK. |
| Virtual Desktop | ✅ Works | |
| ALVR | ✅ Works | |
| Meta Quest Link | ⚠️ Unconfirmed | Unconfirmed, let me know, email: nyabsi@sovellus.cc |
| Air Link | ⚠️ Unconfirmed | Unconfirmed, let me know, email: nyabsi@sovellus.cc |
| Steam Link | ⚠️ Sub-optimal |  v9 has issues, v8 works OK. |
| Display Port powered SLAM devices | ✅ Works | Such as: Pimax, PSVR2, HP Reverb G2, etc. |
| VIVE Hub | ✅ Works | VIVE Focus Vision, via e-mail |

## Calibration Guide

1. Mount a rigid tracker to your headset.
2. Hit **Calibrate**. It goes through a few stages, and the on-screen text tells you what it wants at each one:
    - First it asks you to **move your head around** so it can spot which tracker is the one mounted to your head.
    - Then it calibrates. *Look left, Look center, Look right, Look center, Look up, Look center.*
3. That's it. The profile saves on its own and the override stays running in the background.

> [!TIP]
> **Patience is key.** If the result feels odd or misaligned after calibrating, switch to a **slower calibration speed** and re-try. Slower calibration gives the solver more data to work with and almost always produces a better result.

Once it's calibrated, the headset is driven entirely by the tracker, so if you don't need the SLAM devices you can disable the headset's own tracking too. This also means the override works fine with your headset set to 3DoF mode or with its positional tracking disabled.

## Troubleshooting

### My controllers jump, then settle back

Expected behavior, not a bug. 

Your controllers use the headset's inside-out tracking, which drifts relative to your lighthouse space, the driver corrects that drift and shifts back into place.

### My full body tracking seems weird in VRChat!

This is expected, there is issues with the VRChat IK system that is not obvious from first glance.

Usually Space Calibrator is not precise enough for these issues to show up, but with SpaceOverride you will see these issues.

Each Lighthouse HMD suffers from the same issues and this is unfortunately the reality we live in.

I am unable to solve these issues as they are not caused by SpaceOverride but I am working on potential workarounds.

## FAQ

### How is this different from TrackingOverride?

This is **not** TrackingOverride. TrackingOverride simply substitutes one device's pose for another's, which leaves you to deal with the offset between the tracker and the headset yourself. SpaceOverride instead **automatically calculates the proper offset** between the mounted tracker and the headset during calibration, then continuously reconstructs the headset pose from the tracker using that offset. The result is an aligned, drift-free pose rather than a raw pose swap.

### Does this conflict with OpenVR Space Calibrator?

No. SpaceOverride does not conflict with Space Calibrator, you can have both installed. They solve the alignment problem differently, but having Space Calibrator present won't break the override.

### Can wireless latency affect the pose?

Yes. The headset's display pipeline and the lighthouse tracker run on different clocks, so wireless streaming latency (and an unstable connection in general) can introduce a delay between your real head movement and the tracker-driven pose, which shows up as lag or swimming in your view. A solid Wi-Fi setup (or wired connection where possible) keeps this negligible.

### Does it drift?

No, all drift is induced by poor lighthouse performance, please ensure your lighthouse setup does not have interference.

You can use guide such as: [Link](https://www.notion.so/yeove/SteamVR-Hardware-Troubleshooting-Megathread-Setup-Guide-16fc956d336a8037b738d1b0b1ded2f0#1c0c956d336a8035b76dd1b87527d180)

## Acknowledgements

This project uses/used substancial parts of [OpenVR Space Calibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator). Huge thanks to [pushrax](https://github.com/pushrax) for their work, which this project builds on.

## License

Commits up to and including `1cc0583` are MIT (see [`LICENSE.MIT`](LICENSE.MIT)). Everything after is AGPLv3 (see [`LICENSE`](LICENSE)).
