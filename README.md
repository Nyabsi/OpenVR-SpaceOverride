# OpenVR Space Calibrator (Special Edition)

This is a fork of [OpenVR Space Calibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator) I made for headsets like the Pico and Galaxy XR, where the other calibration solutions never really worked well for me. You'd calibrate, it'd look fine, and then after walking around a bit your hands and trackers would slowly slide out of place again.

So I took a different approach. You rigidly mount a Vive tracker to your headset, and after calibrating, the driver stops using the headset's own pose and instead builds it from the tracker. Now the headset and all your other lighthouse devices are running off the same tracking system, so there's nothing left to drift against. The SLAM tracking is still there underneath, you just get proper alignment on top of it.

## Requirements

- Lighthouse system (or other equivalent)
- Rigid Tracker (i.e., Vive Tracker 3.0 or equivalent)
- Headset with SLAM or other positional tracking system

## Calibration Guide

https://github.com/user-attachments/assets/de458fa9-3029-4aca-95d1-5753ce6602a9

1. Mount a rigid tracker to your headset.
2. Hit **Calibrate**. It goes through a few stages, and the on-screen text tells you what it wants at each one:
    - First it asks you to **move your head around** so it can spot which tracker is the one mounted to your head.
    - Then it calibrates — keep moving, walking in a circle while bobbing your head slightly, until it's done.
3. That's it. The profile saves on its own and the override stays running in the background.

Once it's calibrated, the headset is driven entirely by the tracker, so if you don't need the SLAM devices you can disable the headset's own tracking too.

## Showcase

<img width="1202" height="832" alt="image" src="https://github.com/user-attachments/assets/ad810fc5-caef-4ff7-9efd-4fa208076279" />

<img width="1202" height="832" alt="image" src="https://github.com/user-attachments/assets/34b7b18b-df07-478b-9f0a-2d75d71594c8" />

<img width="1202" height="832" alt="image" src="https://github.com/user-attachments/assets/ff913b65-8f64-4c43-b2d4-840e19b6165d" />

## Troubleshooting

### My view has shifted after leaving the headset unattended for a period of time

You need to re-center your headset tracking while rotating around in a circle til the view matches again, this is a known quirk as of right now.

This can cause SLAM devices to drift, and this is *by design* so, please re-calibrate if that happens, however note this should never happen during active usage.

## TODO

- Add checkbox to turn the headset into true lighthouse tracked device, this is currently omitted for UX reasons

## License

Commits up to and including `1cc0583` are MIT (see [`LICENSE.MIT`](LICENSE.MIT)). Everything after is GPLv3 (see [`LICENSE`](LICENSE)).
