# Hardware testing

What has run on real arms and what has not. The first two tables are from commit `6142f53`; the
section after them is from `4e2844f`, which changed how `stop()`, `attach()` and `moveEe` end. Every
powered test ran at a hover pose, the tool point 5 to 6 cm above its rest height, and ended with the
arm at rest and torque off. Comparisons are against the Python SDKs on the same arm in the same
session.

## Tested

| Test | Metal Arm (two arms, Node) | Maker Arm (one arm, old motor firmware, Node) |
|---|---|---|
| Read-only scan, frames against Python | identical on both arms (28 frames) | identical (14 frames) |
| Controlling process killed at hover, then `attach()`, return to rest, release | both arms: joints hold, the gripper's motor watchdog releases it, rest within 1.0 deg | joints hold (drift 0.27 deg), rest within 0.26 deg; no watchdog on this firmware |
| 5 deg sine on pan and wrist joints, RMS error TypeScript / Python | 0.94/0.95, 1.84/1.83, 0.77/0.74, 0.80/0.77 deg (left arm; right arm alike) | 0.94/1.02, 1.17/1.26, 1.06/1.16, 1.04/1.14 deg |
| 1.5 deg sine on shoulder_lift and elbow_flex, RMS error TypeScript / Python | 0.81/0.81, 0.85/0.84 deg | 0.29/0.31, 0.27/0.30 deg |
| Control loop rate, TypeScript / Python | 99.1 to 99.5 / 100.0 Hz (target 100) | 186 / 150 Hz (target 200) |
| `moveEe`, eight legs | 1 cm legs: commanded tool within 0.9 mm of target; measured within 5.9 mm (left) and 9.1 mm (right), Python 5.8 and 9.7 mm | 3 cm legs: measured within 4.4 mm |
| Gripper open and close | 0.91 to 0.99 open, 0.02 closed | 0.98 open, 0.03 closed |
| Two arms from one `Rig`, 5 deg sine on both | 100.0 Hz each, tracking as each arm alone | one arm only |
| `Teleop`, a scripted leader arm and a follower arm | follower within 1.5 deg RMS, lag 66 to 176 ms by joint, holds when teleop stops | one arm only |

From Chromium over WebUSB (Metal Arm; the adapter granted by policy, see below):

| Test | Result |
|---|---|
| One arm in a page: connect, enable, hover, sine, rest | both arms; 98.8 and 99.4 Hz; tracking as from Node |
| One arm with its control loop in a Web Worker | both arms; 99.5 and 99.8 Hz |
| Two arms from one `Rig` in one page / in one Web Worker | 97.9 and 97.8 Hz / 98.6 and 98.6 Hz |
| `Camera` with three identical USB cameras | each 30 Hz alone; 30, 15 and 15 Hz together (two share a USB 2 bus) |

## At `4e2844f`: stop, takeover and motion ownership

One Metal arm (the left one), Node 24 unless marked, against metal-arm-sdk `36c0bb4`. The other
Metal arm and the Maker arm have not run at this commit.

| Test | Result |
|---|---|
| Read-only scan, frames against Python | identical (28 frames), on Node 24 and Node 22 |
| `stop()` after one interrupt at hover | returned to rest, the release recheck passed on the real bus, torque off on all seven |
| A second interrupt 0.2 s into the return (`stop({ signal })` aborted 8.8 deg above rest) | `stop()` resolved false; the arm held: 1.3 deg past the hold pose after 50 ms, settled 0.8 to 1.1 deg from it within 0.35 s, then no movement; every frame at the profile's gains, loop 100 Hz; a later `stop()` released at rest |
| Process killed at hover, `attach()` with the default ramp | joints hold, gripper released by its watchdog; takeover dip 3.5 deg on elbow_flex for 0.56 s; rest within 0.8 deg |
| The same with `attachGravity: "measured"` | gravity level 1.0 at takeover; dip 0.6 deg, no dwell; rest within 0.1 deg |
| Ten enable, hover, `stop()` cycles, each in its own process | every cycle released at rest; resting pose moved at most 0.15 deg (Metal has no rest window) |
| 5 deg sine on the wrist joints, RMS TypeScript / Python | 1.83/1.84, 0.75/0.79, 0.77/0.80 deg |
| 1.5 deg sine on shoulder_lift and elbow_flex | 0.82/0.82, 0.90/0.89 deg |
| Control loop inside the sine window | 100.0 Hz, longest period 16 ms |
| `moveEe`, 1 cm lateral legs | commanded tool within 0.9 mm; measured within 3.9 mm, Python 5.4 mm |
| `moveEe` cancelled 0.3 s after its first row | resolved false; the arm held where it was (joints steady within 0.01 deg); nothing further toward the target was sent |
| `moveEe` superseded by a second one 0.3 s in | the first resolved false, the second true and arrived (commanded within 0.3 mm) |
| `moveEe` with a 1 ms planning timeout | resolved false, nothing sent |
| Chromium over WebUSB, in a page / in a Web Worker | 100 Hz both (p99 10.5 / 10.2 ms); tracking as from Node; `Camera` as above |
| `examples/node/check-sine.ts` on Node 22, and with one Ctrl-C mid-sine | ran and released; the Ctrl-C stopped the motion, returned to rest and released |

## Limits corrected during the run

Five runs were recorded as failures because the bench script's own limit was wrong, not the arm's
behaviour. The scripts and limits are not part of this repository. Each was re-run with the
corrected criterion and passed; none is an open failure.

| Run | First limit | Measured | Corrected criterion | Re-run |
|---|---|---|---|---|
| Kill and `attach()`, left Metal arm | pose within 3 deg of hover, read 0.3 s after `attach()` | 3.4 deg, read during the takeover dip | the settled pose, read 1.5 s after `attach()`; the dip is reported | pass: dip 3.4 deg, settled 1.6 deg from hover |
| `moveEe`, both Metal arms | measured tool within 5 mm of each target | 6.0 mm (left), 8.0 mm (right); the commanded pose was within 1 mm | commanded tool within 1.5 mm; measured within 12 mm and compared with Python on the same arm | pass: commanded 0.84 and 0.91 mm; measured 5.9 and 9.1 mm, Python 5.8 and 9.7 mm |
| `Teleop`, leader arm and follower arm | follower lag under 150 ms on every joint | wrist_flex 181 ms; that joint lags its own command by about 175 ms in Python too | lag under 250 ms | pass: 66 to 176 ms by joint |
| Second interrupt during the return | every joint within 1 deg of the pose the abort left it at | shoulder_lift 1.09, elbow_flex 1.14 deg (stopped while moving down at 20 deg/s) | within 3 deg and steady within 0.3 deg over the last 2 s (a limp or returning joint goes 9 deg) | pass: settled 0.8 to 1.1 deg, then 0.00 deg of movement |
| Cancelled `moveEe` | commanded tool point ends between start and target | -60 % of the leg: a cancel holds the measured pose, 6 mm below the commanded line under gravity | the last row sent before the cancel was partway along the leg | pass: 21 % along |

## Not tested

- Any motion above hover height, and any contact with an object.
- At `4e2844f`: the Maker arm, the second Metal arm, two arms together, `setMotorId`, and an
  `enable()` refused with `HeldByAnotherProgram`.
- A policy-driven episode (the Dream client with these arms).
- Two Maker arms; the SO-101; a Star leader; an SLCAN adapter.
- Hand-guided teleoperation and float mode.
- Fault paths that need a person: unplugging an adapter, cutting motor power, a blocked joint.
- macOS and Windows with a real adapter (CI covers those systems with simulated devices only).
- The Linux kernel `gs_usb` driver bound to the adapter (neither test machine loads it).

## Seen on the arms

- `attach()` (the default `attachGravity: "ramp"`, as in Python) restarts gravity compensation from
  zero and eases it in over 0.5 s, so a loaded joint dips for a moment at takeover (about 3.5 deg on
  the Metal elbow). The Python SDK does the same. With `"measured"` the dip was 0.6 deg in its one
  run; it needs every gravity-chain joint to report torque state, so old Maker firmware ramps.
- A `moveEe` that keeps the tool's orientation moves the tool sideways by swinging shoulder_pan with
  wrist_yaw counter-rotating: about 24 deg of pan for 1 cm on the Metal arm. The tool point stays on
  the line; the elbow does not.
- Once, while the arm held still, a write to the adapter did not complete within 100 ms. The arm
  faulted that joint and kept holding; `stop()` and `returnToRest()` then refuse because of the
  fault, so another program had to bring the arm down. The adapter answered normally afterwards.
  Not explained.
- Measured tool position after `moveEe` is off by the servo's standing error under gravity
  (about 1 deg on the loaded joints: 6 to 9 mm at the Metal tool). The commanded pose is within 1 mm.
- The Metal wrist_flex joint lags its command by about 175 ms; the other joints by 65 to 100 ms.
- At `6142f53` the rest pose `stop()` returns to was the pose at `enable()`, and repeated
  enable/rest cycles crept: on the Maker arm the resting shoulder_lift moved from -4.5 to -14 deg in
  about 15 cycles and the unpowered arm then tipped backwards. The rest window (`restWindow`, set
  for the Maker arm) is meant to stop this; it has not run on the Maker arm yet.
- Headless Chromium lists no USB devices. For unattended runs use headed Chromium (under `xvfb-run`)
  with the `WebUsbAllowDevicesForUrls` policy granting the adapter to the page's origin; the page
  then finds it with `grantedAdapters()` and no chooser is shown.
