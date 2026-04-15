---
name: walle
description: Project-scoped performance lessons for WALL-E. Loaded automatically in this repo. Grow over time with durable rules distilled from actual sessions.
---

# WALL-E skill: performance rules for this project

Read this at the start of any non-trivial task. Each rule is a
durable lesson learned from a real incident — not speculation.

---

## Rule 1: Dockerfile `apt install` always needs `apt-get update` in the same RUN

**Why:** Stale apt indexes have produced two distinct bugs:
(a) ROS debs republished with new timestamps → 404s on old filenames
(teleop incident, commit `80cdcd6f`); (b) version skew across
dependency groups → libfastcdr/libfastrtps ABI break that crashed
every ROS node with `undefined symbol:
_ZN8eprosima7fastcdr3Cdr9serializeEPc` (slam-toolbox incident, see
memory `project_slam_fastcdr.md`).

**Apply:** Any new `RUN apt-get install -y <pkg>` in
`docker/Dockerfile` must be `RUN apt-get update && apt-get install
-y <pkg>`. If the package pulls in fastcdr/fastrtps transitively,
also `apt-get dist-upgrade -y` or pin versions. Verify with `ldd`
that a typesupport `.so` resolves against the installed libfastcdr
package version before declaring success.

---

## Rule 2: Check config defaults before theorising about architecture

**Why:** Julian and I burned hours chasing a "phantom obstacle" /
"TF tolerance" / "costmap persistence" theory for slow Nav2
movement. The actual cause was `vx_std: 0.05` in MPPI config — the
velocity sample noise was 4× too small so MPPI could never discover
fast trajectories. Fix was one number (commit `bffddb3f`).

**Apply:** Before proposing architectural changes, grep the relevant
config yaml for the parameter whose behavior seems off. Compare to
upstream defaults. A one-line value fix beats a ten-line refactor.

---

## Rule 3: Verify memory against current code before recommending

**Why:** Memory records a point-in-time truth. Files get renamed,
commits get reverted (four SLAM commits reverted 2026-04-14 and
2026-04-15), parameters change. Acting on stale memory wastes time.

**Apply:** If memory references a file path, flag, or function
name, `Glob`/`Grep` to confirm it still exists before telling Julian
to use it. If memory summarises a commit or branch state, run
`git log --oneline -10` to verify.

---

## Rule 4: Teleop vs Nav2 command flow is subtly wired

**Why:** `twist_mux` publishes to `/cmd_vel_out` at runtime even
though `twist_mux.yaml` declares `/cmd_vel` as the output. RoboClaw
subscribes to `/cmd_vel_out` via an explicit remap in
`roboclaw_launch.py:87`. This is load-bearing — "cleaning up" the
yaml will break the motors silently.

**Apply:** Never change the twist_mux yaml output topic. Never
change the RoboClaw remap without also adjusting every publisher
that feeds it. If you're introducing a node between twist_mux and
RoboClaw (e.g. collision_monitor), update the RoboClaw remap to
the new intermediate topic.

---

## Rule 5: Upstream nav2_bringup already launches collision_monitor (Jazzy)

**Why:** In Jazzy, `/opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py`
already instantiates the `collision_monitor` Node and includes it
in `lifecycle_manager_navigation`'s `node_names` list. Our
`nav2_no_map_params.yaml` supplies its config. The previous failed
attempt (`6b613f5d`, reverted) launched a **second** collision_monitor
with a **sidecar** `lifecycle_manager`, causing node-name collision
and dual-manager thrash that killed nav. The real fix for putting
the monitor inline in the cmd_vel chain was a one-line remap of
RoboClaw `/cmd_vel_out` → `/cmd_vel_safe` (commit `a2c735a2`,
verified working 2026-04-15).

**Apply:** Before inventing a launch-level enablement for any
nav2-adjacent node, grep `/opt/ros/jazzy/share/nav2_bringup/launch/`
for it. Upstream usually already wires the node; check before
duplicating.

---

## Rule 6: Respect the pre-SLAM baseline

**Why:** The "pre-SLAM" tree at `d52ba681` is the last known-good
state. The user's mental model of "working robot" anchors to this.
When reverting, revert **all the way** to matching this tree, not
to an interstitial state.

**Apply:** After any multi-commit revert, run `git diff d52ba681..HEAD
--stat`. Empty output = fully reverted. Non-empty = more work to do.

---

## Rule 7: RViz lives on the host, not the Jetson

**Why:** The Jetson's display is headless. RViz runs on Julian's
host machine and subscribes to Jetson topics over the LAN via
shared `ROS_DOMAIN_ID=62`.

**Apply:** When testing, open RViz proactively on the **host**
(separate terminal, not the SSH session) once the robot is up.
Don't suggest launching RViz on the Jetson. Don't open RViz until
testing actually starts — it's not needed at session open.

---

## Rule 8: Opus ≠ smarter by default — do the analysis anyway

**Why:** Julian switched the project to Opus after frustration with
vague suggestions. A smarter model is not a substitute for reading
the code.

**Apply:** For every non-trivial task: read the relevant files,
quote file:line, state the blast radius of the change in one
sentence, then propose. Do not skip steps.
