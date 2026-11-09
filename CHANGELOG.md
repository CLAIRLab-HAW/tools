# Changelog — onrobot-rg6

What changed when. The current state is described in the [README](README.md);
how it embeds into the onboard stack in
[docs/ARCHITECTURE.md](../../docs/ARCHITECTURE.md).

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
the versioning [Semantic Versioning](https://semver.org/).


## 2026-09-29 (rg6_moveit_patch replaces its block under any wording)

### Fixed

- `rg6_moveit_patch` finds its SRDF block by the `onrobot-rg6:BEGIN` tag instead of the whole marker line. The
  a200's `robot.srdf` still carried the block under the German marker text; the patch did not recognize it,
  appended a second `gripper` group, refused with "appears more than once" on every boot and left the SRDF as it
  was on 2026-09-09 -- `open` at 0.0 instead of the joint's lower limit 0.038.

## 2026-09-23 (rg6_control_sim drives a physics plant)


- **Plant mode** of `rg6_control_sim` (`plant_command_topic`): the action and
  `rg6/bridge_state` stay as they are, but the jaws are a simulator's. The
  speed-limited ramp is sent as the command of a position controller of
  `rg6_finger_joint` (std_msgs/Float64MultiArray), the width is read back off
  `plant_state_topic`, and a stop short of a closing target is
  `grip_detected`. The node then publishes no joint state of its own. Why:
  MARWIN 5's MuJoCo plant has to grip the cabinet door physically, behind the
  same surface the a200 mock and the robot offer.

## 2026-09-02 (one shape for the `__main__` block)

- **`if __name__ == "__main__":` now ends in `raise SystemExit(main())`** -- the single form the workspace
  settled on across its 97 entry points (rule and reasoning in `.claude/rules/code-style.md`). It needs no
  `import sys`, carries a `main` returning `int` and one returning `None` alike (`SystemExit(None)` is exit
  code 0), and matches the wrapper setuptools generates for `[project.scripts]`, so running the file, running
  `python -m`, and the installed console command now agree on the exit code.
- `tools/derive_link_inertia.py` only; was `sys.exit(main())`.

## 2026-09-01 (the finger joint stops dropping out)

- **`rg6_grip_bridge` publishes `rg6_finger_joint` in every pass, measurement or not.** New
  `finger_angle_to_publish` decides the angle: the measured one while the gripper measures, the last measured one
  while it does not, the open stop of the linkage table until there has been a first measurement. Until today the
  topic went silent whenever `state.readable` was false -- and the silence reached much further than it looked.
- **What the silence cost, measured at the robot on 2026-09-01 with the arm unpowered:** without the joint,
  `robot_state_publisher` places none of the eight movable RG6 links, so they have no TF, so `move_group`'s
  `shape_mask` cannot mask them out of the depth cloud. Ten `Missing transform for shape mesh` lines per cloud
  (handles 14...23 -- the ten collision meshes on those eight links; both `finger_tip` links carry two), the whole
  hand out of the octomap self filter with the camera sitting on the gripper, and an occupancy update every 2.44 s
  instead of 5 Hz, because the updater waits `shape_transform_cache_lookup_wait_time` = 0.3 s **per** missing link.
  Context and the earlier measurement: R43 in the workspace's ROBOTER-TODO.
- **A held value is not a claimed measurement.** `status_payload` is untouched, so the manipulator diagnostics still
  reads `status: -1`/`safety_failed: true`; an endpoint that does not answer at all still leaves `bridge_state`
  silent, which is that path's own signal. On top of that the substitution is edge-triggered in the log -- one line
  when it starts, one when the measurement returns, with the duration.
- **Why the last measured value before the open stop:** the state this guards against is an unpowered tool
  connector, and an unpowered gripper does not move, so the last measurement is the physically correct angle rather
  than a guess. The open stop is only the fallback for "there has not been one yet", and it is read off the table
  (`q_min`) instead of written as a literal -- it is 0.038 rad (151.1 mm), the widest the DEVICE reaches, and not
  the 0.0 the SRDF's `open` group state carries. That difference is the known
  `Deviation in joint rg6_finger_joint: [0] != [0.038]`.
- **The self-test grew a section** (measured angle, held value, open stop, and the unreachable endpoint), so
  `tests/test_grip_bridge.py` covers the new path through the same subprocess invocation the installer runs.
- `rg6_control_sim` needed no change: it publishes the driver joint unconditionally at 50 Hz.

## 2026-08-31 (the gripper's driver half comes home)

- **`rg6_grip_bridge.py` and its linkage table moved into `rg6_control`.** The bridge is the DRIVER half of a pair
  whose MOCK half (`rg6_control_sim`) has been in this package all along: same action under the same name, same
  `bridge_state` fields character for character, same gear table. That the one lived here and the other in the
  robot's setup repo was an accident of history -- the C++ driver over tool DO0 was deleted when the RG6 moved onto
  the OnRobot URCap, and its replacement was written where the systemd unit already stood.
- **It is not robot-specific.** Everything that names this a200 is a ROS parameter with a default
  (`endpoint_url`, `manipulators_ns`, `driver_joint`, `force_range_n`, `kinematics_file`); what is left is a
  generic RG6-over-URCap driver, which is what a gripper package describes.
- **What did NOT move: the service.** Unit, wrapper and the root-owned copy under `/usr/local/bin` stay in
  `husky-custom-setup` -- a systemd unit is robot setup. The installer deploys the file out of this workspace, the
  same way it already deployed `rg6_moveit_patch`.
- **Two of the three table copies now lie next to their generator.** `derive_finger_kinematics.py` wrote the JSON
  into a FOREIGN repo by shell redirect until today; `test_linkage_parity.py` reaches across a repo boundary for
  one file instead of three, and CI clones one sibling instead of two.
- **The install rule puts the table in `lib/rg6_control`, not under `share/`.** The bridge resolves it with
  `Path(__file__).with_name()`, so script and table have to stay adjacent in the install space as well -- which is
  also how the robot's root-owned pair sits under `/usr/local/bin`.
- **New `tests/test_grip_bridge.py`.** It runs the node's own `--selftest` as a subprocess (a fake URCap: units,
  float coercion, clamping, timeout, status message, linkage, concurrency), and pins the three properties the
  layout depends on: the table lies beside the script, the module imports nothing the robot has no packages for,
  and the names other code reaches for still exist.
- The generated header was regenerated rather than hand-edited; against `urdf/robot.urdf` the table came out
  byte-identical, only its comment changed.

## 2026-08-31 (this package describes a hand, not a robot)

- **The Clearpath extras left for a repo of their own** (`husky-extras`,
  `husky_extras_description`). `urdf/clearpath_extras.urdf.xacro` and
  `meshes/husky_sensor_arch.gltf` are gone from `rg6_description`: the sensor arch and the ArUco marker are
  platform parts, and the gripper block in that file was never the gripper but its MOUNTING -- it names
  `arm_0_tool0`, `top_plate_rear_mount` and `top_plate_front_mount`, frames that exist only once the Clearpath
  generator has built an a200 with a UR5 on it.
- **What that cost while it was here: reusability.** A package that describes an RG6 *and* where it sits on one
  particular Husky cannot be used on any other arm. It now describes the hand -- macro, meshes, measured
  `rg6_v2.yaml` -- and knows no frame of the a200. The include runs the other way: the new package instantiates
  `onrobot_rg_upstream.urdf.xacro`, and nothing points back here.
- **`robot.yaml` addresses the file in the new repo now**, and lists its workspace next to this one's
  (`system.ros2.workspaces`). Both are needed together: the generator finds the extras file by path and then
  expands `$(find rg6_description)` through the ament index.
- `tools/derive_link_inertia.py` keeps its box mode -- it reads boxes out of any link and takes the xacro as an
  argument -- but the link it was written for, `husky_top_assembly`, is now in the neighbouring repo. Its
  docstring says so rather than leaving a path that no longer exists.

## 2026-08-31 (the named posture 'open' is inside the joint it is defined on)

- **`rg6_moveit_patch` wrote a `group_state` the joint cannot reach.** `--angle-open` defaulted to `0.0`, the
  geometric zero of the four-bar chain, while `rg6_v2.yaml` clamps `limit.lower` to the mechanical open stop at
  0.038 rad. MoveIt validates a named posture against the joint limits of the model and refuses anything outside
  them: measured on 2026-08-28 on the a200-0553, every MoveIt Task Constructor grasp died with "Goal state is out
  of bounds!" after finding 28, 32, 37 and 28 valid IK solutions -- none of them connectable, because the pregrasp
  they all start from does not exist.
- **The angle comes out of `rg6_description/config/rg6_v2.yaml` now, not out of a literal.** The SRDF posture and
  the URDF limit are the same number, so they are read from the same place: `find_model_config` looks along
  `AMENT_PREFIX_PATH` and then beside the script itself, which covers the checkout as well as colcon's isolated
  and merged install spaces. AMENT_PREFIX_PATH alone would not do -- the offboard entrypoint sources the rg6
  overlay in a subshell around the generator run and calls the tool outside it.
- **One literal is left, and it is pinned.** `OPEN_ANGLE_FALLBACK` carries 0.038 for the case where the config is
  unreadable or PyYAML is missing; `tests/test_rg6_moveit_patch.py` asserts it against `limit.lower`, so it cannot
  drift. Falling back to the geometric zero instead would have put the refused posture straight back.
- **New `tests/test_rg6_moveit_patch.py`**, the first tests this tool has: the default IS the limit, the patched
  SRDF states a posture INSIDE it, a missing config warns instead of writing zero, and an explicit `--angle-open`
  still wins.
- **The workaround in `husky-offboard`'s `entrypoint.sh` can go, and with it R44.** It rewrote
  `value="0.0"` to the URDF's lower limit with `sed` after every generator run, and said so in its own comment: the
  value comes out of onrobot-rg6 and belongs fixed there. The pattern it greps for no longer occurs -- both repos
  are outside this one, so retiring it is their commit.
- **The help text of `--angle-open` said 153.2 mm.** That is what the chain computes at `q = 0`, an opening the
  hardware never delivers; at the stop the hand is 151.1 mm wide (measured 2026-08-27 with a caliper, 151.13 mm in
  the linkage table). The stroke figure in the `disable_collisions` rationale was stale for the same reason.

## 2026-08-31 (the gripper weighs what the data sheet says)

- **The `inertial:` block of `rg6_v2.yaml` is derived instead of inherited.** Upstream shipped one placeholder for
  every part -- `mass: 0.001` with `ixx = iyy = izz = 0.001` -- wrong in both directions at once: three orders too
  light for a hand the data sheet puts at 1.25 kg, about four orders too stiff for a part that fits in 3 cm. The
  whole model weighed 0.789 kg, so 0.461 kg were missing from the wrist of the UR5.
- **New `tools/derive_link_inertia.py`.** Integrates each collision mesh exactly (Eberly, polyhedral mass
  properties), gives the parts one homogeneous density and fixes it so the sum over all instances is the data
  sheet's 1.25 kg. Density comes out at 1343.1 kg/m^3.
- **The result is cross-checked against the second data sheet figure, and lands.** Nothing in the derivation aims
  at the centre of gravity -- the distribution is fixed by volume alone -- so it is a free result: 90.58 mm above
  the tool base point against the published 90.0 mm, 0.58 mm out. That also settles a reading the mass alone
  leaves open: the 1.25 kg includes the mounting bracket.
- **The `_inertial` macro now writes an `<origin>`.** Without it URDF reads a tensor taken about the part's centre
  of mass as being about the link frame; for the body that is 52 mm out, and it parses either way.
- **`husky_top_assembly` has an `<inertial>`.** It carried six collision boxes and no mass, so
  `twinlink.urdf_mujoco._ensure_inertial` was substituting 0.1 kg for a half-metre portal frame -- a factor of 62.
  2.3 L of measured structure volume at the assumed density of 6xxx aluminium gives 6.21 kg, distributed over the
  six boxes by volume. The density is an assumption, not a measurement; R47 carries the scale.
- **The patcher for those apt-side gaps lives in `husky-custom-setup`, not here.** `urdf_physics_patch` edits the
  UR joints, the wheel joints and the a200 top plate -- an arm and a platform concern, with not one gripper part
  among its targets -- so it sits next to `clearpath_custom_setup.py`, whose package edits it is the third of.
  `rg6_moveit_patch` stays: it writes a foreign file about this repo's own subject.

## 2026-08-30 (ruff resolves the same settings from anywhere)

- **CI pins `ruff>=0.16.5,<0.17`** -- the minor the lint scope was measured against, the same bound the
  workspace dev group carries. Unpinned, a ruff release can stabilise new rules and turn this CI red without
  a commit of ours.

## 2026-08-30 (the gripper URDF parses again)

- **An XML comment cannot contain `--`.** The English rewrite of 2026-08-29 left `visual mesh -- hence` in the
  collision-block comment of `onrobot_rg_upstream.urdf.xacro`, which makes the file not well-formed. Every
  consumer of the robot URDF fails on it: `mock` aborts in `_process_urdf` with
  `ExpatError: not well-formed (invalid token): line 221, column 91`, so `move_group` never comes up and the whole
  offboard stack stays down.
- It stayed invisible because the container images had the pre-rewrite file baked in; a rebuild is what surfaced
  it. The em-dash of the prose is now a comma -- the only place in the workspace where the sentence and the file
  format disagree about `--`.

## 2026-08-29 (the prose and the output of this repo are English)

- **`rg6_moveit_patch` logs, help texts and SRDF marker are English.** The marker line inserted into
  `robot.srdf` reads `<!-- onrobot-rg6:BEGIN (rg6_moveit_patch; do not edit by hand) -->`. Only the marker's
  own text changed, not where it goes or how it is matched, and the SRDF is regenerated on every boot before the
  patch runs, so nothing carries an old marker across.
- **The comments in the three C++ nodes and in the two xacro files are English**, transliterated umlauts and
  all, and `LICENSE-THIRD-PARTY.md` is an English document — its subject is a third-party licence, which is
  read outside this workspace.

## [Unreleased]

### Added

- **A test suite for the linkage table and its generator** (`tests/`, 77 tests, plain pytest from the workspace
  root). The table exists three times over — the generated C++ header here, `rg6_finger_kinematics.json` next to the
  gripper bridge on the robot, and `gripper.linkage.table` in the `robot_contract` profile — and each copy carries
  its own interpolation code. Only the generator writes the first two; the profile is a HAND copy, exactly like the
  arm poses were before `test_ssot_parity.py`, and those had already drifted. A drift here would make the container
  mock compute a different width from the same command than the real robot, which is the one thing the mock exists
  not to do. The suite compares the data and the results: it compiles the generated header with a probe program and
  checks it against both Python implementations, and it covers the generator itself against synthetic chains. It
  needs neither ROS nor a robot, and skips by name where a neighbouring repo is not checked out or no C++ compiler is
  on PATH.

- **A GitHub workflow** (`.github/workflows/ci.yml`) that clones `husky-custom-setup` and `robot-contract` into the
  workspace layout so the cross-repo comparisons actually run, compiles the generated header with the runner's own
  compiler and runs the suite. No docker, no ROS, no robot. `colcon test` is not run: `rg6_control` declares no ament
  linters on purpose, so it would build a ROS workspace to execute nothing.

### Changed

- **A missing sibling repo now FAILS the suite instead of skipping it, once a workspace root is found.** Skipping is
  right for this repo cloned on its own — it cannot know where its siblings would be — but wrong in CI, which is the
  one place the checkouts are scripted: a mistyped path there would turn every cross-repo comparison into a skip and
  report green having compared nothing. Verified by removing the checkout in a CI-shaped copy: 2 failed, 26 errors,
  naming the missing path; without the `workspace.repos` marker the same tree is 48 passed, 29 skipped. Same rule as
  `libs/clearlog`'s shell-parity test.

  The shared helper is `tests/table_sources.py`, named for this repo's subject and not the obvious `siblings`:
  `deploy/husky-offboard/tests` already carries a helper of that generic name, both conftests put their own directory
  on `sys.path`, and in the shared root run whichever landed first wins for BOTH trees while the loser dies at import.
  Test-helper module names are effectively global in this workspace.

  Verified by mutation rather than by passing: eight seeded faults — a drifted digit in either copy, the C++ clamp
  removed, the inversion searching the wrong way, the upper joint limit not sampled, the two-gripping-face check
  dropped, a sign flipped in the rotation matrix, the accuracy figure measured against a degenerate grid — were each
  confirmed to turn the suite red.

- **`derive_finger_kinematics.py` is English throughout the interpreter-visible layer** — `_kette` -> `_chain`, the
  refusal when a URDF does not yield two gripping faces, and the `argparse` help. The German comment in the generated
  C++ template was pulled along in both the template and the checked-in header, so the two stay identical.

- **The `BUILD_TESTING` note in `rg6_control/CMakeLists.txt` said there was no characteristic curve left to check.**
  That is true of the AI2 analogue curve, which went away with the tool-connector driver, and not of
  `finger_kinematics.hpp`, which is one. The note now says where the table is checked instead.

## 2026-08-26 (rg6_msgs removed)

### Removed

- **`rg6_msgs` is gone** — `msg/GripperState` and `srv/Grip`, the interfaces of the tool-DO driver retired on
  2026-08-19. Nothing built them and nothing read them: `rg6_control` dropped the dependency the same day, the
  robot installer builds `--packages-select rg6_description rg6_control`, and the container's
  `colcon build --packages-up-to rg6_control` therefore never pulled it in. Measured in the running
  `husky-offboard-mock-robot-1` on 2026-08-26: `/opt/onrobot-rg6/install/` holds `rg6_control` and
  `rg6_description`, and `ros2 pkg list | grep rg6` names exactly those two. The package had not been shipped for
  a week.
- **The reason given on 2026-08-19 for keeping it does not hold.** The claim was that the five recordings in
  `clearpath/data/recordings/` carrying `/a200_0553/manipulators/rg6/state` would become unplayable. They do carry
  it (`freedrive`, `greifer`, `lifecycle`, `pstop-recovery`, `stapel-episode`; `fahren` and `record_2026_06_25_3`
  do not) — but the recordings are self-describing: each MCAP embeds the full `.msg` definition, `std_msgs/Header`
  included, as the connection's message definition. Measured on 2026-08-26 with no ROS installation present,
  `rosbags.AnyReader` deserialises the topic correctly (`width=0.16`, `busy=False`). The workspace's own replay
  path never touches it either — `twinlink`'s `McapSource` reads only `mapping.topics()`, and no mapping names a
  gripper topic. What would still need a built interface package is ROS-native `ros2 bag play`, and no script,
  test or skill in the workspace runs it.

## 2026-08-24 (.gitignore normalised to the workspace base)

- **`.gitignore` now uses the workspace's lean 8-line base** (`__pycache__/`, `*.py[cod]`, `*.egg-info/`, `build/`, `dist/`, `.venv/`, `.pytest_cache/`, `.DS_Store`); replaces the ~280-line auto-generated toptal.com template (Django/Flask/C/C++ patterns this package never produces). Package-specific extras: `docs/`, `install/`, `log/`, `*.pcd`, `COLCON_IGNORE`, `AMENT_IGNORE`.

## 2026-08-23 (Bezeichner auf Englisch)

- **Die Bezeichner dieses Pakets sind englisch**, die Prosa bleibt deutsch —
  dieselbe Konvention wie in `sdk/skill-tree` und wie CLAUDE.md sie vorgibt
  ("Doku ist deutsch"). Umbenannt wurden Funktionen, Klassen, Konstanten,
  Parameter und lokale Variablen; Docstrings und Kommentare NICHT.
- **Was ein Programm AUSGIBT, bleibt deutsch**: Abschnittsmarken, JSON-Feld-
  namen und Log-Meldungen sind der Bericht an den Menschen, nicht Code.
- Umbenannt wurde mit einem `tokenize`-Werkzeug (nur NAME-Token), nicht per
  Regex — deshalb ist kein Kommentar und kein String mitgewandert. Drei
  Stellen, die `tokenize` NICHT sieht, wurden eigens nachgezogen:
  f-String-Interpolationen (unter Python 3.11 ist ein f-String EIN Token),
  die Parameternamen in `pytest.mark.parametrize` und Bezeichner, die
  quelltextlesende Tests als String erwarten.
- Gegengemessen: `uv run pytest` steht unveraendert bei 2465 passed,
  3 skipped — derselbe Stand wie vor der Umbenennung.

## [Unreleased]

- **The open end of the finger table is the mechanical stop, not the geometric zero of the chain.**
  `rg6_v2.yaml` clamps `limit.lower` from `0.0` to `0.038` rad, the same class of clamp `limit.upper` already
  carries. Measured on 2026-08-27 at the a200-0553 with a caliper, pad face to pad face: 151,10 mm at the stop,
  while the four-bar chain computes 153,17 mm at q = 0 -- 2,07 mm of opening the model handed out and the hardware
  never delivered. It is not an error of the chain: in the same series the model was right to within 0,30 mm at
  q = 0,20 / 0,41336 / 0,73487 / 1,00485 (140,25 against 140,08 mm; 119,70 against 120,00; 79,95 against 80,00;
  40,15 against 40,00), so it carries to right next to the stop and only the stop falls out. Nor is it the
  direction dependence, which was measured at 1,00 mm at q = 0,20 and points the other way (opening reads WIDER,
  and the stop can only be reached by opening). The regenerated table now starts at 151,13 mm, 0,03 mm off the
  measurement.

- **`derive_finger_kinematics.py` ignored `limit.lower` and sampled from 0 regardless.** It read only `upper` and
  started every table at zero, so a clamped URDF and its table would have described the same joint differently --
  the URDF refusing an angle the table still offered a width for. It now reads both ends and samples over
  `[lower, upper]`; `joint_limits_rad` carries the real lower bound.

- **`--format cpp` could not run at all.** `HPP_HEAD` still used the German placeholders `{fehler_mm}` and
  `{zeilen}` while the call passed `error_mm` and `lines`, so every invocation died with `KeyError` -- a leftover
  of the German-to-English pass. The C++ header for `rg6_control_sim` was therefore unregenerable; it is fixed and
  the header is regenerated, including a `kQMinRad` that is no longer hard-coded to 0.

### Hinzugefügt
- **`husky_top_assembly` hat Kollisionsgeometrie (ROBOTER-TODO R15).** Der
  Sensorbügel war im Viewer sichtbar und für `move_group` nicht vorhanden:
  eine Bahn konnte hindurchführen, ohne dass Planer oder
  `check_state_validity` es bemerkt hätten. Er steht 0,772 m von der Armbasis,
  der UR5 reicht 0,85 m — er ist erreichbar.

  **Keine Gesamt-Bounding-Box.** Das Mesh füllt seine Hülle nur zu **2,4 %**
  (2,3 L Struktur in 91,3 L Hülle); eine einzelne Box hätte das ganze Heck
  zugemauert. Stattdessen **sechs Kästen** entlang der aus dem Mesh
  dekodierten Struktur — zwei Portalbeine, Querträger, Sensorkopf, zwei Füße
  mit Diagonalstrebe — zusammen **5,87 L**. Nachgerechnet: **alle 4560
  Vertices** liegen darin, keiner außerhalb.

- **Die Kästen beginnen 2 mm über dem Fuß, und das ist der Punkt.** Der
  Link-Ursprung sitzt bei Welt-z 0,2307, exakt auf der Oberkante von
  `top_plate_link` (0,2240…0,2307). Bei z = 0 berühren sich beide Körper, und
  MoveIt liest eine Berührung als **Dauerkollision**: der Roboter wäre in
  jeder Stellung ungültig gewesen und `move_group` hätte nichts mehr geplant.
  Am geladenen `robot_description` des Roboters gegengerechnet — mit dem
  Schnitt: **null dauerhafte Überlappungen** für alle sechs Kästen.

- **Der Frontstoßfänger bekommt bewusst nichts.** R15 führt
  `front_bumper_link` als „in Reichweite, echt ungemodellt". Nachgemessen ist
  er das nicht: sein Volumen (x 0,406…0,494, y ±0,267, z 0,078…0,104) liegt
  **vollständig** in der unteren Kollisionsbox von `base_link`
  (x ±0,494, y ±0,285, z 0…0,124). Eigene Geometrie hätte nichts abgedeckt,
  was nicht schon abgedeckt ist — derselbe Fall wie `top_chassis_link`, den
  R15 selbst als redundant führt.

### Behoben
- **`rg6_moveit_patch` gibt das Paar `flex_finger` <-> `flex_finger` frei --
  ohne das war der Greifer in MoveIt nicht zu schliessen.** Die beiden
  Gummipads beruehren sich nur in den letzten 1,3 % des Hubs (gemessen am
  laufenden `move_group` per Bisektion: letzte gueltige Stellung 1,238486 rad
  = 3,01 mm lichte Weite, Kontaktband 0,016294 rad). Dort landet das Sampling
  des `moveit_collision_updater` praktisch nie, das Paar galt ihm als
  "manchmal in Kollision" und blieb aktiv.

  Folge: `move_group` verwies jede Stellung enger als 3 mm -- waehrend
  dieselbe SRDF einen `group_state` `close` bei 1,25478 rad (0,40 mm) anbot.
  Das Modell verbot seinen eigenen Named State, und nichts duenner als 3 mm
  war ueber MoveIt greifbar. Der Hub geht von 153,17 mm auf 0,40 mm.

  Freigegeben statt `close` entschaerft: die Selbstkollisionspruefung soll den
  Roboter vor sich selbst schuetzen, und ein leer zufahrender Greifer ist der
  Normalfall, den die Hardware bei jedem Griff ausfuehrt. Die zwei Pads SIND
  die Greifflaechen. Der echte Anschlag bleibt das Gelenklimit.

  `reason="User"`: weder `Adjacent` (im URDF keine Nachbarn) noch `Default`
  (in der Standardstellung 153 mm auseinander) trifft zu.

  Damit sind es drei gepatchte Paare statt zwei. Das dritte fehlte aus dem
  umgekehrten Grund wie die ersten beiden: die sind ueber den ganzen Hub in
  Kontakt, dieses fast nirgends.

## [0.2.0] - 2026-08-19

### README auf den Ist-Zustand
- **Die README beschrieb noch den geloeschten Tool-DO-Treiber.** Sie fuehrte
  neun `rg6_control/*`-Services, das Topic `rg6/state`, den
  `rg6_joint_state_broadcaster`, die AI2-Kennlinie samt Kalibrierverfahren, das
  17-Bit-URScript-Protokoll und `rg6_bringup.launch.py` als aktuellen Stand --
  und schrieb im ersten Absatz vor, die OnRobot-URCap muesse **aus** bleiben,
  waehrend genau sie der Steuerweg ist. Alle Beispielaufrufe zeigten auf
  Schnittstellen, die es nicht mehr gibt.

  Neu geschrieben auf das, was das Repo heute ist: Modell (`rg6_description`),
  MoveIt-Anbindung (`rg6_moveit_patch` + robot.yaml), joint_states-Verrohrung
  und Container-Mock (`rg6_control_sim`) -- mit dem ausdruecklichen Hinweis,
  dass der reale Greifer von `rg6_grip_bridge` in husky-custom-setup
  kommandiert wird. Die kanonische Schnittstelle (Action `gripper_cmd`,
  Zustand `rg6/bridge_state`, Gelenkwert in rad statt Weite) steht jetzt ganz
  oben, weil sie auf beiden Stufen dieselbe ist.
- **Historische Bezuege aus den Quellkommentaren entfernt.** Was einmal war,
  steht in dieser Datei; im Quelltext stand es doppelt. Betroffen:
  `rg6_control_sim.cpp` (der "AM 2026-08-19 ZURUECKGESCHNITTEN"-Block,
  dreimal "Bis 2026-08-19 stand hier ..."), `joint_states.launch.py`,
  `CMakeLists.txt`, `package.xml`, `rg6_moveit_patch`, `rg6_v2.yaml`.
  Die Warnungen selbst sind geblieben, nur ohne Datum und Vorgeschichte --
  etwa dass ein RobotState mit einem modellfremden Gelenknamen `move_group`
  ueber `getVariableIndex` zum Absturz bringt.
- **Ein toter Verweis dabei gefunden:** `rg6_moveit_patch` begruendete seine
  Endwinkel mit `rg6_control/analog_model.hpp` -- die Datei ist mit dem
  Analogmodell entfallen. Die Werte kommen aus `finger_kinematics.hpp`.
- Gemessene Zahlen behalten ihr Datum. Sie sagen, wie frisch der Wert ist.

### moveit.yaml zurueck an robot.yaml

- **`rg6_moveit_patch` patcht nur noch die SRDF und prueft den Rest.** Die
  zweite Haelfte des Tools -- GripperCommand-Controller
  `manipulators/rg6_gripper_controller` und
  `robot_description_planning.joint_limits.rg6_finger_joint` -- ist nach
  `robot.yaml` gewandert (`manipulators.moveit.ros_parameters.move_group`,
  husky-custom-setup). Nachgemessen im Container: der Clearpath-Generator
  schreibt aus diesem Block eine `moveit.yaml`, die mit der frueher gepatchten
  bis auf die **Reihenfolge** von `controller_names` identisch ist (als Menge
  gleich, Rest deckungsgleich); `robot.srdf` blieb byteweise identisch. Damit
  steht die Greifer-Anbindung in der versionierten SSOT statt in einem
  Python-Template, gilt fuer Roboter und Offboard-Container zugleich, und eine
  der beiden generierten Dateien verlaesst den Patch-Pfad.
- **Fuer die SRDF gibt es diesen Weg nicht -- am Quelltext belegt.**
  `clearpath_config` enthaelt das Wort `srdf` an **keiner** Stelle, es gibt
  kein Gegenstueck zu `platform.extras.urdf` (`ExtrasConfig` kennt nur `urdf`,
  `launch`, `ros_parameters`), `robot.srdf.xacro` entsteht ausschliesslich aus
  den drei Schleifen Arms/Grippers/Lifts, und `Gripper.MODEL` ist ein
  geschlossenes Enum (`franka_gripper`, `kinova_2f_lite`, `robotiq_2f_140`,
  `robotiq_2f_85`), das `rg6` mit `ValueError` ablehnt. Der einzige
  SRDF-Hebel in robot.yaml ist `poses` -> `group_state`, und der haengt an
  einem Manipulator-Objekt, das es fuer den RG6 nicht geben kann.
- **Neu: `verify_moveit_yaml` statt stiller Fahrt ohne Greifer.** Fehlen die
  Werte in der generierten `moveit.yaml` -- veraltete `robot.yaml` unter
  `/etc/clearpath` oder aus `ROBOT_YAML_URL` --, listet das Tool jeden
  fehlenden Schluessel auf, nennt die SSOT und endet mit **Exit-Code 1**.
  Ohne diese Pruefung faellt der Fall erst auf, wenn ein GripperCommand-Goal
  ins Leere laeuft. Kein Aufrufer bricht daran ab; Installer und
  offboard-Entrypoint loggen den Code. Entfallen sind die Argumente
  `--action-ns`, `--max-effort`, `--max-velocity`, `--max-acceleration` --
  ihre Werte stehen jetzt in robot.yaml.

### Altlasten

- **`rg6_control_sim` bildet nur noch die Oberflaeche der Bruecke nach.**
  Entfallen sind die Services `rg6_control/{open,close,grip,set_force_preset,
  set_tool_power}`, das Topic `rg6/state` (`rg6_msgs/GripperState`) und das
  AI2-Modell. Sie gehoerten zum geloeschten Tool-DO-Treiber; am Roboter gibt
  es sie nicht mehr, und seit `husky_sdk` auf die Action umgestellt ist, ruft
  sie niemand. Ein Mock, der eine Oberflaeche nachbildet, die es nirgends
  gibt, ist eine Vorlage fuer Code gegen etwas, das nicht existiert. Der Node
  ist von 542 auf 347 Zeilen geschrumpft; am laufenden Container nachgesehen
  bietet er jetzt genau `rg6_gripper_controller/gripper_cmd`,
  `rg6/bridge_state` und das Treibergelenk.
- **Das alte Greifermodell ist weg** — `onrobot_rg6.xacro`,
  `onrobot_rg6_model.xacro`, `onrobot_rg6_model_macro.xacro` und die vier
  Meshes `visual/{base_link,inner_finger,inner_knuckle,outer_knuckle}.stl`.
  Sie referenzierten nur noch einander: eine geschlossene tote Insel, die
  jederzeit wieder jemand haette einbinden koennen.
- **`analog_model.hpp` und `test_analog_model.cpp` sind entfallen.** Die
  Kennlinie Greifweite <-> Tool-AI2-Spannung beschreibt einen Kanal, an dem
  der RG6 nicht mehr haengt, und die Kurbelschwinge darin gehoerte zum alten
  Modell. Was der Greifer im Container tut, prueft jetzt eine E2E gegen den
  laufenden Stack (`plan-bridge/tests/test_offboard_gripper.py`).
- `std_srvs`, `rg6_msgs` und `ur_robot_driver` sind aus den Abhaengigkeiten
  von `rg6_control` entfallen.
- **`rg6_msgs` BLEIBT** und sagt jetzt in seiner Paketbeschreibung warum:
  alle fuenf Aufnahmen in `clearpath/data/recordings/` enthalten
  `/a200_0553/manipulators/rg6/state` vom Typ `rg6_msgs/msg/GripperState`.
  Ohne das Paket sind sie nicht mehr abspielbar -- es ist die Typdefinition
  eines Archivs, keine Altlast.

### Nachlese zum URCap-Umbau

- **Der SRDF-Patch traegt wieder zwei `disable_collisions` — und ohne sie plant
  `move_group` gar nichts.** Der Patch hatte seine Eintraege am selben Tag
  entfernt, in der Annahme, der `moveit_collision_updater` erzeuge alle 82
  Greiferpaare selbst und korrekt. Er erzeugt sie — bis auf
  `moment_arm ↔ finger_tip` desselben Fingers. Diese beiden Glieder der
  Viergelenkkette beruehren sich ueber den GANZEN Hub; gemessen mit
  `check_state_validity` bei `rg6_finger_joint` von 0,0 bis 1,25478: in jeder
  Stellung zwei Kontakte, immer dieses Paar. Die Folge war nicht eine
  schlechtere Planung, sondern keine: `CheckStartStateCollision` brach die
  Pipeline ab (`error_code 99999`, 0 Trajektorienpunkte). Mit den zwei Paaren
  ist der ganze Hub gueltig und dasselbe Ziel plant mit 63 Punkten.
- **Der Greifer-Mock spricht das Gelenk, das das Modell hat.** `rg6_control_sim`
  publizierte sechs Gelenke, fuenf davon mit den Namen des alten Modells
  (`left_inner_knuckle_joint` & Co.), und den Treiber bei ganz offener Hand mit
  **-0,93766 rad** — ausserhalb der Grenzen des rg6_v2 (0,0 … 1,25478). Der Wert
  kam aus der Kurbelschwinge des alten Modells (`rg6_control::linkage`), deren
  q=0 bei 93,4 mm liegt statt bei 153,2 mm. Jetzt: nur das Treibergelenk, aus
  der Tabelle des generierten URDF (`finger_kinematics.hpp`). Die fuenf
  Folgegelenke haengen per `<mimic>` am Treiber und werden ohnehin abgeleitet.
  **Das war kein Schoenheitsfehler:** ein `RobotState` mit einem dieser Namen
  brachte `move_group` ueber `RobotModel::getVariableIndex` zum Absturz
  (`std::terminate`, SIGABRT, zweimal reproduziert) — jeder Verbraucher, der
  `joint_states` liest und in eine MoveIt-Anfrage zurueckgibt, war eine
  Abschussrampe.
- **`rg6_control_sim` publiziert `rg6/bridge_state`** (`std_msgs/String`, flaches
  JSON) unter demselben Namen und mit denselben Feldern wie `rg6_grip_bridge`
  am Roboter. Ohne das las der `plan_server` im Container ins Leere.
- `tools/derive_finger_kinematics.py` kann die Getriebetabelle zusaetzlich als
  C++-Header ausgeben (`--format cpp`). Zwei Ausgabeformate, EINE Quelle: die
  Bruecke liest JSON, der Sim braucht C++, und zwei handgepflegte Tabellen sind
  genau die Zweitfassung, vor der die Datei selbst warnt.
- `ur_msgs` ist aus `package.xml`/`CMakeLists.txt` entfallen (die Abhaengigkeit
  gehoerte dem geloeschten Realtreiber); die Paketbeschreibung sagt jetzt, was
  das Paket noch enthaelt.

- **Der SRDF-Patch ist halb so gross und nicht mehr schaedlich.** Er trug die
  Linknamen des ALTEN Greifermodells hart ein (`left_inner_knuckle`,
  `right_outer_knuckle`, …) und haette sie nach jedem Boot in ein korrektes
  SRDF zurueckgeschrieben; sein Idempotenz-Check merkt das nicht, weil er nur
  prueft, *ob* sein Block dasteht, nicht *ob dessen Links existieren*.
  Gemessen und daraufhin entfernt: die **`disable_collisions`-Haelfte ist
  ueberfluessig geworden** — das frisch erzeugte SRDF enthaelt bereits **82**
  Greiferpaare, die der `moveit_collision_updater` aus dem neuen URDF selbst
  ableitet. Sie war ein Behelf fuer das selbstgebaute Modell.
  **Geblieben** sind Planungsgruppe, `group_states` open/close und
  `end_effector` (mit den neuen Gelenknamen und den Ankern 0,0 / 1,25478) sowie
  der `moveit.yaml`-Teil: GripperCommand-Controller, `max_effort` und
  `joint_limits`. Damit kann MoveIt den Greifer weiter oeffnen, schliessen,
  auf eine Weite fahren — und die Kraft kommt entweder aus `max_effort` oder
  je Ziel aus dem `effort`-Feld des Bahnpunkts (`gripper_command_controller_handle.hpp`
  Zeile 143 gegen 147).
  Gegenprobe am Container: das SRDF nennt **keinen** Link und **kein** Gelenk,
  das es im URDF nicht gibt (vorher sieben).


- **Das vermessene RG6-Modell ist einvendort** (`onrobot_description`, rg6_v2,
  MIT, INRIA — Herkunft in [LICENSE-THIRD-PARTY.md](LICENSE-THIRD-PARTY.md)):
  sieben visuelle Meshes plus eigene Kollisionsmeshes, Ursprünge und Grenzen
  als Daten in `config/rg6_v2.yaml` statt fest im Xacro. Der Treibergelenkname
  ist parametrisiert, damit `rg6_finger_joint` bleibt, was es ist. Gegenüber
  Upstream entfernt: der `<ros2_control>`-Block und der Gazebo-Aufruf — der
  Greifer hängt an diesem Roboter nicht am `controller_manager`, sondern an
  der XML-RPC-Brücke. Verdrahtet ist es noch nicht; das Modell im Betrieb ist
  weiterhin das alte.
- **Die Bracket-Geometrie des neuen Modells stimmt an der Hardware:** der
  Owner hat 81 mm Bracket-Höhe und ~41 mm Einstecktiefe des Greiferkörpers
  gemessen, das Mesh sagt 81,1 und 41,5 mm.
- **Ein Irrtum desselben Tages, am selben Tag zurückgenommen:** zwischenzeitlich
  stand `rg6_tool0_to_base` auf `xyz="0 0 0.0196"` — 19,6 mm für eine
  Quick-Changer-Roboterseite, die ROBOTER-TODO R20 aus dem Datenblatt
  erschlossen hatte. Am Gerät nachgesehen sitzt das Bracket **unmittelbar auf
  dem UR-Flansch**; sein Fuss ist selbst die Kopplungsfläche (Ø 71 mm, an
  beiden Modellen am Mesh nachgemessen). Der Wert steht wieder auf `0 0 0`.

---

**Vor der Einführung von SemVer (2026-08-19)** wurde nach Datum
geführt. Die Abschnitte darunter behalten ihre Datumsüberschrift — ihnen
nachträglich Versionsnummern zu geben, würde eine Release-Historie
erfinden, die es nicht gab.
- **SemVer eingeführt.** Version auf `0.2.0`, dieses Changelog folgt
  [Keep a Changelog](https://keepachangelog.com/de/1.1.0/), Tag `v0.2.0`.
  Ältere Abschnitte behalten ihre Datumsüberschrift — ihnen nachträglich
  Versionsnummern zu geben, würde eine Release-Historie erfinden.
- **README nach dem Workspace-Schema** (readme.so): Features · Tech Stack ·
  Installation · Usage · Running Tests · Related · Versioning · License. Die
  vorhandene Prosa ist erhalten und unter den passenden Abschnitt gewandert.
## 2026-08-13

- Die Trigger-Aliasse `rg6_control/open_gripper` und
  `rg6_control/close_gripper` entfallen — in `rg6_control` **und** im
  Simulations-Zwilling `rg6_control_sim`. Sie waren seit
  „Simplify service call names to open and close" reine Rückwärtskompatibilität
  zum damals ausgerollten Stand und hingen an derselben `handle_open_close`;
  zwei Namen für dieselbe Tat sind an der Schnittstelle nur eine Frage mehr, die
  man beim Debuggen beantworten muss. Kanonisch sind `rg6_control/open` und
  `rg6_control/close`.
  **Kein Aufrufer im Workspace war betroffen** — Profil
  (`contract/.../a200_0553.yaml`), SDK (`husky_sdk.ros.client`, dessen
  Python-Methoden `open_gripper()`/`close_gripper()` unverändert bleiben und
  schon immer `…/open` bzw. `…/close` riefen), `scripts/record.sh` und der
  Container-Mock rufen ausschliesslich die kanonischen Namen. Angepasst wurden
  nur Beschreibungen: README, `rg6_bringup.launch.py`-Docstring,
  `docs/ARCHITECTURE.md`.
  **Auf dem a200-0553 läuft bis zum nächsten Bau + Deploy von `rg6_control`
  weiterhin der Stand mit Aliassen** (Aufgabe A9 in `docs/gesamtplan.md`); der
  Systemsnapshot vom 2026-07-31 listet sie deshalb weiterhin — er ist das
  Protokoll jenes Tages und wurde nicht nachgezogen.
