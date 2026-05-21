# Robot Design Spec — DECODE 2025–26
**Team:** Decode Robotics  
**Date:** 2026-05-09  
**Goal:** From-scratch robot optimised for long-range ball shooting with maximum throughput  
**Status:** Option B selected — spec updated with scattered-ball robustness additions (wide funnel + colour vision)

---

## 1. Game Context

### Primary objectives
- Score **ARTIFACT balls** (green & purple) by launching them into the open top of the **GOAL**
- Priority: **long-range shooting accuracy** + **highest possible throughput** (balls per minute)
- Secondary: operate the **GATE** to release pre-staged balls from the **RAMP**

### Critical rules that shape every design decision

| Rule | Constraint | Design impact |
|------|-----------|---------------|
| **G416** | Robot may only LAUNCH while inside a **LAUNCH ZONE** or on a LAUNCH LINE | Cannot shoot while roaming. Every cycle must return to zone. **Primary throughput bottleneck.** |
| **G408** | Max **3 artifacts controlled** simultaneously | Magazine/buffer capped at 3 balls — no need for large hoppers |
| **G418** | Cannot contact artifacts on the RAMP except by operating the GATE | Gate mechanism must be reliable — it's the only way to access pre-staged ramp balls |
| **G413** | Cannot grab/attach to field elements; gate = push lever **down** only | Gate actuator must be a simple push arm — no hooks or grippers |
| **G415 + R105** | Vertical expansion above 18″ only permitted in **final 20 seconds AND when not in a launch zone** | Robot must stay ≤ 18″ tall during all shooting and most of match play. Option B's ~14″ build height is compliant. |
| **R101** | Starting configuration: **18″ × 18″ × 18″** cube | Intake must fold inward to start |
| **R105A** | Horizontal expansion locked at **18″ × 18″** at all times — enforced **mechanically** at inspection (software limits not sufficient) | Funnel wings must have physical hinge stops preventing extension beyond 18″. Inspectors will check maximum mechanical extension. |
| **R503** | Max **8 motors, 10 servos** | 4 drive motors + 4 mechanism motors maximum |
| **R207** | No pneumatics; no high-speed airflow | Flywheels for ball manipulation are **explicitly permitted** |

### Strategic insight from G416
Since the robot **must return to a fixed launch zone to shoot**, a rotating turret adds little value — the robot is always launching from approximately the same field position. Eliminating the turret frees the 8th motor slot for a belt indexer, enabling 3-ball burst fire which directly maximises throughput.

---

## 2. Shared Foundation (all three options)

These components are common across all options:

### Drivetrain
- **4× goBILDA Yellow Jacket 5203 series** (312 RPM) on mecanum wheels
- **96 mm or 104 mm mecanum wheels** — better top speed than standard 75 mm
- **GoBILDA Pinpoint** odometry computer with two swingarm pods (forward + strafe)
- **Pedro Pathing** motion library for autonomous path following and teleop auto-return to launch zone
- Field-centric drive mode for intuitive driver control

### Vision
- **USB webcam** (640 × 480, manual exposure 2 ms, gain 100)
- **VisionPortal** hosts two simultaneous processors:
  - **AprilTagProcessor** — ID 20 (Blue) / ID 24 (Red) for goal range measurement; range → hood servo angle
  - **ColorBlobLocatorProcessor** — detects `ARTIFACT_GREEN` and `ARTIFACT_PURPLE` blobs; outputs bearing and approximate range to nearest ball for autonomous collection steering

### Gate mechanism
- **1× servo push arm** on the front or side of the chassis
- Depresses the gate lever downward only (per G413 — no closing force applied)
- Retract on a spring or second servo position

### Control hub
- **REV Control Hub** + REV Expansion Hub (for extra ports)
- 12V NiMH battery (goBILDA Nested or REV Slim)

---

## 3. Option A — Zone-Dominant (Wide Static Intake)

### Philosophy
Stay near the launch zone. Use a full-width intake to sweep nearby balls without large travel. Shoot immediately. Win by maximising time in the zone.

### Top-down layout
```
┌─────────────────────────┐
│  [FW1]      [FW2]       │  ← dual flywheel (fixed angle, rear)
│  ┌───── HOOD SERVO ─────┤
│  │                      │
│  │   3-BALL VERTICAL    │  ← magazine (stacked)
│  │      MAGAZINE        │
│  │                      │
│  └──────────────────────┤
│  ████ INTAKE ROLLER ████│  ← 18″ full-width, folds on hinge
│  ══════ FOLD HINGE ═════│
└─────────────────────────┘
         ▼ FRONT
```

### Side profile
```
         ┌────────────────┐
         │   FLYWHEEL     │  ~top
         ├────────────────┤
         │   MAGAZINE     │
         ├────────────────┤
chassis: │                │
         ├────────────────┤
         │    INTAKE      │  ~14″ total height
         └────────────────┘
    ●                      ●   (wheels)
════════════════════════════   (ground)
```

### Hardware specification

| System | Component | Notes |
|--------|-----------|-------|
| Intake | 18″ wide roller, 1× goBILDA motor | Folds flat against chassis for 18″ start config |
| Magazine | Vertical 3-ball stack, gravity-fed | Polycarbonate tube; indexer servo gates each ball |
| Flywheel | 2× goBILDA 5203 (435 RPM) | Fixed angle — no turret |
| Hood | 1× servo | Adjustable angle for range compensation |
| Gate arm | 1× servo | Side-mounted push arm |

**Motor count:** 4 drive + 1 intake + 2 flywheel = **7 / 8**  
**Servo count:** 1 hood + 1 indexer gate + 1 gate arm = **3 / 10**

### Trade-offs

| Pros | Cons |
|------|------|
| Maximum time in launch zone — fewest trips away | Cannot collect balls far from the zone |
| Simplest mechanism stack | 18″ folding intake is mechanically complex to build reliably |
| Fewest failure points | Depends on balls accumulating nearby (or partner pushing them close) |
| 7/8 motors leaves one spare for future use | Wide intake may struggle to funnel balls into narrow magazine |

---

## 4. Option B — Burst Collector ★ Recommended

### Philosophy
Drive anywhere on the field to collect a full 3-ball buffer, auto-path back to the launch zone with Pedro Pathing, fire a 3-ball burst, repeat. No turret — the robot faces the goal on arrival. The 8th motor drives the belt indexer for burst fire.

### Top-down layout
```
┌─────────────────────────┐
│  [FW1]       [FW2]      │  ← dual flywheel (fixed angle, rear)
│  ╔═════ HOOD SERVO ═════╗
│  ║ ○ ─ belt ─ ○ ─ belt ║  ← belt indexer motor
│  ║   3-BALL HORIZONTAL  ║  ← 3 balls side-by-side
│  ╚══════════════════════╝
│  [poly]               [poly]  ← straight polycarbonate side guides
│  ┌──── INTAKE (14″) ────┐
│  │   roller + funnel    │
╲  └──────────────────────┘  ╱  ← angled polycarb funnel wings
 ╲      [WEBCAM]            ╱     (widens mouth to ~18″)
  ╲     [PINPOINT]         ╱
   ╚═════════════════════╝       ← ~18″ funnel mouth at floor
           ▼ FRONT
```

### Side profile
```
         ┌────────────────┐
         │   FLYWHEEL     │
         ├────────────────┤
         │  BELT INDEXER  │   ←──── ball path ────
         ├────────────────┤
chassis: │                │
         ├────────────────┤
         │    INTAKE      │  ~14″ total height
         └────────────────┘
    ●                      ●
════════════════════════════
```

### Hardware specification

| System | Component | Notes |
|--------|-----------|-------|
| Intake roller | 14″ roller, 1× goBILDA motor | Silicone or foam compliant wheels for grip |
| Intake funnel | Angled polycarbonate wings (custom-cut, semi-custom) | Creates ~18″ wide mouth; balls deflect inward to roller. Hinged to fold within 18″ starting cube. Stays within R105A horizontal limit when deployed. |
| Belt indexer | Horizontal 3-ball belt, 1× goBILDA motor | Feeds balls one at a time into flywheel; enables burst fire |
| Flywheel | 2× goBILDA 5203 (435 RPM) with PIDF velocity control | Fixed angle — no turret |
| Hood | 1× servo | Adjustable launch angle, range-compensated via AprilTag |
| Stopper | 1× servo | Blocks ball from entering flywheel until velocity is reached |
| Gate arm | 1× servo | Side-mounted push arm — depresses gate lever only (G413) |
| Webcam | USB webcam, 640 × 480 | Dual pipeline: AprilTag (goal range) + ColorBlob (ball detection) |
| Pinpoint | GoBILDA Pinpoint | 2-pod odometry for Pedro Pathing |

**Motor count:** 4 drive + 1 intake + 1 indexer + 2 flywheel = **8 / 8**  
**Servo count:** 1 hood + 1 stopper + 1 gate arm = **3 / 10**

### Cycle flow

```
Launch zone
    │
    ▼
COLLECTION PHASE (vision-guided)
    │  1. ColorBlobLocator detects nearest ARTIFACT_GREEN / ARTIFACT_PURPLE
    │  2. Robot steers toward bearing (field-centric mecanum)
    │  3. Intake roller running — wide funnel guides ball to roller
    │  4. Belt indexer counts ball; advances one position
    │  5. If indexer count < 3 → seek next ball (step 1)
    │  6. If indexer count = 3 → exit collection phase
    │
    ▼
RETURN PHASE
    │  Pedro Pathing path → launch zone
    │  Robot rotates to face goal on arrival
    │  Flywheels spin up during transit (pre-warm)
    │
    ▼
SHOOT PHASE
    │  AprilTag range → set hood servo angle + flywheel target velocity
    │  Stopper releases ball 1 → wait for velocity recovery → ball 2 → ball 3
    │
    ▼
Repeat
```

### Scattered ball robustness

When balls are spread unpredictably across the field (e.g. after gate release or contact play), two additions keep throughput high:

#### Addition 1 — Wide polycarbonate intake funnel (hardware)

The straight 14″ side guides are replaced with angled polycarbonate wings that create an **~18″ wide funnel mouth** at floor level, narrowing to the 14″ roller.

- A ball anywhere within a 9″ radius of the robot centreline is automatically captured as the robot drives forward — no precise driver alignment needed.
- The funnel wings are hinged and fold flat against the chassis face for the 18″ starting configuration (R101).
- At maximum deployment the funnel stays within the 18″ × 18″ horizontal expansion limit (R105A) — the wings angle *inward*, not outward.
- Material: 3 mm polycarbonate sheet, custom-cut (semi-custom fabrication).

```
Side guide geometry (top-down):

  far left        centre         far right
      \    14″ roller    /
       \  ┌──────────┐  /
        \ │  INTAKE  │ /
    ~9″  \│  ROLLER  │/  ~9″    ← funnel catches balls
          └──────────┘             within 9″ either side
       ←── ~18″ total ──→
```

#### Addition 2 — Colour-vision ball detection (software only, no new hardware)

The FTC 11.0 SDK's `VisionPortal` supports multiple processors on a single camera. A `ColorBlobLocatorProcessor` is added alongside the existing `AprilTagProcessor`.

- Configured for `ARTIFACT_GREEN` and `ARTIFACT_PURPLE` HSV ranges.
- Outputs bearing (degrees) and approximate range (pixels → inches via calibration) to the largest detected blob.
- During the collection phase, the robot steers toward the bearing using field-centric mecanum while the intake roller runs.
- Switches back to AprilTag-only mode once 3 balls are collected and the robot is returning to the launch zone.
- No extra hardware. No extra motor. Zero impact on the motor/servo budget.

| Ball location | Without additions | With funnel + vision |
|---------------|-------------------|----------------------|
| Near launch zone | Good | Excellent |
| Mid-field, scattered | Driver must manually align | Auto-detected, auto-collected |
| Far field / unpredictable | Long cycle time | Autonomously seeks nearest ball |
| Just released from ramp gate | Driver reactive | Vision detects immediately |

### Software changes from current codebase

| Change | Effort | Notes |
|--------|--------|-------|
| Remove turret motor + control logic | Low | Delete `turret` `hardwareMap.get`, turret PIDF, bearing pipeline |
| Add belt indexer motor control | Medium | Velocity-timed feed; ball-count via encoder position or IR sensor |
| Add `returnToLaunchZone()` Pedro path | Low | Pedro Pathing already works — new named path only |
| AprilTag range → hood angle lookup table | Low | Replaces turret bearing maths; table tuned empirically |
| Add `ColorBlobLocatorProcessor` to VisionPortal | Low | SDK built-in; configure HSV ranges for green/purple artifacts |
| Collection steering loop (vision-guided drive) | Medium | Field-centric bearing steer while intake runs; exit on count = 3 |
| Flywheel PIDF | None | Already tuned — reuse as-is |

### Trade-offs

| Pros | Cons |
|------|------|
| Full field coverage — collects balls anywhere | Uses all 8 motors — no spare |
| Wide funnel captures balls without precise alignment | Belt indexer requires careful timing to avoid jamming |
| Colour vision auto-seeks balls — no driver hunting | More software components than Option A |
| 3-ball burst = highest possible scoring rate | Cycle time still depends on how far balls are from zone |
| Pedro Pathing handles return path reliably | Funnel wings need robust hinge for repeated folding |
| No turret = simpler mechanism, lower CoG | Dual vision pipeline adds ~5 ms processing latency |
| AprilTag range still used for hood angle | |

---

## 5. Option C — Turret Shooter (Current Concept Refined)

### Philosophy
Keep a rotating turret for flexible aiming. Prioritise long-range accuracy. Robot does not need to face the goal — the turret handles aiming from any robot orientation.

### Top-down layout
```
┌─────────────────────────┐
│    ╭── turret sweep ──╮  │
│    │  ╭──────────╮   │  │
│    │  │ [FW1][FW2│   │  │  ← flywheel on turret platform
│    │  │  TURRET  │   │  │
│    │  ╰──────────╯   │  │
│    ╰─────────────────╯  │
│  ┌──── 2-BALL BUFFER ───┤
│  └──────────────────────┤
│   ┌── INTAKE (~12″) ───┐ │
│   └────────────────────┘ │
│         [WEBCAM]         │
└─────────────────────────┘
         ▼ FRONT
```

### Side profile
```
    ┌──────────┐
    │ FLYWHEEL │   ← on turret, ~20″ height
    ├──────────┤
    │  TURRET  │   ← rotating platform (↻)
    ╔══════════╗
    ║  BUFFER  ║
    ╠══════════╣
    ║  chassis ║
    ╠══════════╣
    ║  INTAKE  ║  ~20″ total height
    ╚══════════╝
●               ●
═════════════════
```

### Hardware specification

| System | Component | Notes |
|--------|-----------|-------|
| Intake | 12″ roller, 1× goBILDA motor | Narrower — turret footprint limits intake width |
| Buffer | 2-ball stopper-gate buffer | Same as current — stopper servo blocks/releases |
| Turret | 1× goBILDA motor, ~340° sweep | RUN_TO_POSITION; encoder ticks → bearing conversion |
| Flywheel | 2× goBILDA 5203 (435 RPM) | Mounted on turret platform |
| Hood/flap | 1× servo | Launch angle adjustment |
| Stopper | 1× servo | Ball gate |
| Gate arm | 1× servo | Side push arm |
| Webcam | USB webcam | AprilTag bearing + range |

**Motor count:** 4 drive + 1 intake + 1 turret + 2 flywheel = **8 / 8** (no indexer — turret uses slot 8)  
**Servo count:** 1 hood + 1 stopper + 1 gate arm = **3 / 10**

### Trade-offs

| Pros | Cons |
|------|------|
| No robot rotation required to aim | Turret adds height (~20″ vs ~14″) — higher centre of gravity |
| AprilTag auto-aim code already written and tuned | Turret uses 8th motor slot — cannot add belt indexer |
| Most accurate at varying long-range distances | 2-ball buffer means slower burst rate than B |
| Robot orientation doesn't affect shooting | Turret mechanical complexity — slip ring or cable management needed |
| Easiest code migration from current robot | G416 makes turret freedom less valuable (fixed launch zone) |

---

## 6. Recommendation: Option B (with funnel + colour vision)

Option B is recommended because:

1. **G416 neutralises the turret's main benefit.** Since you must be in the launch zone to shoot, the robot will always be roughly facing the goal on arrival. Robot rotation via mecanum (handled by Pedro Pathing) is effectively free — turret flexibility is wasted.

2. **The 8th motor is better spent on a belt indexer.** Three balls in a horizontal magazine, fired in a burst immediately after spin-up, gives the highest possible scoring rate. Option C fires one ball at a time from a 2-ball buffer; Option B fires 3 in rapid succession before re-entering the collection cycle.

3. **Lower centre of gravity.** No turret tower means the robot sits at ~14″ tall instead of ~20″, improving stability during high-speed mecanum driving.

4. **Simpler aiming logic.** AprilTag range → hood servo angle lookup table replaces the full turret bearing pipeline. Less to go wrong, easier to tune.

5. **Pedro Pathing already works.** The auto-return path to the launch zone is a natural extension of existing autonomous infrastructure.

6. **Wide funnel handles real-game ball scatter.** In practice, balls spread unpredictably after gate releases and contact play. The angled polycarbonate funnel wings catch balls within a 9″ radius of the centreline — the robot does not need precise driver alignment.

7. **Colour vision eliminates driver search time.** `ColorBlobLocatorProcessor` (FTC 11.0 SDK, no extra hardware) autonomously steers toward the nearest artifact during the collection phase. This is the single largest remaining throughput gain available without adding hardware.

---

## 7. Open Questions

**Mechanical**
- [ ] Confirm belt indexer material and pulley size (GT2 belt + 20T pulleys is a common starting point)
- [ ] Decide intake roller material — silicone for grip, foam for compliance with irregular balls
- [ ] Determine polycarbonate funnel wing angle — steeper = more deflection, shallower = less jamming risk
- [ ] Confirm funnel hinge mechanism folds reliably within 18″ starting cube (R101)
- [ ] Add physical hinge stops to funnel wings so maximum mechanical extension cannot exceed 18″ — required for R105A inspection (software limits alone are not accepted by inspectors)
- [ ] Deburr and round all polycarbonate funnel wing edges — sharp edges contacting balls violate R206
- [ ] Verify total robot height ≤ 18″ with all mechanisms deployed in shooting position (G415 — expansion above 18″ prohibited while in launch zone)
- [ ] Determine if a goBILDA 5203 or 5202 is sufficient for intake roller torque at ball size/weight
- [ ] Decide between 96 mm and 104 mm mecanum wheels (top speed vs torque trade-off)
- [ ] Choose flywheel wheel diameter and material for target velocity range

**Software**
- [ ] Implement hard ball-count stop at 3 in belt indexer (G408 requires robot cannot inadvertently control 4+ artifacts — use IR sensor or encoder position; indexer must refuse a 4th ball mechanically or by stopping the belt)
- [ ] Tune HSV ranges for `ColorBlobLocatorProcessor` on actual ARTIFACT_GREEN and ARTIFACT_PURPLE balls under event lighting
- [ ] Validate dual VisionPortal pipeline does not drop below acceptable frame rate (~15 fps minimum)
- [ ] Measure field ball cluster positions for Pedro Pathing collection waypoints
- [ ] Tune belt indexer timing (ms per ball advance) to prevent jamming and double-feed

---

*Generated during brainstorming session on 2026-05-09*
