# Robot Design Spec — DECODE 2025–26
**Team:** Decode Robotics  
**Date:** 2026-05-09  
**Goal:** From-scratch robot optimised for long-range ball shooting with maximum throughput  
**Status:** Design exploration — three options proposed, pending selection

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
| **R101** | Starting configuration: **18″ × 18″ × 18″** cube | Intake must fold inward to start |
| **R105A** | Horizontal expansion locked at **18″ × 18″** at all times | No intake that extends beyond chassis footprint when deployed |
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
- **AprilTag processor** — ID 20 (Blue) / ID 24 (Red) for range measurement
- Range used to set flywheel velocity and hood angle

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
│  [FW1]       [FW2]      │  ← dual flywheel (fixed)
│  ╔═════ HOOD SERVO ═════╗
│  ║ ○ ─ belt ─ ○ ─ belt ║  ← belt indexer motor
│  ║   3-BALL HORIZONTAL  ║  ← 3 balls side-by-side
│  ╚══════════════════════╝
│  [poly]               [poly]  ← polycarbonate side guides
│  ┌──── INTAKE (14″) ────┐
│  │   roller + funnel    │
│  └──────────────────────┘
│        [WEBCAM]          │
│        [PINPOINT]        │
└─────────────────────────┘
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
| Intake | 14″ roller, 1× goBILDA motor | Polycarbonate side guides funnel balls to centre |
| Belt indexer | Horizontal 3-ball belt, 1× goBILDA motor | Feeds balls one at a time into flywheel; enables burst fire |
| Flywheel | 2× goBILDA 5203 (435 RPM) with PIDF velocity control | Fixed angle — no turret |
| Hood | 1× servo | Adjustable launch angle, range-compensated via AprilTag |
| Stopper | 1× servo | Blocks ball from entering flywheel until velocity is reached |
| Gate arm | 1× servo | Side-mounted push arm |
| Webcam | USB webcam | AprilTag range measurement |
| Pinpoint | GoBILDA Pinpoint | 2-pod odometry for Pedro Pathing |

**Motor count:** 4 drive + 1 intake + 1 indexer + 2 flywheel = **8 / 8**  
**Servo count:** 1 hood + 1 stopper + 1 gate arm = **3 / 10**

### Cycle flow
```
Launch zone
    │
    ▼
Drive to balls (Pedro Pathing path)
    │
    ▼
Collect up to 3 balls into belt indexer
    │
    ▼
Auto-return to launch zone (Pedro Pathing)
    │
    ▼
Spin up flywheels → burst-fire 3 balls → repeat
```

### Software changes from current codebase
- Remove turret motor and all turret control logic
- Add belt indexer motor control (velocity-based feed timing)
- Pedro Pathing already implemented — add `returnToLaunchZone()` path
- AprilTag range → hood servo angle lookup table (replaces turret bearing calculation)
- Flywheel PIDF already tuned — reuse

### Trade-offs

| Pros | Cons |
|------|------|
| Full field coverage — can collect from anywhere | Uses all 8 motors — no spare |
| 3-ball burst = highest possible scoring rate | Cycle time depends on Pedro Pathing efficiency |
| Pedro Pathing handles return path reliably | More software complexity than A |
| No turret = simpler mechanism, lower CoG | Robot must physically rotate to face goal (handled by Pedro Pathing) |
| AprilTag range still used for hood angle | Belt indexer requires careful timing to avoid jamming |

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

## 6. Recommendation: Option B

Option B is recommended because:

1. **G416 neutralises the turret's main benefit.** Since you must be in the launch zone to shoot, the robot will always be roughly facing the goal on arrival. Robot rotation via mecanum (handled by Pedro Pathing) is effectively free — turret flexibility is wasted.

2. **The 8th motor is better spent on a belt indexer.** Three balls in a horizontal magazine, fired in a burst immediately after spin-up, gives the highest possible scoring rate. Option C fires one ball at a time from a 2-ball buffer; Option B fires 3 in rapid succession before re-entering the collection cycle.

3. **Lower centre of gravity.** No turret tower means the robot sits at ~14″ tall instead of ~20″, improving stability during high-speed mecanum driving.

4. **Simpler aiming logic.** AprilTag range → hood servo angle lookup table replaces the full turret bearing pipeline. Less to go wrong, easier to tune.

5. **Pedro Pathing already works.** The auto-return path to the launch zone is a natural extension of existing autonomous infrastructure.

---

## 7. Open Questions

- [ ] Confirm fabrication method for belt indexer (belt material, pulley size)
- [ ] Decide intake roller material (silicone, foam, or rubber compliant wheels)
- [ ] Measure actual field positions of ball clusters relative to launch zone for path planning
- [ ] Determine if a single goBILDA 5203 or 5202 is sufficient for intake torque
- [ ] Decide between 96 mm and 104 mm mecanum wheels (top speed vs torque)
- [ ] Choose flywheel wheel diameter and material for target velocity range

---

*Generated during brainstorming session on 2026-05-09*
