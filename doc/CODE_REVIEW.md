# DQP2025 Code Review — Main Branch
**Date:** 2026-05-02

This document reviews the robot code on the `main` branch. Each finding explains **what the problem is**, **why it matters** for competition performance or reliability, and **exactly how to fix it**. Findings are grouped by severity.

---

## Part 1 — Build-Breaking Bugs
These must be fixed before the project will compile into an APK. Until they are fixed, **no code can be deployed to the robot at all** — not even code that has nothing to do with these bugs.

---

### 1. `Constants.createFollower()` does not exist
**Files:** `RedClose15.java:192`, `FlywheelTesting.java:106`

#### What is happening
When Java compiles your code, it reads every line and checks that every method call refers to something that actually exists. `RedClose15.java` line 192 calls:
```java
follower = Constants.createFollower(hardwareMap);
```
But if you open `Constants.java`, you will see only two methods defined:
```java
public static Follower createAutoFollower(HardwareMap hardwareMap) { ... }
public static Follower createTeleopFollower(HardwareMap hardwareMap) { ... }
```
There is no `createFollower`. Java sees a call to a method that doesn't exist and refuses to compile. The same issue is in `FlywheelTesting.java:106`.

#### Why it happened
The Pedro Pathing documentation shows `createFollower` as the standard method name. At some point `Constants.java` was updated to use more descriptive names (`createAutoFollower` / `createTeleopFollower`), but `RedClose15.java` and `FlywheelTesting.java` were never updated to match.

#### Fix
Add a `createFollower` method to `Constants.java` that calls `createAutoFollower`:
```java
public static Follower createFollower(HardwareMap hardwareMap) {
    return createAutoFollower(hardwareMap);
}
```
This is one line of code. It makes `createFollower` a valid method name again without changing anything else.

---

### 2. `GoalPos` API mismatch in Gate autos
**Files:** `BlueCloseGate.java:47`, `RedCloseGate.java:48`, `RedFarGate.java:50`, `BlueFarGate.java:48`

#### What is happening
The `GoalPos` class tracks where the goal is on the field. It was recently updated to track the goal in three dimensions (X, Y, and Z — where Z is the height of the goal). Before the update, `GoalPos` only tracked X and Y.

The four Gate autonomous files (`BlueCloseGate`, `RedCloseGate`, `RedFarGate`, `BlueFarGate`) were never updated after this change. They still use the old 2-argument constructor:
```java
GoalPos goalPos = new GoalPos(147, 144);  // OLD — only X and Y
```
But `GoalPos.java` now only has a 3-argument constructor:
```java
public GoalPos(double X, double Y, double Z) { ... }  // CURRENT
```
Java sees `new GoalPos(147, 144)` — two arguments where three are required — and refuses to compile.

The same files also call the old version of the `update()` method with 4 arguments:
```java
goalPos.update(xPos, yPos, bearing, camRange);  // OLD — 4 args
```
But `GoalPos.update()` now requires 6 arguments:
```java
public void update(double alpha, double x, double y, double heading, double elevation, double dist)
```
This is a second compile error in the same files.

#### Fix
Update all four Gate auto files. Change the constructor:
```java
GoalPos goalPos = new GoalPos(147, 144, 15.5);  // add Z height (inches)
```
Change the update call in `aiming()`:
```java
goalPos.update(0.08, xPos, yPos, bearing, elevation, camRange);
```
Note: `elevation` must also be captured from the detection — look at how `RedClose15.java` does it as a reference.

---

### 3. `BlueCloseGate` — flywheel motor mode not set before applying PIDF
**File:** `BlueCloseGate.java:186`

#### What is happening
The flywheel motor uses a **PIDF controller** — a set of four numbers (P, I, D, F) that tell the motor controller how aggressively to maintain a target speed. Applying PIDF values to a motor requires the motor to first be in `RUN_USING_ENCODER` mode, because that is the mode where the motor controller measures actual speed and uses the PIDF to correct it.

In `RedCloseGate.java`, the initialization is done correctly — mode is set first, then PIDF:
```java
flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);       // ✓ step 1
flyWheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, fwPID); // ✓ step 2
```
In `BlueCloseGate.java:186`, the mode-setting line is missing:
```java
// flyWheel1.setMode(...) is MISSING
flyWheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, fwPID); // applied with no effect
```
The FTC SDK **silently ignores** the `setPIDFCoefficients` call if the motor is not already in `RUN_USING_ENCODER` mode. No error is thrown — the code runs, but the PIDF values you tuned are never actually applied. The flywheel runs in **open-loop** mode, meaning the motor controller is just applying raw power with no feedback, and the flywheel speed will vary depending on battery voltage.

#### Why this matters
Your `toFWV()` function calculates the exact flywheel speed needed to reach the goal at a given distance. If the PIDF is silently ignored, the flywheel never reaches that precise speed reliably. Shots on the blue side will be inconsistent in power, even if the targeting angle is correct.

#### Fix
Add one line before `setPIDFCoefficients` in `BlueCloseGate.init()`:
```java
flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);  // add this
flyWheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, fwPID);
```

---

### 4. `BlueCloseGate` — second flywheel motor missing
**File:** `BlueCloseGate.java`

#### What is happening
The robot has two flywheel motors (`FW1` and `FW2`). `RedCloseGate.java` initializes and drives both:
```java
flyWheel1 = hardwareMap.get(DcMotorEx.class, "FW1");
flyWheel2 = hardwareMap.get(DcMotorEx.class, "FW2");
// ...
flyWheel1.setVelocity(targetV);
flyWheel2.setVelocity(targetV);
```
`BlueCloseGate.java` only initializes `FW1`. `FW2` is never referenced. If both motors spin the same flywheel wheel (one on each side), then only half the motors are contributing to spin-up. The flywheel will spin up more slowly and may not reach the target velocity before the stopper opens, causing underpowered shots.

#### Fix
Copy the `flyWheel2` initialization from `RedCloseGate.java` into `BlueCloseGate.init()`, and add `flyWheel2.setVelocity(targetV)` and `flyWheel2.setVelocity(0)` in the same places `flyWheel1` is used in `shoot()`.

---

### 5. `toFWV()` ignores its own input parameter
**Files:** `RedCloseGate.java:431`, `BlueCloseGate.java:423`, `BlueClose.java:403`, `RedClose.java:401`

#### What is happening
`toFWV` is a function that converts a distance (range) into the flywheel velocity needed to reach that distance. It takes one argument named `r` (for range). But look at the function body:

```java
public double toFWV(double r) {
    return (0.00673 * range * range) + (5.54 * range) + (1162);
    //                  ^^^^^ uses the field, not the parameter r
}
```

The parameter `r` is completely ignored. The calculation uses `range` — the class field — instead. This is a programming mistake where a variable is declared but never used, and a different variable with a similar name is used instead.

#### Why it "works" right now but is still a bug
Right now, every call to `toFWV` looks like this:
```java
double targetV = toFWV(range);
```
The argument passed in (`range`) happens to be the same value as the field (`range`), so the result is the same either way. The bug is invisible in practice.

But it is still a bug because:
1. It misleads anyone reading the code — the signature says it takes a range as input, but it doesn't actually use it
2. If anyone ever calls `toFWV` with a different argument (for example, `toFWV(estimatedRange)` or `toFWV(40)` during testing), the function will silently ignore the input and give a wrong answer with no error or warning

#### Fix
Change `range` to `r` inside the function body in all four files:
```java
public double toFWV(double r) {
    return (0.00673 * r * r) + (5.54 * r) + (1162);
}
```

---

### 6. `OfficialRedTeleop.java` and `OfficialBlueTeleop.java` are empty
**Files:** `TeleOp/OfficialRedTeleop.java`, `TeleOp/OfficialBlueTeleop.java`

Both files exist but contain only a single blank line. These are meant to be the competition TeleOp modes based on their names and location. They must be implemented before any competition match.

---

## Part 2 — High Impact Performance
These bugs do not stop the code from compiling, but they cost measurable time during the 30-second autonomous period.

---

### 7. Flywheel spin-up skipped before one shoot cycle
**File:** `RedClose15.java`

#### Background — how `move()` with `idle=true` works
Your `move()` method has an optional `idle` parameter:
```java
public void move(PathChain path, PathState nextPath, boolean idle) {
    if (!moving) {
        if (idle) {
            flyWheel1.setVelocity(1100);  // spin up flywheel while driving
            flyWheel2.setVelocity(1100);
        }
        follower.followPath(path, false);
        moving = true;
    }
    ...
}
```
When `idle=true`, the flywheel starts spinning **while the robot is driving** to the scoring position. This is smart — by the time the robot arrives, the flywheel is already at or near the target speed, and the `shoot()` method can open the stopper almost immediately.

When `idle` is `false` (the default), the flywheel does not spin up during the drive. The robot arrives at the scoring position and the flywheel starts from zero. The `shoot()` method then has to wait until the flywheel reaches target speed before opening the stopper. This wait is typically **300–600ms**.

#### The problem
Looking at `RedClose15`'s state machine, `idle=true` is applied on most outtake moves, but one is missing:
```
OUTTAKE1 → SHOOT1    move(Outtake1, PathState.SHOOT1, true)   ✓ spinning during drive
OUTTAKEB → SHOOTB    move(OuttakeB, PathState.SHOOTB, true)   ✓ spinning during drive
OUTTAKE2 → SHOOT2    move(Outtake2, PathState.SHOOT2)         ✗ NOT spinning — costs ~500ms
OUTTAKE3 → SHOOT3    move(Outtake3, PathState.SHOOT3, true)   ✓ spinning during drive
```
The third shoot cycle wastes up to 500ms waiting for the flywheel to spin up after arrival.

#### Fix
Change line in `RedClose15.java`:
```java
move(Outtake2, PathState.SHOOT2);        // current
move(Outtake2, PathState.SHOOT2, true);  // fix
```

---

### 8. Shoot timeout is a fixed countdown rather than detecting when the ball has left
**Files:** All autos — `RedClose15`: 1600ms timeout, Gate autos: 2800ms timeout

#### How the current shooting sequence works
```java
public void shoot(PathState nextPath) {
    flyWheel1.setVelocity(targetV);

    if (flyWheel1.getVelocity() >= targetV) {
        stopper.setPosition(0.973);  // open — ball enters flywheel
        intake.setPower(1);          // push ball toward flywheel
    }

    if (actionTimer.getElapsedTime() > 1600) {  // wait 1.6 seconds total
        intake.setPower(0);
        stopper.setPosition(0.9);    // close
        flyWheel1.setVelocity(0);
        pathState = nextPath;        // move on
    }
}
```
The stopper opens as soon as the flywheel is at speed. The ball travels through the flywheel and is launched. The entire ball-in-flywheel-to-ball-launched process takes well under 500ms. But the code always waits the full 1600ms (or 2800ms in Gate autos) before moving on, regardless of whether the ball has already left.

#### Why this wastes time
With 4 shoot states per auto (`SHOOTPRE`, `SHOOT1`, `SHOOT2`, `SHOOT3`), and each one waiting the full timeout:
- `RedClose15`: 4 × 1600ms = **6.4 seconds** spent in shoot states
- Gate autos: 4 × 2800ms = **11.2 seconds** spent in shoot states

Out of a 30-second autonomous, 6–11 seconds is a huge fraction. If the ball is gone in 500ms but the code waits 1600ms, you are wasting **over 1 second per shot** sitting still doing nothing.

#### Better approach — detect when the ball has left
When the ball passes through the flywheel, the flywheel velocity dips briefly (it takes energy to launch the ball), then recovers. You can detect this recovery as evidence the ball has gone:

```java
boolean flywheelRecovered = flyWheel1.getVelocity() > targetV * 0.98;
boolean stopperIsOpen = stopper.getPosition() > 0.97;
boolean minimumTimeElapsed = actionTimer.getElapsedTime() > 300;

if (stopperIsOpen && flywheelRecovered && minimumTimeElapsed) {
    // Ball has passed through — move on early
    intake.setPower(0);
    stopper.setPosition(0.9);
    flyWheel1.setVelocity(0);
    pathState = nextPath;
    actionTimer.resetTimer();
}
```
Keep the 1600ms hard timeout as a fallback in case the velocity sensor misses the dip, but the above condition will trigger much sooner in normal operation.

---

### 9. Two-step intake paths are two separate states when they could be one
**Files:** All autos — `Intake21 → Intake22`, `Intake31 → Intake32`

#### What is happening
Every auto approaches each row of field pieces in two moves. For example in `RedClose15`:
- `INTAKE31`: Drive from outtake position to the start of the row (approach)
- `INTAKE32`: Drive slowly along the row while running the intake (collect)

These are two separate `PathChain` objects and two separate states in the FSM. Between them, the state machine has to:
1. Detect that `Intake31` is finished (`!follower.isBusy()`)
2. Wait 50ms (`actionTimer.getElapsedTime() > 50`)
3. Transition to `INTAKE32` state
4. Start `Intake32` on the next loop iteration

That transition takes at least **50ms plus one loop cycle** (~33ms at 30Hz) = roughly **80ms per two-step intake**. With 2 two-step intakes per auto cycle and typically 3 cycles, that's ~480ms wasted in transitions alone.

More importantly, Pedro Pathing plans deceleration path-by-path. When `Intake31` ends and `Intake32` starts, Pedro treats it as two separate journeys — it decelerates to a stop at the end of `Intake31`, then accelerates again for `Intake32`. A single multi-segment `PathChain` would flow smoothly through the intermediate point without stopping.

#### Fix
Pedro Pathing's `pathBuilder()` can chain multiple path segments together:
```java
// Current: two separate paths
Intake31 = follower.pathBuilder()
    .addPath(new BezierLine(outtake, intake3p1))
    .setConstantHeadingInterpolation(0)
    .build();
Intake32 = follower.pathBuilder()
    .addPath(new BezierLine(intake3p1, intake3p2))
    .setConstantHeadingInterpolation(0)
    .build();

// Fix: one path with two segments
Intake3 = follower.pathBuilder()
    .addPath(new BezierLine(outtake, intake3p1))      // approach
    .addPath(new BezierLine(intake3p1, intake3p2))    // collect
    .setConstantHeadingInterpolation(0)
    .setGlobalDeceleration(0.9)
    .build();
```
Also remove the `INTAKE31` state and rename `INTAKE32` to `INTAKE3` in the enum and switch statement.

---

### 10. `setGlobalDeceleration()` is never used
**Files:** All autos

#### What is Global Deceleration?
By default, Pedro Pathing only applies braking at the **very end** of the last path in a chain. For a single-path chain, this means the robot brakes sharply at the very last moment.

Global Deceleration changes this so that braking is spread over the **entire PathChain**, starting earlier and more smoothly. The Pedro Pathing documentation recommends it for all PathChains:

> *"This mode is recommended for use globally, even when path chains are only one path, because it is more optimized than the default mode."*

The `brakingStrength` parameter controls how hard the braking is — lower values mean the robot carries speed further into the path before decelerating. You can tune this per path to find the fastest setting that doesn't cause overshoot.

#### Fix
Add `.setGlobalDeceleration(brakingStrength)` to every `pathBuilder()` call. Start with `0.9` and tune lower to go faster:
```java
Outtake1 = follower.pathBuilder()
    .addPath(new BezierLine(intake1p2, outtake))
    .setConstantHeadingInterpolation(0)
    .setGlobalDeceleration(0.9)  // add this
    .build();
```

---

### 11. `RedCloseGate` — robot does not hold position at scoring location
**File:** `RedCloseGate.java:303`

#### What `holdEnd` does
`follower.followPath(path, holdEnd)` — the second argument tells Pedro Pathing what to do when the path finishes. 
- `true`: Pedro keeps applying motor corrections to hold the robot at the end position, fighting any drift
- `false`: Pedro stops controlling the motors; the robot coasts to a stop wherever it ends up

`RedCloseGate.move()` uses `false`:
```java
follower.followPath(path, false);  // robot drifts after path ends
```
`BlueCloseGate` and `RedClose15` use `true`:
```java
follower.followPath(path, true);   // robot held in place
```

#### Why this matters for shooting
During `SHOOT1`, `SHOOT2`, `SHOOT3`, the robot needs to be perfectly stationary while the turret aims and the ball is launched. Even a slow 1-inch drift in 2 seconds can change the `range` calculation enough to affect the shot trajectory.

With `holdEnd=false` in `RedCloseGate`, the robot arrives at the outtake position, Pedro stops motor corrections, and the robot slowly drifts on the carpet. The turret is targeting a moving robot at a static goal — the aiming error grows the longer the shoot state runs.

#### Fix
Change `false` to `true` in `RedCloseGate.move()`:
```java
follower.followPath(path, true);  // was false
```

---

### 12. PathConstraints are set so loosely that one constraint never activates
**File:** `Constants.java:69`

#### What PathConstraints do
Pedro Pathing uses five criteria to decide when a path is "done" (`follower.isBusy()` returns false). Your `PathConstraints` set four of them:
```java
new PathConstraints(0.99, 100, 1, 1)
//                  tVal  vel  trans  heading
```
- **tVal = 0.99**: 99% of the path must be completed ✓ reasonable
- **velocity = 100 in/s**: robot must be moving slower than 100 inches/second ← **problem**
- **translational = 1 inch**: robot must be within 1 inch of the path end ✓ workable
- **heading = 1 radian**: robot heading must be within ~57 degrees ← **very loose**

The robot's maximum forward speed is approximately **85 inches/second** (from `xVelocity(85.3)` in `Constants.java`). The velocity constraint is set to 100 in/s — a value the robot can **never reach**. So this constraint is permanently satisfied from the moment the path starts, meaning it contributes nothing to ensuring the path is actually complete.

Your FSM relies on the 50ms `actionTimer` guard as the real completion check, not these constraints. The robot may still be moving when the 50ms expires.

#### Fix
Tighten the constraints to values the robot must actually achieve before proceeding:
```java
new PathConstraints(0.99, 0.5, 0.3, 0.01)
// velocity < 0.5 in/s (nearly stopped), within 0.3 inches, within 0.01 radians heading
```
Test carefully after this change — if too tight, paths may never complete.

---

## Part 3 — Robustness
These are bugs that may not affect every match but will cause failures at the worst possible time — in competition.

---

### 13. Servos do not have a defined starting position
**Files:** All autos — `init()` method

#### What is happening
When a servo is connected to the Control Hub, it moves to whatever position it was last commanded to — it remembers its position between OpMode runs. Your `init()` method sets the servo direction but never sets an initial position:
```java
stopper = hardwareMap.get(Servo.class, "stopper");
stopper.setDirection(Servo.Direction.FORWARD);
// ← stopper.setPosition() is never called here
```
If the previous match ended mid-shot with the stopper open (`0.973`), the stopper will still be at `0.973` — **open** — when the next auto begins. During the `PRELOAD` drive, the stopper is open and any pieces loaded into the robot before the match will fall out.

The same issue applies to the flap servo — if it was left at `0.24` (maximum angle for long range) from the previous match, it stays there for the start of the next match regardless of the actual starting range.

#### Fix
Add `setPosition()` calls immediately after the servo is retrieved in every auto's `init()`:
```java
stopper = hardwareMap.get(Servo.class, "stopper");
stopper.setDirection(Servo.Direction.FORWARD);
stopper.setPosition(0.9);   // closed — keep pieces in robot during preload drive

flap = hardwareMap.get(Servo.class, "flap");
flap.setDirection(Servo.Direction.FORWARD);
flap.setPosition(0.2);      // default mid-range angle
```

---

### 14. Stopper left open if match ends mid-shot
**Files:** All autos — `stop()` method

#### What is happening
`RedClose15.stop()` only closes the vision portal:
```java
public void stop() {
    if (visionPortal != null) {
        visionPortal.close();
        visionPortal = null;
    }
    // stopper position is not reset
}
```
If the 30-second autonomous ends while the robot is in the middle of a `shoot()` state with the stopper open (`0.973`), the stopper stays open until the next OpMode is run. Between the end of autonomous and the start of TeleOp (~10 seconds), pieces could fall out. More importantly, if the robot is inspected or handled by a team member between rounds, the open stopper is a safety consideration.

#### Fix
Add `stopper.setPosition(0.9)` to every auto's `stop()` method:
```java
public void stop() {
    stopper.setPosition(0.9);  // always close on stop
    if (visionPortal != null) {
        visionPortal.close();
        visionPortal = null;
    }
}
```

---

### 15. `BlueCloseGate` — turret tracks using a stale distance value
**File:** `BlueCloseGate.java — `aiming()` method`

#### Background — what `range` is used for
`range` is the calculated distance from the robot's current position to the goal. It is used in two ways:
1. In `aiming()` — to correct for the physical offset between the camera and the turret axis (`Math.atan(2.5/range)`)
2. In `shoot()` — to set the flywheel speed (`toFWV(range)`) and flap angle

For these to be accurate, `range` needs to be updated to the robot's *current* position on every loop.

#### The problem
`RedCloseGate.aiming()` correctly updates `range` at the start of the method:
```java
public void aiming(List<AprilTagDetection> detectedTags) {
    range = goalPos.findRange(xPos, yPos);  // ✓ updated every loop
    ...
}
```
`BlueCloseGate.aiming()` does **not** update `range`:
```java
public void aiming(List<AprilTagDetection> detectedTags) {
    // range is NOT updated here
    if (gainSet) {
        for (AprilTagDetection detection : detectedTags) {
            bearing = detection.ftcPose.bearing - Math.toDegrees(Math.atan(2.5/range));
            //                                                              ^^^^ stale!
```
`range` is only updated inside `shoot()`:
```java
public void shoot(PathState nextPath) {
    range = goalPos.findRange(xPos, yPos);  // updated here, but only during shooting
```
During path-following states (`INTAKE`, `OUTTAKE`, `OPENGATE`), `range` holds whatever value it had from the last time `shoot()` ran — which could be from a completely different position. The bearing correction (`Math.atan(2.5/range)`) is therefore wrong, causing the turret to point at a slightly incorrect angle during transit.

#### Fix
Add `range = goalPos.findRange(xPos, yPos)` as the first line of `BlueCloseGate.aiming()`, matching `RedCloseGate`.

---

### 16. The first shot may fire before the camera has locked on
**Files:** All autos

#### The problem in detail
When the autonomous starts, the webcam on the Control Hub goes through a boot sequence:
1. `OPENING_CAMERA_DEVICE` — USB connection is being established
2. `STARTING_STREAM` — camera is initializing
3. `STREAMING` — frames are available for processing

This takes **1–3 seconds** depending on the camera and Control Hub load. Until the camera reaches `STREAMING` state and your `cameraControls()` method successfully applies manual exposure settings, `gainSet` remains `false`.

In `aiming()`, all tag detection is skipped while `gainSet` is false:
```java
if (gainSet) {  // skip everything until camera is ready
    for (AprilTagDetection detection : detectedTags) {
        // ... update goalPos
    }
}
```
Meanwhile, the turret still runs — it just aims at the hardcoded starting estimate:
```java
GoalPos goalPos = new GoalPos(147, 143, 15.5);  // initial guess
```
For `RedClose15`, the preload path from `(120, 133)` to `(93, 90)` takes approximately **1.5–2 seconds**. If the camera takes 2 seconds to warm up, the robot may arrive at the outtake position and immediately enter `SHOOTPRE` with zero vision corrections having been applied. The turret is pointing at the hardcoded estimate, not the measured goal position.

If the initial `GoalPos` estimate is off by 3–4 inches (a reasonable positioning error when placing the robot), the turret could be several degrees wrong on the first shot.

#### Fix
Before transitioning out of `PRELOAD`, add a guard that waits for vision lock:
```java
case PRELOAD:
    move(Preload, PathState.SHOOTPRE, true);
    // Only transition if camera is ready OR 3 seconds have elapsed as a safety timeout
    if (pathState == PathState.SHOOTPRE && !gainSet && opmodeTimer.getElapsedTimeSeconds() < 3.0) {
        pathState = PathState.PRELOAD;  // wait here until camera warms up
    }
    break;
```

---

### 17. Turret angle wrapping works differently between blue and red
**Files:** `BlueCloseGate.aiming()` vs `RedCloseGate.aiming()`

#### What angle wrapping is and why it matters
The turret can physically rotate from -990 to +850 encoder ticks (red side) or 0 to 1865 ticks (blue side). The `goalPos.findAngle()` method returns an angle in degrees. This angle needs to be mapped to encoder ticks and kept within the hardware limits.

The problem is that angles "wrap around" — 181 degrees and -179 degrees represent nearly the same direction, but if the code doesn't handle this carefully, it may command the turret to rotate nearly all the way around the long way instead of making a small correction.

`BlueCloseGate` uses **0-to-360 degree** wrapping:
```java
if (turretTarget > 360 + 30) turretTarget -= 360;  // blue
else if (turretTarget < 0 - 30) turretTarget += 360;
```
`RedCloseGate` uses **±180 degree** wrapping:
```java
if (turretTarget > 180 + 30) turretTarget -= 360;  // red
else if (turretTarget < -180 - 30) turretTarget += 360;
```
These are two different mathematical approaches. They correspond to the different turret limit conventions (blue turret goes 0 to 1865, red turret goes -990 to 850). The danger is that **if anyone copies the aiming code from one file to the other without also copying the matching limit values**, the turret will sometimes try to spin the long way around and slam into its physical stop.

#### Fix
Document this explicitly in both files with a comment, and consider creating a shared `TURRET_LOW_LIMIT`, `TURRET_HIGH_LIMIT`, and `wrapAngle()` method that is passed the limits as arguments, so the wrapping and limits are always changed together.

---

### 18. Intake motor may keep running after autonomous ends
**Files:** `BlueCloseGate.java`, `RedCloseGate.java`

#### What is happening
At 29.3/29.5 seconds into the autonomous, a time check stops the flywheels and returns the turret to center:
```java
if (opmodeTimer.getElapsedTimeSeconds() < 29.5) {
    aiming(detectedTags);
} else {
    intake.setPower(0);          // ✓ intake stopped in RedClose15
    flyWheel1.setVelocity(0);
    flyWheel2.setVelocity(0);
    turret.setTargetPosition(0);
}
```
`RedClose15` correctly stops the intake in this block. However, in `BlueCloseGate` and `RedCloseGate`, `intake.setPower(0)` is missing from this block. If the match timer runs out while the robot is in a `moveIntake` state — driving slowly along the field edge picking up pieces — the intake motor keeps running until `moveIntake()` naturally completes, which may be after the match has ended.

Running a motor after autonomous ends is a **game rule violation** in FTC. Depending on the referee, this may result in a penalty.

#### Fix
Add `intake.setPower(0)` to the time-expiry else block in both files:
```java
} else {
    intake.setPower(0);          // add this to BlueCloseGate and RedCloseGate
    flyWheel1.setVelocity(0);
    flyWheel2.setVelocity(0);
    turret.setTargetPosition(0);
}
```

---

## Part 4 — Vision System

### 19. Debug drawing overlays are enabled in competition code
**Files:** All autos — `initWebcam()`

#### What these settings do
```java
aprilTag = new AprilTagProcessor.Builder()
    .setDrawAxes(true)            // draws 3D axis lines on the tag in the camera image
    .setDrawCubeProjection(true)  // draws a 3D cube over the tag
    .setDrawTagOutline(true)      // draws an outline around the tag
    .build();
```
These three settings tell the AprilTag processor to draw visual overlays onto every camera frame it processes. This is useful during development — you can see on a monitor whether the robot is detecting the correct tag and estimating its position correctly.

In competition, you have `enableLiveView(false)` set, which means these annotated frames are never displayed anywhere. The processor is doing extra work on every frame to draw 3D graphics that nobody ever sees, consuming CPU time that could be used for faster detection.

#### Fix
Set all three to `false` in every auto's `initWebcam()`:
```java
.setDrawAxes(false)
.setDrawCubeProjection(false)
.setDrawTagOutline(false)
```
Keep them `true` only in test/development OpModes like `FlywheelTesting.java` where you actually view the camera output.

---

### 20. Vision targeting jumps slowly to the correct position on first detection
**Files:** Gate autos

#### How the EMA filter works
`GoalPos.update()` uses an **Exponential Moving Average (EMA)** — a standard technique for smoothing noisy sensor readings. The `alpha` parameter (0.08) controls how quickly new measurements are trusted:

```java
a = a * (1 - 0.08) + newMeasurement * 0.08;
// = 92% old estimate + 8% new measurement
```

On each camera frame, the estimate moves only 8% of the way toward the new measurement. This is excellent for filtering out frame-to-frame noise (camera jitter, motion blur, random errors), but it means the estimate takes many frames to converge to the true value.

With the camera at ~30fps and alpha=0.08, it takes approximately **35 frames (1.2 seconds)** for the estimate to reach 95% of the true position. During the first second after the camera first sees the tag, the turret is pointing at an estimate that is still 5–50% of the way from the initial guess to the real goal.

#### The fix — snap on first detection, then smooth
`FlywheelTesting.java` already implements the correct approach. It uses `alpha=1.0` for the very first detection (meaning: trust the measurement completely and jump straight to it) and `alpha=0.08` for all subsequent detections (smooth out noise):
```java
double alpha = hasEst ? 0.08 : 1.0;
goalPos.update(alpha, xPos, yPos, bearing, elevation, camRange);
hasEst = true;
```
With `alpha=1.0`, the estimate immediately becomes the measured value — no gradual convergence. After that, `alpha=0.08` smooths out noise from subsequent frames.

The Gate autos should adopt this same pattern.

---

### 21. Camera offset values are undocumented
**Files:** All autos

#### What these numbers are
```java
private final double camOffsetX = 2;                          // in aiming(): range + 2
bearing = detection.ftcPose.bearing + Math.toDegrees(Math.atan(3.2/range));  // RedClose15
bearing = detection.ftcPose.bearing - Math.toDegrees(Math.atan(2.5/range));  // Gate autos
```
These numbers compensate for the fact that the camera is physically not at the exact same location as the turret's rotation axis. The `camOffsetX = 2` accounts for the camera being mounted 2 inches in front of the turret axis. The `3.2` and `2.5` in the bearing correction are the lateral (sideways) offset of the camera from the turret axis — positive for one alliance side, negative for the other (because the goal is on different sides relative to the camera).

These are physical measurements of your specific robot. If the camera is remounted even slightly — for example during a repair — these numbers need to be re-measured and updated. But there is no documentation in the code explaining that these are physical measurements or how to re-measure them.

#### Fix
Add a comment near each value explaining what it represents and how it was measured:
```java
// Distance in inches from the turret rotation axis to the camera lens, forward direction
private final double CAM_FORWARD_OFFSET = 2.0;

// Distance in inches from the turret rotation axis to the camera lens, sideways direction
// Positive = camera is to the left of the turret axis (red alliance)
// Re-measure with a ruler if the camera mount is changed
private final double CAM_LATERAL_OFFSET = 3.2;
```

---

## Part 5 — Architecture & Maintainability
These are not bugs that affect competition immediately, but they make the codebase harder to maintain and are the root cause of several of the bugs above.

---

### 22. Every autonomous OpMode is a copy-paste of the same code
**Files:** All 9 auto OpModes

#### The problem
Every single autonomous file contains identical copies of:
- `initWebcam()` — ~30 lines, word-for-word identical
- `cameraControls()` — ~10 lines, identical
- `move()` — identical
- `moveIntake()` — nearly identical
- `shoot()` — nearly identical
- `toFWV()` — identical (with the parameter bug in 4 of them)
- Hardware initialization in `init()` — identical

This is called **copy-paste programming** and it is one of the most common sources of hard-to-find bugs. When `toFWV` had a parameter bug introduced at some point, it was present in 4 files and fixed in only 1 (`RedClose15`) — because there are 5 separate copies of the function. When the `GoalPos` API changed to add a Z dimension, some files were updated and others were not, causing the build failure in Finding #2.

Every time you need to change the shooting logic, the camera setup, or the `toFWV` formula, you have to make the same change in 9 different files. If you miss one, that auto will behave differently from the others without any obvious sign.

#### Fix
Create a `BaseAuto` abstract class that contains all the shared code. Each specific auto (RedClose15, BlueCloseGate, etc.) extends `BaseAuto` and only defines the things that differ: starting position, target tag ID, goal coordinates, turret limits, and bearing sign.

```java
// BaseAuto.java — contains initWebcam, cameraControls, move, moveIntake, shoot, toFWV
public abstract class BaseAuto extends OpMode {
    // Subclasses define these:
    protected abstract int getTagId();
    protected abstract GoalPos createGoalPos();
    protected abstract double getLowLimit();
    protected abstract double getHighLimit();

    // Shared code lives here once:
    protected void initWebcam() { ... }
    protected void shoot(PathState next) { ... }
    protected double toFWV(double r) { ... }
}

// RedClose15.java — only defines what's different
public class RedClose15 extends BaseAuto {
    protected int getTagId() { return 24; }
    protected GoalPos createGoalPos() { return new GoalPos(147, 143, 15.5); }
    protected double getLowLimit() { return -990; }
    protected double getHighLimit() { return 850; }
}
```

---

### 23. `DecodeDriveTrain` subsystem is not used in any autonomous mode
**Files:** `DecodeDriveTrain.java`, all auto OpModes

#### What is happening
`DecodeDriveTrain.java` was created to be a reusable **subsystem** — a class that encapsulates all the drivetrain code so that every OpMode doesn't have to duplicate it. This is a good software engineering pattern.

However, every autonomous OpMode initializes and controls its drive motors through Pedro Pathing directly (`Constants.createAutoFollower()`) rather than through `DecodeDriveTrain`. The `DecodeDriveTrain` class is only used in `FlywheelTesting.java`.

Additionally, the motor names and directions are defined in two places:
- `DecodeDriveTrain.java`: `FL`, `FR`, `BL`, `BR` with their directions
- `Constants.java (driveConstants)`: same motor names and directions for Pedro Pathing

If the robot's wiring changes (for example, a motor is swapped or re-plugged into a different port), both files must be updated independently or they will be out of sync.

---

### 24. `DecodeDriveTrain` will crash if telemetry is requested
**File:** `DecodeDriveTrain.java:163`

#### What is happening
`DecodeDriveTrain` has a field declared but never initialized:
```java
private Pose2D pose2D;  // declared — value is null
```
If any caller passes `showTelemetry = true` to the `Teleop()` method, the code reaches:
```java
telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
//                                      ^^^^^^^^ NullPointerException — pose2D is null!
```
Java will throw a `NullPointerException` — the app crashes. This is because `pose2D` requires the Pinpoint odometry to be running, but the Pinpoint initialization is commented out in the constructor.

Currently no caller passes `showTelemetry = true`, so this crash never happens. But it is a hidden trap — if someone uncomments the telemetry call while debugging, the app will crash with no obvious explanation.

---

### 25. Flywheel PIDF values differ between files with no explanation
**Files:** All autos

| File | P value | Notes |
|------|---------|-------|
| `RedClose15.java` | 400 | Active competition auto |
| `RedCloseGate.java` | 400 | Active competition auto |
| `BlueCloseGate.java` | 380 | Active competition auto |
| `FlywheelTesting.java` | 200 | Test/tuning mode |

The `P` value in a PIDF controller determines how aggressively the motor responds to a speed error — a higher P means the motor corrects faster but may overshoot. A lower P is more stable but slower to reach the target speed.

`FlywheelTesting` using 200 (half of competition) is appropriate for a tuning mode where you want gentle, observable behavior. But the 380 vs 400 difference between the blue and red competition autos is unexplained. It may reflect that these were tuned separately and the values were not synchronized, or there may be a genuine reason (different flywheel loads on each side). Either way, the reason should be documented so future team members know whether to keep the difference or unify the values.

---

### 26. Outdated files still active in the Driver Station menu
**Files:** `BlueTeleopWebcam.java`, `RedTeleopWebcam.java`, `centerstageTeleOp.java`, `TestAuto.java`

These are older versions of OpModes from previous seasons or development phases. Because they are not marked `@Disabled`, they appear in the Driver Station menu alongside current competition modes. In a timed match, accidentally selecting the wrong OpMode is a real risk.

Additionally, `BlueTeleopWebcam.java` and `RedTeleopWebcam.java` both call `Constants.createFollower()` — the missing method from Finding #1 — which means they contribute to the build failure even though they are not used.

#### Fix
Add `@Disabled` to each of these files, or delete them and keep them only in git history.

---

## Priority Order

| Priority | Findings | Why This Priority |
|----------|----------|-------------------|
| **P0 — Fix to build** | 1, 2, 3, 4, 5, 6 | Project does not compile until these are fixed |
| **P1 — Match performance** | 7, 8, 9, 10, 11 | Direct time savings in autonomous |
| **P2 — Reliability** | 12, 13, 14, 15, 16, 17, 18 | Prevent failures in competition |
| **P3 — Vision accuracy** | 19, 20, 21 | Improve targeting precision |
| **P4 — Architecture** | 22, 23, 24, 25, 26 | Prevent future bugs and make code easier to maintain |
