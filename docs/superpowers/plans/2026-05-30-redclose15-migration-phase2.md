# RedClose15 Migration + Phase 2 Naming Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate `RedClose15` from `extends OpMode` to `extends BaseAuto`, eliminating ~200 lines of duplicated code, and complete all Phase 2 naming fixes across the codebase in the same pass.

**Architecture:** Purely mechanical refactoring — no logic changes except: (1) `RedClose15` gains servo initial-position and `stop()` stopper-close fixes inherited from `BaseAuto`; (2) OUTTAKE2 gains `idle=true` flywheel pre-spin (code review Finding #7). All changes verified by `./gradlew assembleDebug`. There are no unit tests in this project — a clean build is the test.

**Tech Stack:** Java, FTC SDK, Pedro Pathing, Android Gradle

---

## File Map

| File | Change type |
|------|-------------|
| `subsystems/GoalPos.java` | Rename 4 private fields + 1 method |
| `subsystems/DecodeDriveTrain.java` | Rename 4 fields, 1 method, 4 local vars |
| `Autos/BaseAuto.java` | Rename method, add overload, rename 4 fields |
| `Autos/RedClose15.java` | Full BaseAuto migration |
| `Autos/BlueCloseGate.java` | 2 callsite updates + 9 path renames |
| `Autos/RedCloseGate.java` | Same as BlueCloseGate |
| `Autos/BlueClose.java` | 2 callsite updates + 9 path renames |
| `Autos/RedClose.java` | Same as BlueClose |
| `Autos/BlueFar.java` | 2 callsite updates + 5 path renames |
| `Autos/RedFar.java` | Same as BlueFar |
| `Autos/BlueFarGate.java` | 2 callsite updates + 8 path renames |
| `Autos/RedFarGate.java` | 2 callsite updates + 7 path renames |
| `BlueTeleopWebcam.java` | Fix broken constructor + callsite + field renames |
| `RedTeleopWebcam.java` | 1 callsite rename + field renames |
| `testcode/FlywheelTesting.java` | Fix broken constructor + callsite + field renames |

All paths relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

---

## Task 1: GoalPos — rename private fields and findAngle()

`findAngle()` is used in 11 files. Rename the method in `GoalPos` and update every callsite in the same commit — a partial rename breaks the build.

**Files:**
- Modify: `subsystems/GoalPos.java`
- Modify: `Autos/BlueCloseGate.java`, `Autos/RedCloseGate.java`, `Autos/BlueClose.java`, `Autos/RedClose.java`, `Autos/BlueFar.java`, `Autos/RedFar.java`, `Autos/BlueFarGate.java`, `Autos/RedFarGate.java`
- Modify: `BlueTeleopWebcam.java`, `RedTeleopWebcam.java`
- Modify: `testcode/FlywheelTesting.java` (one callsite is commented — update it too for consistency)

- [ ] **Step 1: Rewrite GoalPos.java with all renames applied**

Replace the entire file contents with:

```java
package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

public class GoalPos {
    private double goalX;
    private double goalY;
    private double goalZ;
    private double camElevationAngleDeg = 20; // degrees

    public GoalPos(double X, double Y, double Z) {
        goalX = X;
        goalY = Y;
        goalZ = Z;
    }

    public void setX(double n) { goalX = n; }
    public void setY(double n) { goalY = n; }
    public void setZ(double n) { goalZ = n; }

    public double getX() { return goalX; }
    public double getY() { return goalY; }
    public double getZ() { return goalZ; }

    public void update(double alpha, double x, double y, double heading, double elevation, double dist) {
        elevation += Math.toRadians(camElevationAngleDeg);
        double phi = Math.PI / 2 - elevation;
        phi = Range.clip(phi, 0, Math.PI);
        double x2 = x + dist * Math.cos(heading) * Math.sin(phi);
        double y2 = y + dist * Math.sin(heading) * Math.sin(phi);
        double z2 = dist * Math.cos(phi);
        goalX = goalX * (1 - alpha) + x2 * alpha;
        goalY = goalY * (1 - alpha) + y2 * alpha;
        goalZ = goalZ * (1 - alpha) + z2 * alpha;
        if (goalZ < 18) goalZ = 18;
    }

    public double findRange(double x, double y) {
        return Math.sqrt(Math.pow(goalX - x, 2) + Math.pow(goalY - y, 2));
    }

    public double findBearing(double x, double y) {
        return Math.toDegrees(Math.atan2((goalY - y), (goalX - x)));
    }

    @NonNull
    public String toString() {
        return "(" + Math.round(goalX * 100) / 100.0 + "," + Math.round(goalY * 100) / 100.0 + "," + Math.round(goalZ * 100) / 100.0 + ")";
    }
}
```

- [ ] **Step 2: Update findAngle → findBearing in all 8 auto subclasses**

In each of `BlueCloseGate.java`, `RedCloseGate.java`, `BlueClose.java`, `RedClose.java`, `BlueFar.java`, `RedFar.java`, `BlueFarGate.java`, `RedFarGate.java`:

Find (in `aiming()` method):
```java
double turretTarget = goalPos.findAngle(xPos, yPos)
```
Replace with:
```java
double turretTarget = goalPos.findBearing(xPos, yPos)
```

- [ ] **Step 3: Update findAngle → findBearing in TeleOp files**

In `BlueTeleopWebcam.java` line 309:
```java
// old:
double turretTarget = goal.findAngle(xPos, yPos)
// new:
double turretTarget = goal.findBearing(xPos, yPos)
```

In `RedTeleopWebcam.java` line 312:
```java
// old:
double turretTarget = goal.findAngle(xPos, yPos)
// new:
double turretTarget = goal.findBearing(xPos, yPos)
```

- [ ] **Step 4: Update findAngle → findBearing in FlywheelTesting**

In `testcode/FlywheelTesting.java` line 296 (active callsite):
```java
// old:
double turretTarget = goalPos.findAngle(xPos, yPos)
// new:
double turretTarget = goalPos.findBearing(xPos, yPos)
```

Line 404 is commented — update it too:
```java
//        telemetry.addData("target bearing", (goalPos.findBearing(follower.getPose().getX(), follower.getPose().getY())));
```

- [ ] **Step 5: Build**

```bash
./gradlew assembleDebug
```
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 6: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/GoalPos.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BlueCloseGate.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedCloseGate.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BlueClose.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedClose.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BlueFar.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedFar.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BlueFarGate.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedFarGate.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BlueTeleopWebcam.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RedTeleopWebcam.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/testcode/FlywheelTesting.java
git commit -m "refactor: rename GoalPos fields and findAngle → findBearing"
```

---

## Task 2: DecodeDriveTrain — rename fields, method, local vars; fix broken callers

**Context:** `BlueTeleopWebcam` and `FlywheelTesting` currently don't compile — they call `new DecodeDriveTrain(hardwareMap)` (1 arg) but the constructor requires 5 args, and they call `drivetrain.Teleop(gamepad1, heading, telemetry, fieldCentric)` (4 args) which doesn't exist. Both are fixed here alongside the rename.

**Files:**
- Modify: `subsystems/DecodeDriveTrain.java`
- Modify: `RedTeleopWebcam.java`
- Modify: `BlueTeleopWebcam.java`
- Modify: `testcode/FlywheelTesting.java`

- [ ] **Step 1: Rename fields and local vars in DecodeDriveTrain.java**

Replace the four motor field declarations (around line 22–25):
```java
// old:
private DcMotorEx FL;
private DcMotorEx FR;
private DcMotorEx BL;
private DcMotorEx BR;
// new:
private DcMotorEx frontLeft;
private DcMotorEx frontRight;
private DcMotorEx backLeft;
private DcMotorEx backRight;
```

In the constructor body, replace `FL`, `FR`, `BL`, `BR` assignments with `frontLeft`, `frontRight`, `backLeft`, `backRight`.

In the `Teleop()` method body, replace local variable declarations:
```java
// old:
double PowerFL = 0;
double PowerFR = 0;
double PowerBL = 0;
double PowerBR = 0;
// new:
double powerFl = 0;
double powerFr = 0;
double powerBl = 0;
double powerBr = 0;
```

Replace every subsequent use of `PowerFL`→`powerFl`, `PowerFR`→`powerFr`, `PowerBL`→`powerBl`, `PowerBR`→`powerBr` throughout the method body (approximately 20 occurrences — use find-and-replace within the file).

Replace every use of the field names `FL`→`frontLeft`, `FR`→`frontRight`, `BL`→`backLeft`, `BR`→`backRight` throughout the method body (`FL.setPower(...)` etc.).

- [ ] **Step 2: Rename the Teleop() method to drive()**

Change the method signature at line 94:
```java
// old:
public void Teleop(double heading) {
// new:
public void drive(double heading) {
```

- [ ] **Step 3: Fix RedTeleopWebcam callsite**

In `RedTeleopWebcam.java` line 162:
```java
// old:
drivetrain.Teleop(heading);
// new:
drivetrain.drive(heading);
```

- [ ] **Step 4: Fix BlueTeleopWebcam — constructor and callsite**

In `BlueTeleopWebcam.java` line 95, fix the broken 1-arg constructor call:
```java
// old:
drivetrain = new DecodeDriveTrain(hardwareMap);
// new:
drivetrain = new DecodeDriveTrain(hardwareMap, gamepad1, telemetry, false, fieldCentric);
```

In `BlueTeleopWebcam.java` line 148, fix the broken 4-arg Teleop call:
```java
// old:
drivetrain.Teleop(gamepad1, heading, telemetry, fieldCentric);
// new:
drivetrain.drive(heading);
```

- [ ] **Step 5: Fix FlywheelTesting — constructor and callsite**

In `testcode/FlywheelTesting.java` line 105, fix the broken 1-arg constructor call:
```java
// old:
drivetrain = new DecodeDriveTrain(hardwareMap);
// new:
drivetrain = new DecodeDriveTrain(hardwareMap, gamepad1, telemetry, false, fieldCentric);
```

In `testcode/FlywheelTesting.java` line 151, fix the broken 4-arg Teleop call:
```java
// old:
drivetrain.Teleop(gamepad1, heading, telemetry, fieldCentric);
// new:
drivetrain.drive(heading);
```

- [ ] **Step 6: Build**

```bash
./gradlew assembleDebug
```
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 7: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/DecodeDriveTrain.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RedTeleopWebcam.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BlueTeleopWebcam.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/testcode/FlywheelTesting.java
git commit -m "refactor: rename DecodeDriveTrain fields and Teleop → drive; fix broken callers"
```

---

## Task 3: BaseAuto — rename baseInit(), add idle-move overload, rename PIDF fields

`baseInit()` is called in 8 subclasses. Rename in BaseAuto and update all 8 callsites in the same commit.

**Files:**
- Modify: `Autos/BaseAuto.java`
- Modify: `Autos/BlueCloseGate.java`, `Autos/RedCloseGate.java`, `Autos/BlueClose.java`, `Autos/RedClose.java`, `Autos/BlueFar.java`, `Autos/RedFar.java`, `Autos/BlueFarGate.java`, `Autos/RedFarGate.java`

- [ ] **Step 1: Rename baseInit() → initHardware() in BaseAuto**

In `BaseAuto.java` line 80 (the method declaration):
```java
// old:
protected void baseInit() {
// new:
protected void initHardware() {
```

- [ ] **Step 2: Rename PIDF fields in BaseAuto**

Replace the four field declarations (lines 61–64):
```java
// old:
protected double p;
protected double d = 0;
protected double i = 0;
protected double f = 13.5;
// new:
protected double pidP;
protected double pidD = 0;
protected double pidI = 0;
protected double pidF = 13.5;
```

In the `initHardware()` body, replace every use of `p`, `d`, `i`, `f` that refers to these fields:
```java
// old:
p = getPIDFP();
fwPID = new PIDFCoefficients(p, i, d, f);
// new:
pidP = getPIDFP();
fwPID = new PIDFCoefficients(pidP, pidI, pidD, pidF);
```

- [ ] **Step 3: Add the idle-move overload to BaseAuto**

After the existing `move(PathChain path, Runnable onComplete)` method, add:

```java
public void move(PathChain path, Runnable onComplete, boolean idle) {
    if (!moving) {
        if (idle) flyWheel1.setVelocity(1100);
        follower.followPath(path, true);
        moving = true;
    }
    if (!follower.isBusy() && actionTimer.getElapsedTime() > 50) {
        moving = false;
        onComplete.run();
    }
}
```

- [ ] **Step 4: Update baseInit → initHardware in all 8 subclasses**

In each of `BlueCloseGate.java`, `RedCloseGate.java`, `BlueClose.java`, `RedClose.java`, `BlueFar.java`, `RedFar.java`, `BlueFarGate.java`, `RedFarGate.java`, find the `init()` method and change:
```java
// old:
baseInit();
// new:
initHardware();
```

- [ ] **Step 5: Build**

```bash
./gradlew assembleDebug
```
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 6: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BaseAuto.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BlueCloseGate.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedCloseGate.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BlueClose.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedClose.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BlueFar.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedFar.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BlueFarGate.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedFarGate.java
git commit -m "refactor: rename baseInit → initHardware, add idle-move overload, rename PIDF fields"
```

---

## Task 4: RedClose15 — full BaseAuto migration

Replace the entire `RedClose15.java` with the migrated version. The file goes from ~460 lines to ~250 lines.

**Files:**
- Modify: `Autos/RedClose15.java`

- [ ] **Step 1: Replace RedClose15.java with migrated content**

Write the complete new file:

```java
package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Red 15", group = "Autos")
public class RedClose15 extends BaseAuto {

    private DcMotorEx flyWheel2;
    private final double lowLimit = -990;
    private final double highLimit = 850;
    private double elevation;

    private enum PathState {
        PRELOAD, SHOOTPRE,
        INTAKE11, INTAKE12, OUTTAKE1, SHOOT1,
        OPENGATE, BIGBACK, OUTTAKEB, SHOOTB,
        INTAKE2, OUTTAKE2, SHOOT2,
        INTAKE31, INTAKE32, OUTTAKE3, SHOOT3,
        END, STOP
    }

    private PathState pathState;

    // Poses
    private final Pose start        = new Pose(120, 133, Math.toRadians(0));
    private final Pose outtakePre   = new Pose(93, 90, Math.toRadians(0));
    private final Pose outtake      = new Pose(100, 90, Math.toRadians(0));
    private final Pose intake1p1    = new Pose(105, 67, Math.toRadians(0));
    private final Pose intake1p2    = new Pose(130, 65, Math.toRadians(0));
    private final Pose outtake1Point= new Pose(106, 65, Math.toRadians(0));
    private final Pose gatePoint    = new Pose(122, 46, Math.toRadians(46));
    private final Pose gate         = new Pose(133, 65.5, Math.toRadians(20));
    private final Pose bigBack      = new Pose(142, 53, Math.toRadians(46));
    private final Pose bigBackPoint = new Pose(133, 50, Math.toRadians(46));
    private final Pose outtakeBPoint= new Pose(100, 60, Math.toRadians(0));
    private final Pose intake2      = new Pose(127, 88, Math.toRadians(0));
    private final Pose intake3p1    = new Pose(100, 45, Math.toRadians(0));
    private final Pose intake3p2    = new Pose(130, 39, Math.toRadians(0));
    private final Pose end          = new Pose(108, 77, Math.toRadians(0));

    // Paths
    private PathChain preload, intake11, intake12, outtake1;
    private PathChain openGate, bigBackPath, outtakeB;
    private PathChain intake2Path, outtake2;
    private PathChain intake31, intake32, outtake3;
    private PathChain endPath;

    @Override protected double getPIDFP()       { return 400; }
    @Override protected GoalPos createGoalPos() { return new GoalPos(147, 143, 15.5); }
    @Override protected Pose getStartPose()     { return start; }
    @Override protected double getFWVConstant() { return 1150; }

    @Override
    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setLinearHeadingInterpolation(start.getHeading(), outtakePre.getHeading())
                .build();
        intake11 = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, intake1p1))
                .setConstantHeadingInterpolation(0)
                .build();
        intake12 = follower.pathBuilder()
                .addPath(new BezierLine(intake1p1, intake1p2))
                .setConstantHeadingInterpolation(0)
                .build();
        outtake1 = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(intake1p2, outtake1Point, outtake)))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();
        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(outtake, gatePoint, gate)))
                .setLinearHeadingInterpolation(outtake.getHeading(), gate.getHeading())
                .setBrakingStrength(0.08)
                .build();
        bigBackPath = follower.pathBuilder()
                .addPath(new BezierLine(gate, bigBack))
                .setLinearHeadingInterpolation(gate.getHeading(), bigBack.getHeading())
                .build();
        outtakeB = follower.pathBuilder()
                .addPath(new BezierCurve(bigBack, outtakeBPoint, outtake))
                .setLinearHeadingInterpolation(gate.getHeading(), outtake.getHeading())
                .build();
        intake2Path = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2))
                .setConstantHeadingInterpolation(0)
                .build();
        outtake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, outtake))
                .setConstantHeadingInterpolation(0)
                .build();
        intake31 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake3p1))
                .setConstantHeadingInterpolation(0)
                .build();
        intake32 = follower.pathBuilder()
                .addPath(new BezierLine(intake3p1, intake3p2))
                .setConstantHeadingInterpolation(0)
                .build();
        outtake3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3p2, outtake))
                .setConstantHeadingInterpolation(0)
                .build();
        endPath = follower.pathBuilder()
                .addPath(new BezierLine(outtake, end))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    protected void setPathState(PathState newState) {
        pathState = newState;
        actionTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.PRELOAD;
        initHardware();
        flyWheel2 = hardwareMap.get(DcMotorEx.class, "FW2");
        flyWheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, fwPID);
    }

    @Override
    public void stop() {
        flyWheel2.setVelocity(0);
        super.stop();
    }

    @Override
    public void statePathUpdate() {
        telemetry.update();
        List<AprilTagDetection> detectedTags = aprilTag.getDetections();

        switch (pathState) {
            case PRELOAD:
                flyWheel2.setVelocity(1100);
                move(preload, () -> setPathState(PathState.SHOOTPRE), true);
                break;
            case SHOOTPRE:
                shoot(PathState.INTAKE11);
                break;
            case INTAKE11:
                move(intake11, () -> setPathState(PathState.INTAKE12));
                break;
            case INTAKE12:
                moveIntake(intake12, 0.7, false, 50, () -> setPathState(PathState.OUTTAKE1));
                break;
            case OUTTAKE1:
                flyWheel2.setVelocity(1100);
                move(outtake1, () -> setPathState(PathState.SHOOT1), true);
                break;
            case SHOOT1:
                shoot(PathState.OPENGATE);
                break;
            case OPENGATE:
                move(openGate, () -> setPathState(PathState.BIGBACK));
                break;
            case BIGBACK:
                moveIntake(bigBackPath, 0.65, false, 50, () -> setPathState(PathState.OUTTAKEB));
                break;
            case OUTTAKEB:
                flyWheel2.setVelocity(1100);
                move(outtakeB, () -> setPathState(PathState.SHOOTB), true);
                break;
            case SHOOTB:
                shoot(PathState.INTAKE2);
                break;
            case INTAKE2:
                moveIntake(intake2Path, 0.7, false, 50, () -> setPathState(PathState.OUTTAKE2));
                break;
            case OUTTAKE2:
                flyWheel2.setVelocity(1100);
                move(outtake2, () -> setPathState(PathState.SHOOT2), true);
                break;
            case SHOOT2:
                shoot(PathState.INTAKE31);
                break;
            case INTAKE31:
                move(intake31, () -> setPathState(PathState.INTAKE32));
                break;
            case INTAKE32:
                moveIntake(intake32, 0.7, false, 50, () -> setPathState(PathState.OUTTAKE3));
                break;
            case OUTTAKE3:
                flyWheel2.setVelocity(1100);
                move(outtake3, () -> setPathState(PathState.SHOOT3), true);
                break;
            case SHOOT3:
                shoot(PathState.END);
                break;
            case END:
                move(endPath, () -> setPathState(PathState.STOP));
                break;
        }

        if (pathState != PathState.END && pathState != PathState.STOP
                && opmodeTimer.getElapsedTimeSeconds() < 29.5) {
            aiming(detectedTags);
        } else {
            intake.setPower(0);
            flyWheel1.setVelocity(0);
            flyWheel2.setVelocity(0);
        }
    }

    public void shoot(PathState nextPath) {
        double targetV = toFWV(range);
        flyWheel1.setVelocity(targetV);
        flyWheel2.setVelocity(targetV);
        if (range < 40) {
            flapPos = 0;
        } else if (range < 95) {
            flapPos = 0.2;
        } else {
            flapPos = 0.24;
        }
        flap.setPosition(flapPos);
        double fwv = flyWheel1.getVelocity();
        if (fwv >= targetV) {
            stopper.setPosition(0.973);
            intake.setPower(1);
        }
        if (actionTimer.getElapsedTime() > 1600) {
            intake.setPower(0);
            stopper.setPosition(0.9);
            flyWheel1.setVelocity(0);
            flyWheel2.setVelocity(0);
            pathState = nextPath;
            actionTimer.resetTimer();
        }
    }

    public void aiming(List<AprilTagDetection> detectedTags) {
        range = goalPos.findRange(xPos, yPos);
        if (gainSet) {
            for (AprilTagDetection detection : detectedTags) {
                if (detection.metadata != null && detection.id == 24) {
                    camRange = detection.ftcPose.range + camOffsetX;
                    bearing = detection.ftcPose.bearing + Math.toDegrees(Math.atan(3.2 / range));
                    elevation = detection.ftcPose.elevation;
                    bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180 / 976;
                    bearing = Math.toRadians(bearing);
                    elevation = Math.toRadians(elevation);
                    goalPos.update(0.08, xPos, yPos, bearing, elevation, camRange);
                    break;
                }
            }
        }
        double turretTarget = goalPos.findBearing(xPos, yPos)
                - startingAngle
                - Math.toDegrees(heading);
        if (turretTarget > 180 + 30) turretTarget -= 360;
        else if (turretTarget < -180 - 30) turretTarget += 360;
        turretTarget = 976.0 / 180.0 * turretTarget;
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit);
        turret.setTargetPosition((int) turretTarget);
    }
}
```

> **Note on path name conflict:** `BigBack` is renamed to `bigBackPath` and `Intake2` to `intake2Path` to avoid shadowing the `Pose` fields `bigBack` and `intake2`. Verify pose field names against the original if the build fails with "already defined" errors.

- [ ] **Step 2: Build**

```bash
./gradlew assembleDebug
```
Expected: `BUILD SUCCESSFUL`

If the build fails with a "duplicate local variable" or "already defined" error, check that the `PathChain` field names don't shadow the `Pose` field names. Adjust the `PathChain` field names as needed (e.g., `intake2Path` if `intake2` is taken by the pose).

- [ ] **Step 3: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedClose15.java
git commit -m "refactor: migrate RedClose15 to BaseAuto; apply Phase 2 path renames; fix Finding 7"
```

---

## Task 5: Phase 2 path renames in existing 8 BaseAuto subclasses

Each file is independent. Rename PascalCase `PathChain` fields to camelCase throughout each file (declarations, `buildPaths()`, and all `statePathUpdate()` switch-case references). Do all 8 in one commit.

**Files:**
- Modify: all 8 BaseAuto subclasses (not RedClose15 — done in Task 4)

- [ ] **Step 1: BlueCloseGate.java and RedCloseGate.java**

Both files have identical path field names. Apply these renames in each file:

| Old | New |
|-----|-----|
| `Preload` | `preload` |
| `Intake1` | `intake1` |
| `Opengate` | `openGate` |
| `Outtake1` | `outtake1` |
| `Intake21` | `intake21` |
| `Intake22` | `intake22` |
| `Outtake2` | `outtake2` |
| `Intake31` | `intake31` |
| `Intake32` | `intake32` |
| `Outtake3` | `outtake3` |
| `End` | `end` |

Use find-and-replace within each file. These are private fields — no external callsites.

- [ ] **Step 2: BlueClose.java and RedClose.java**

Both files have identical path field names:

| Old | New |
|-----|-----|
| `Preload` | `preload` |
| `Intake1` | `intake1` |
| `Outtake1` | `outtake1` |
| `Intake21` | `intake21` |
| `Intake22` | `intake22` |
| `Outtake2` | `outtake2` |
| `Intake31` | `intake31` |
| `Intake32` | `intake32` |
| `Outtake3` | `outtake3` |
| `End` | `end` |

- [ ] **Step 3: BlueFar.java and RedFar.java**

Both files have identical path field names:

| Old | New |
|-----|-----|
| `Preload` | `preload` |
| `AlignIntake` | `alignIntake` |
| `Intake1` | `intake1` |
| `Outtake1` | `outtake1` |
| `End` | `end` |

- [ ] **Step 4: BlueFarGate.java**

| Old | New |
|-----|-----|
| `Preload` | `preload` |
| `AlignIntake` | `alignIntake` |
| `Intake1` | `intake1` |
| `Outtake1` | `outtake1` |
| `AlignIntake2` | `alignIntake2` |
| `Togate` | `toGate` |
| `Outtake2` | `outtake2` |
| `End` | `end` |

- [ ] **Step 5: RedFarGate.java**

| Old | New |
|-----|-----|
| `Preload` | `preload` |
| `AlignIntake` | `alignIntake` |
| `Intake1` | `intake1` |
| `Outtake1` | `outtake1` |
| `Intake21` | `intake21` |
| `Intake22` | `intake22` |
| `Outtake2` | `outtake2` |
| `End` | `end` |

- [ ] **Step 6: Build**

```bash
./gradlew assembleDebug
```
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 7: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BlueCloseGate.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedCloseGate.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BlueClose.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedClose.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BlueFar.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedFar.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/BlueFarGate.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedFarGate.java
git commit -m "refactor: rename PascalCase PathChain fields to camelCase in all BaseAuto subclasses"
```

---

## Task 6: Phase 2 field renames in TeleOp and FlywheelTesting

**Files:**
- Modify: `BlueTeleopWebcam.java`, `RedTeleopWebcam.java`, `testcode/FlywheelTesting.java`

- [ ] **Step 1: BlueTeleopWebcam.java — rename PIDF and flywheel fields**

Rename field declarations (around lines 57–62, 84–87):
```java
// old:
double FWTarget = 0;
double FWV1;
double FWV2;
double p = 400;
double d = 0;
double i = 0;
double f = 13.5;
// new:
double flywheelTarget = 0;
double flywheelVelocity1;
double flywheelVelocity2;
double pidP = 400;
double pidD = 0;
double pidI = 0;
double pidF = 13.5;
```

Replace all subsequent uses of `FWTarget`→`flywheelTarget`, `FWV1`→`flywheelVelocity1`, `FWV2`→`flywheelVelocity2`, `p`→`pidP`, `d`→`pidD`, `i`→`pidI`, `f`→`pidF` throughout the file (use find-and-replace, but be careful that `p`, `i`, `d`, `f` as standalone identifiers only appear as PIDF fields — spot-check a few replacements).

- [ ] **Step 2: RedTeleopWebcam.java — rename PIDF and flywheel fields**

Field declarations at lines 59, 63–64 and 86–89:
```java
// old:
double FWTarget = 0;
double FWV1;
double FWV2;
double p = 400;
double d = 0;
double i = 0;
double f = 13.5;
// new:
double flywheelTarget = 0;
double flywheelVelocity1;
double flywheelVelocity2;
double pidP = 400;
double pidD = 0;
double pidI = 0;
double pidF = 13.5;
```

Replace all uses of `FWTarget`→`flywheelTarget`, `FWV1`→`flywheelVelocity1`, `FWV2`→`flywheelVelocity2`, `p`→`pidP`, `d`→`pidD`, `i`→`pidI`, `f`→`pidF` throughout the file.

- [ ] **Step 3: FlywheelTesting.java — rename PIDF and flywheel fields**

Field declarations (around lines 54, 61–64, 71–72):
```java
// old:
double FW1Target = 0;
double FWV1;
double FWV2;
double p = 200;
double d = 0;
double i = 0;
double f = 13.5;
// new:
double flywheelTarget = 0;
double flywheelVelocity1;
double flywheelVelocity2;
double pidP = 200;
double pidD = 0;
double pidI = 0;
double pidF = 13.5;
```

Replace all uses of `FW1Target`→`flywheelTarget`, `FWV1`→`flywheelVelocity1`, `FWV2`→`flywheelVelocity2`, `p`→`pidP`, `d`→`pidD`, `i`→`pidI`, `f`→`pidF` throughout the file.

- [ ] **Step 4: Build**

```bash
./gradlew assembleDebug
```
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BlueTeleopWebcam.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RedTeleopWebcam.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/testcode/FlywheelTesting.java
git commit -m "refactor: rename PIDF and flywheel fields to camelCase in TeleOp and FlywheelTesting"
```

---

## Task 7: Final verification

- [ ] **Step 1: Clean build**

```bash
./gradlew clean assembleDebug
```
Expected: `BUILD SUCCESSFUL` with zero warnings about undefined symbols.

- [ ] **Step 2: Verify success criteria**

```bash
# 1. RedClose15 extends BaseAuto
grep "extends" TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autos/RedClose15.java
# Expected: public class RedClose15 extends BaseAuto

# 2. No Teleop method remains
grep -rn "\.Teleop(" TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
# Expected: no output

# 3. No baseInit() remains
grep -rn "baseInit" TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
# Expected: no output

# 4. No findAngle remains
grep -rn "findAngle" TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
# Expected: no output

# 5. No single-letter PIDF fields remain (spot check)
grep -rn "double p = \|double i = \|double d = \|double f = " TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
# Expected: no output

# 6. No PascalCase PathChain fields remain (spot check)
grep -rn "PathChain Preload\|PathChain Intake\|PathChain Outtake\|PathChain End\b" TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
# Expected: no output
```

- [ ] **Step 3: Commit if any fixups were needed, otherwise done**
