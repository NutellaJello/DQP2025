# Library Review & Recommendations
**Date:** 2026-05-02

---

## Current Stack

| Library | Version | Role |
|---|---|---|
| FTC SDK (RobotCore, Hardware, Vision, etc.) | 11.0.0 | Core framework |
| Pedro Pathing (`com.pedropathing:ftc`) | 2.0.5 | Autonomous path following |
| Pedro Pathing Telemetry | 1.0.0 | Dashboard telemetry |
| FullPanels (`com.bylazar:fullpanels`) | 1.0.6 | Field visualizer UI |
| AndroidX AppCompat | 1.2.0 | Android support |

---

## Recommendation 1 — Limelight Vision Coprocessor
**Impact: High — offloads all vision computation from the Control Hub**

The Limelight 3A is a dedicated vision coprocessor designed specifically for FTC/FRC. Currently, AprilTag detection and the turret targeting pipeline run on the Control Hub's CPU, competing with Pedro Pathing, motor control, and all other OpMode logic for the same processor.

**What Limelight provides:**
- Onboard ARM processor running vision pipelines independently of the Control Hub
- Built-in AprilTag detection pipeline with no CPU cost on the Control Hub
- Python-scriptable pipelines for custom vision tasks
- Latency as low as 10ms for tag detection vs ~30–50ms on the Control Hub
- Direct I2C or network interface — integrates with FTC SDK via the Limelight Java SDK

**Impact on your code:**
The current `initWebcam()` / `cameraControls()` / `aprilTag.getDetections()` pattern would be replaced with a Limelight query:
```java
// Current: AprilTag on Control Hub camera
List<AprilTagDetection> detectedTags = aprilTag.getDetections();

// With Limelight:
LLResult result = limelight.getLatestResult();
if (result != null && result.isValid()) {
    LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
    double camRange = tag.getTargetXDegrees(); // or 3D pose
}
```

**Why this matters for your robot specifically:**
- The turret needs to track the goal at ~30Hz. Currently, vision and turret control share CPU with Pedro Pathing's path follower. Limelight decouples them entirely.
- The `gainSet` race condition (waiting for the Control Hub camera to reach `STREAMING` state before shooting) goes away — the Limelight is always running.
- Camera exposure/gain tuning (`cameraControls()`) is handled in the Limelight web UI, not in code.
- The 3D pose output from Limelight's AprilTag pipeline can replace the `GoalPos` EMA estimator with a direct measurement, removing the `alpha=0.08` convergence delay.

**Hardware:** Limelight 3A (~$150). Mounts where your current webcam is. Powered via USB from the Control Hub.

**SDK dependency:**
```groovy
// Add to build.dependencies.gradle
implementation 'com.limelightvision:limelight-ftc:1.0.0'
```

---

## Recommendation 2 — Pedro Pathing: Switch to Predictive Braking
**Impact: High — ~15% faster autonomous paths**

Pedro Pathing 2.0.5 includes a Predictive Braking algorithm that replaces the manually-tuned translational and drive PIDFs. The Pedro docs state: *"A world-record autonomous was achieved, and many other teams' autos were sped up by ~15%, all while automatically tuning in a few minutes."*

**How it works:** Instead of a D term to prevent overshoot, the algorithm predicts how far the robot will slide during braking using measured deceleration constants, and begins braking at exactly the right moment. It maximizes deceleration without overshoot.

**What to remove from `Constants.java`:**
```java
// Remove:
.translationalPIDFCoefficients(...)
.secondaryTranslationalPIDFCoefficients(...)
.drivePIDFCoefficients(...)
.secondaryDrivePIDFCoefficients(...)
.useSecondaryTranslationalPIDF(true)
.useSecondaryDrivePIDF(true)
.centripetalScaling(0.0005)
```

**What to add:**
```java
.predictiveBrakingCoefficients(
    new PredictiveBrakingCoefficients(0.1, kLinear, kQuadratic)
)
.centripetalScaling(0)
```

**Tuning steps:**
1. Run `Tuning.java → Automatic → PredictiveBrakingTuner` — outputs `kLinear` and `kQuadratic` automatically
2. Run `LineTest` and tune `kP` between 0.05–0.3
3. Lower `tValueConstraint` from `0.99` to `0.95` in `PathConstraints` — Predictive Braking stops accurately so the path can end earlier

**Note:** Also lower `centripetalScaling` to `0` — Predictive Braking accounts for centripetal forces natively.

---

## Recommendation 3 — Update FullPanels 1.0.6 → 1.0.12
**Impact: Medium — visualizer fixes and compatibility**

The Pedro Pathing installation docs now specify `1.0.12`. Six minor versions behind means missing bug fixes in the field visualizer used during tuning sessions.

**Change in `build.dependencies.gradle`:**
```groovy
implementation 'com.bylazar:fullpanels:1.0.12'  // was 1.0.6
```

---

## Recommendation 4 — Add FTC Dashboard for Shooting System Tuning
**Impact: Medium — real-time flywheel/turret tuning without rebuilding**

Pedro Pathing's FullPanels covers drive path visualization, but the flywheel + turret + vision pipeline has no real-time tuning tool. Every adjustment to `p`, `f`, flap thresholds, or `toFWV` constants requires a full APK rebuild and redeploy (~2–3 minutes).

FTC Dashboard provides a web UI at `192.168.43.1:8080` with live-editable fields and telemetry graphs. The `@Config` annotation exposes any class's static fields as sliders:

```java
@Config
public class FlywheelTesting extends LinearOpMode {
    public static double p = 200;
    public static double f = 13.5;
    public static double FW1Target = 0;
    public static double FLAP_NEAR = 0.0;
    public static double FLAP_MID = 0.2;
    public static double FLAP_FAR = 0.24;
    // All editable live in the browser
}
```

**Add to `build.dependencies.gradle`:**
```groovy
repositories {
    maven { url = 'https://maven.brott.dev/' }
}
dependencies {
    implementation 'com.acmerobotics.dashboard:dashboard:0.4.16'
}
```

---

## Recommendation 5 — Use Pedro Pathing Ivy (Already Included)
**Impact: Medium — parallel drive + mechanism execution**

Ivy is already bundled in `com.pedropathing:ftc:2.0.5` but unused. It allows the flywheel to spin up in parallel while the robot is driving to the scoring position, rather than waiting until the robot arrives.

```java
// Current (sequential): drive → arrive → spin up → wait → shoot
// With Ivy (parallel):
parallel(
    follow(follower, Outtake1),
    run(() -> {
        flyWheel1.setVelocity(targetV);
        flyWheel2.setVelocity(targetV);
    })
)
```

No new dependency. See Pedro Pathing Ivy docs for full integration.

---

## Recommendation 6 — Evaluate SparkFun OTOS as Pinpoint Alternative
**Impact: Low-Medium — depends on current localization reliability**

Pedro Pathing supports the SparkFun OTOS (Optical Tracking Odometry Sensor) as an alternative to the GoBilda Pinpoint. OTOS uses optical flow rather than mechanical encoder pods.

| | GoBilda Pinpoint (current) | SparkFun OTOS |
|---|---|---|
| Mechanism | Two physical encoder pods | Optical flow sensor |
| Failure mode | Pod slip, wear, carpet snagging | Highly reflective or featureless floors |
| Wiring | I2C + pod cables | Single I2C cable |
| Robot weight | Heavier (pods + hardware) | Lighter |
| Drift | Low on good surfaces | Very low on most FTC floors |

**Recommendation:** Only evaluate if localization drift is observed mid-auto (robot missing intake positions on the 3rd or 4th cycle). If Pinpoint is tracking reliably, no change is warranted.

---

## Not Recommended

### AndroidX AppCompat 1.2.0
Leave at current version. The FTC SDK pins to `1.2.0` due to its `targetSdkVersion 28` constraint. Upgrading independently risks breaking SDK UI components with no performance benefit.

---

## Priority Summary

| # | Change | Effort | Expected Gain |
|---|--------|--------|---------------|
| 1 | Add Limelight 3A coprocessor | Hardware + 1–2 days integration | Vision latency, Control Hub CPU freed |
| 2 | Switch to Predictive Braking | 1–2 hour tuning session | ~15% faster auto paths |
| 3 | Update FullPanels to 1.0.12 | 5 minutes | Visualizer fixes |
| 4 | Add FTC Dashboard | 1 hour setup | Real-time shooting system tuning |
| 5 | Use Ivy for parallel execution | Code refactor | 0.5–1s saved per shoot cycle |
| 6 | Evaluate OTOS | 1 test session | Localization reliability |

**Recommended order of implementation:**
1. Fix all P0 build errors first (see `CODE_REVIEW.md`)
2. Predictive Braking (quick win, no hardware cost)
3. FTC Dashboard (speeds up all subsequent tuning)
4. Limelight (biggest vision improvement, plan for next build window)
5. Ivy (refactor once build is stable)
