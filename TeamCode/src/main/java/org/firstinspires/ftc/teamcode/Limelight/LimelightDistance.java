package org.firstinspires.ftc.teamcode.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * FTC-legal Limelight 3A extractor for floor-level balls/game pieces using a
 * DOWNWARD-facing camera. Computes straight-line (Euclidean) distance to the
 * target via mount-angle trig, same approach as StableDistanceLExtractor
 * (AprilTags) but with the vertical geometry flipped for a camera looking
 * down at something below/near floor level instead of up at a tag.
 *
 * Geometry:
 *   - LIMELIGHT_HEIGHT_INCHES: lens height off the floor.
 *   - LIMELIGHT_ANGLE_DEGREES: how far the camera is tilted DOWN from
 *     horizontal (0 = camera level with horizon, 90 = camera pointing
 *     straight down at the floor).
 *   - BALL_HEIGHT_INCHES: height of the ball's center off the floor
 *     (roughly its radius, or 0 if you're targeting where it touches
 *     the floor).
 *
 * verticalDistance = LIMELIGHT_HEIGHT_INCHES - BALL_HEIGHT_INCHES (lens is
 * above the ball, so this is positive) horizontalDistance =
 * verticalDistance / tan(totalAngle) euclideanDistance = sqrt(horizontal^2 +
 * vertical^2)
 *
 * IMPORTANT — verify sign convention on your robot before trusting this:
 * as the ball gets closer, it should appear lower in frame, which should
 * make ty push the totalAngle toward "more downward" (steeper). Print ty
 * telemetry while manually moving the ball closer/farther to confirm which
 * sign (+ty or -ty) does that on your camera, then set TY_SIGN accordingly.
 *
 * Ownership note: only one consumer should call limelight.getLatestResult()
 * for a given Limelight3A instance.
 *
 * Usage:
 *   StableBallDistanceLExtractor ballDistance = new StableBallDistanceLExtractor(hardwareMap);
 *   ballDistance.setTelemetry(telemetry); // optional
 *   ballDistance.startReading();
 *   ...
 *   Double distance = ballDistance.getDistance(); // inches, null if no ball visible
 *   ...
 *   ballDistance.stopReading(); // call when OpMode ends
 */
public class LimelightDistance {

    // --- Tunable values ---
    private static final double SMOOTHING_FACTOR = 0.1;      // exponential smoothing for tx/ty (0-1, higher = less smoothing)
    private static final long STALE_TIMEOUT_MS = 500;        // discard readings older than this
    private static final int POLL_RATE_HZ = 100;
    private static final int PIPELINE_INDEX = 1;               // set to your ball-detector pipeline index

    // --- Mount geometry — measure on your robot ---
    private static final double LIMELIGHT_HEIGHT_INCHES = 10.7;  // lens height off floor
    private static final double LIMELIGHT_ANGLE_DEGREES = 45.0;  // degrees DOWN from horizontal
    private static final double BALL_HEIGHT_INCHES = 2.4;        // ball center height off floor (~radius)

    // Flip to -1 if ty telemetry shows the wrong direction (see class doc above)
    private static final double TY_SIGN = 1.0;

    private final Limelight3A limelight;
    private Telemetry opModeTelemetry;

    // Raw Limelight values
    private volatile Double tx = null;
    private volatile Double ty = null;
    private volatile Double ta = null;

    // Smoothed / derived values
    private volatile double horizontalAngle = 0.0;
    private volatile double verticalAngle = 0.0;
    private volatile boolean targetVisible = false;

    private volatile String connectionStatus = "Not connected";
    private volatile long lastUpdateTime = 0;
    // Last exception seen inside the polling thread, surfaced to telemetry instead
    // of being swallowed / killing the thread silently. Cleared on a clean read.
    private volatile String lastPollError = null;

    private Thread pollingThread;
    private volatile boolean running = false;
    // True once horizontalAngle/verticalAngle have a real seeded value for the
    // CURRENT continuous sighting of the target. Reset to false whenever the
    // target goes stale, so the next reacquisition seeds directly from the raw
    // reading instead of exponentially blending up from the last value (which,
    // combined with the stale-reset-to-zero below, otherwise makes distance
    // read artificially close for several frames after every reacquisition).
    private volatile boolean smoothingInitialized = false;

    public LimelightDistance(HardwareMap hardwareMap) {
        this(hardwareMap, "Limelight");
    }

    public LimelightDistance(HardwareMap hardwareMap, String deviceName) {
        // hardwareMap.get() throws IllegalArgumentException if deviceName doesn't
        // match the robot config exactly. Re-throwing with the device name in the
        // message turns a generic "IllegalArgumentException" crash screen into one
        // that immediately tells you which config entry is wrong.
        Limelight3A ll;
        try {
            ll = hardwareMap.get(Limelight3A.class, deviceName);
        } catch (Exception e) {
            throw new RuntimeException(
                    "LimelightDistance: no device named \"" + deviceName + "\" in the active robot config. " +
                            "Check Driver Station config matches this name exactly (case-sensitive).", e);
        }
        limelight = ll;
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.start();
    }

    public void setTelemetry(Telemetry telemetry) {
        this.opModeTelemetry = telemetry;
    }

    public void startReading() {
        // Guard against double-start: if a previous OpMode's onStop() didn't run
        // (driver force-stopped, or a prior crash skipped cleanup) and a new
        // LimelightDistance calls startReading() again while an old thread from a
        // DIFFERENT instance is still alive, they'll both be hitting the same
        // physical device. This guard only protects against double-starting the
        // SAME instance; it can't fix a leaked thread from a previous instance,
        // but stopReading() below now interrupts+joins so that leak shouldn't happen.
        if (running) return;
        running = true;

        pollingThread = new Thread(() -> {
            while (running && !Thread.currentThread().isInterrupted()) {
                try {
                    LLResult result = limelight.getLatestResult();
                    long now = System.currentTimeMillis();

                    if (result != null && result.isValid()) {
                        tx = result.getTx();
                        ty = result.getTy();
                        ta = result.getTa();

                        lastUpdateTime = now;
                        connectionStatus = "Connected";
                    }

                    if (now - lastUpdateTime > STALE_TIMEOUT_MS) {
                        tx = null;
                        ty = null;
                        ta = null;
                        targetVisible = false;
                        horizontalAngle = 0.0;
                        verticalAngle = 0.0;
                        smoothingInitialized = false; // next reacquisition seeds fresh, not blended from 0
                        connectionStatus = "No data (stale)";
                    } else {
                        targetVisible = ta != null && ta > 0.0;
                        if (targetVisible) {
                            if (!smoothingInitialized) {
                                // First good frame of this sighting -- seed directly from
                                // the raw reading instead of blending from whatever
                                // horizontalAngle/verticalAngle last held (0.0 after a
                                // stale reset). Avoids a multi-frame "ramp up" transient
                                // that otherwise reports the wrong distance right after
                                // every reacquisition.
                                horizontalAngle = tx != null ? tx : 0.0;
                                verticalAngle = ty != null ? ty : 0.0;
                                smoothingInitialized = true;
                            } else {
                                horizontalAngle = smooth(horizontalAngle, tx != null ? tx : horizontalAngle);
                                verticalAngle = smooth(verticalAngle, ty != null ? ty : verticalAngle);
                            }
                        }
                    }

                    lastPollError = null;
                    Thread.sleep(10); // small delay to reduce CPU usage
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                } catch (Exception e) {
                    // Previously uncaught: any non-InterruptedException here (e.g. the
                    // Limelight3A object throwing because the hardware map/USB
                    // connection was torn down by a stale thread from a prior OpMode)
                    // would kill this thread silently. Now it's caught, logged to
                    // telemetry via lastPollError, and the loop keeps retrying instead
                    // of dying — so a transient read failure degrades to "stale data"
                    // instead of a permanently dead reader.
                    lastPollError = e.getClass().getSimpleName() + ": " + e.getMessage();
                    connectionStatus = "Poll error (see lastPollError)";
                    try {
                        Thread.sleep(50); // back off a bit longer after an error
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        });
        pollingThread.start();
    }

    public void stopReading() {
        running = false;
        if (pollingThread != null) {
            pollingThread.interrupt();
            try {
                // Wait for the thread to actually exit before returning. Without this,
                // stopReading() can return while the old thread is still mid-iteration,
                // and the next OpMode's onInit() can create a second reader on the same
                // device before the first one has really let go of it.
                pollingThread.join(200);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
        }
        try {
            limelight.stop();
        } catch (Exception e) {
            // Device may already be torn down (e.g. OpMode ending abnormally). Don't
            // let a failure here mask/replace whatever crash triggered onStop().
        }
        connectionStatus = "Stopped";
    }

    /**
     * Straight-line (Euclidean) distance to the visible ball, in inches.
     * Returns null if no ball is visible or data is stale.
     */
    public Double getEuclideanDistance() {
        if (ty == null) return null; // null means no current/recent reading at all -- not a smoothing concern

        // Use the exponentially-smoothed vertical angle, not raw ty. Raw ty
        // carries full per-frame detection noise straight into the distance
        // number; verticalAngle already filters that out (see SMOOTHING_FACTOR)
        // but was previously computed and then never actually used here.
        double smoothedTy = verticalAngle;
        double totalAngleDeg = LIMELIGHT_ANGLE_DEGREES + (TY_SIGN * smoothedTy);
        double verticalDistance = LIMELIGHT_HEIGHT_INCHES - BALL_HEIGHT_INCHES;

        // Near 0/180 degrees: camera looking parallel to the floor, tan() -> 0,
        // so horizontalDistance -> infinity. This IS a genuine divide-by-zero
        // case -- bail out.
        if (Math.abs(totalAngleDeg) < 1e-6 || Math.abs(totalAngleDeg - 180.0) < 1e-6) {
            return null;
        }

        // Near 90 degrees: camera looking straight down at the ball. This is
        // NOT a divide-by-zero case -- tan(90) blowing up correctly drives
        // horizontalDistance toward 0, meaning the ball is directly below the
        // lens. With a mount angle near/at 90 (straight-down camera) this is
        // the single most common good reading (ball centered in frame, ty≈0),
        // so it's handled explicitly instead of being discarded as null.
        if (Math.abs(totalAngleDeg - 90.0) < 1e-6) {
            return verticalDistance;
        }

        double totalAngleRad = Math.toRadians(totalAngleDeg);
        double horizontalDistance = verticalDistance / Math.tan(totalAngleRad);

        return Math.sqrt(horizontalDistance * horizontalDistance + verticalDistance * verticalDistance);
    }

    public Double getDistance() {
        return getEuclideanDistance();
    }

    /**
     * Adds Limelight diagnostic telemetry lines (status, tx/ty/ta, angles,
     * distance) to the telemetry object passed to setTelemetry(). Call once
     * per loop from your OpMode.
     *
     * IMPORTANT: this does NOT call telemetry.update() itself -- it only adds
     * data. Previously it did, which meant calling this before adding your
     * own telemetry lines caused two separate flushes per loop (Limelight
     * data in one packet, your OpMode's lines in another), so the Driver
     * Station display alternated between them instead of showing both
     * together. Now the caller is responsible for calling telemetry.update()
     * once, after all telemetry.addData() calls for the cycle are done.
     */
    public void update() {
        if (opModeTelemetry != null) {
            addTelemetry(opModeTelemetry);
        }
    }

    private void addTelemetry(Telemetry telemetry) {
        Double euclideanDistance = getEuclideanDistance();
        telemetry.addData("Limelight Status", connectionStatus);
        telemetry.addData("Target Visible", targetVisible);
        telemetry.addData("tx", tx != null ? String.format("%.2f", tx) : "N/A");
        telemetry.addData("ty", ty != null ? String.format("%.2f", ty) : "N/A");
        telemetry.addData("ta", ta != null ? String.format("%.2f", ta) : "N/A");
        telemetry.addData("Horizontal Angle", String.format("%.2f", horizontalAngle));
        telemetry.addData("Vertical Angle", String.format("%.2f", verticalAngle));
        telemetry.addData("Euclidean Distance (in)", euclideanDistance != null ? String.format("%.2f", euclideanDistance) : "N/A");
        if (lastPollError != null) {
            telemetry.addData("Last Poll Error", lastPollError);
        }
    }

    private double smooth(double oldVal, double newVal) {
        return oldVal * (1.0 - SMOOTHING_FACTOR) + newVal * SMOOTHING_FACTOR;
    }

    // ------------------- GETTERS -------------------

    public Double getTx() { return tx; }
    public Double getTy() { return ty; }
    public Double getTa() { return ta; }
    public double getHorizontalAngle() { return horizontalAngle; }
    public double getVerticalAngle() { return verticalAngle; }
    public boolean isTargetVisible() { return targetVisible; }
    public String getStatus() { return connectionStatus; }
    public String getLastPollError() { return lastPollError; }
}