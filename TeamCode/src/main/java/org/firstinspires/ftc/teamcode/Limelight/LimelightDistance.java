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
    private static final double SMOOTHING_FACTOR = 0.2;      // exponential smoothing for tx/ty (0-1, higher = less smoothing)
    private static final long STALE_TIMEOUT_MS = 500;        // discard readings older than this
    private static final int POLL_RATE_HZ = 100;
    private static final int PIPELINE_INDEX = 0;               // set to your ball-detector pipeline index

    // --- Mount geometry — measure on your robot ---
    private static final double LIMELIGHT_HEIGHT_INCHES = 10.0;  // lens height off floor
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

    private Thread pollingThread;
    private volatile boolean running = false;

    public LimelightDistance(HardwareMap hardwareMap) {
        this(hardwareMap, "Limelight");
    }

    public LimelightDistance(HardwareMap hardwareMap, String deviceName) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.start();
    }

    public void setTelemetry(Telemetry telemetry) {
        this.opModeTelemetry = telemetry;
    }

    public void startReading() {
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
                        connectionStatus = "No data (stale)";
                    } else {
                        targetVisible = ta != null && ta > 0.0;
                        horizontalAngle = smooth(horizontalAngle, tx != null ? tx : 0.0);
                        verticalAngle = smooth(verticalAngle, ty != null ? ty : 0.0);
                    }

                    Thread.sleep(10); // small delay to reduce CPU usage
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        pollingThread.start();
    }

    public void stopReading() {
        running = false;
        if (pollingThread != null) {
            pollingThread.interrupt();
        }
        limelight.stop();
        connectionStatus = "Stopped";
    }

    /**
     * Straight-line (Euclidean) distance to the visible ball, in inches.
     * Returns null if no ball is visible or data is stale.
     */
    public Double getEuclideanDistance() {
        if (ty == null) return null;

        double totalAngleDeg = LIMELIGHT_ANGLE_DEGREES + (TY_SIGN * ty);
        if (Math.abs(totalAngleDeg) < 1e-6 || Math.abs(totalAngleDeg - 90.0) < 1e-6) {
            return null; // avoid division by zero / tan undefined near 90 degrees
        }

        double totalAngleRad = Math.toRadians(totalAngleDeg);
        double verticalDistance = LIMELIGHT_HEIGHT_INCHES - BALL_HEIGHT_INCHES;
        double horizontalDistance = verticalDistance / Math.tan(totalAngleRad);

        return Math.sqrt(horizontalDistance * horizontalDistance + verticalDistance * verticalDistance);
    }

    public Double getDistance() {
        return getEuclideanDistance();
    }

    /** Call once per loop from your OpMode if you want telemetry printed automatically. */
    public void update() {
        if (opModeTelemetry != null) {
            addTelemetry(opModeTelemetry);
            opModeTelemetry.update();
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
}