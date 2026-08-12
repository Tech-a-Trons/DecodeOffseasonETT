package org.firstinspires.ftc.teamcode.Reyansh.Limelight;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Limelight.LimelightDistance;
import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.Intaker;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp
public class object_following extends NextFTCOpMode {

    public object_following() {
        addComponents(
                new SubsystemComponent(
                        Intaker.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    // --- Tunable values ---
    private static final double TX_DEADBAND = 3;      // degrees, tolerance to ENTER aligned/locked state
    private static final double SEARCH_POWER = 0.2;      // power used when searching for target
    private static final double MAX_POWER = 0.5;          // clamp

    // Plain proportional gains, no PID/ControlSystem.
    private static final double kP_turn = 0.02;   // turnPower = kP_turn * tx (used ONLY while aligning, pre-lock)
    private static final double kP_drive = 0.02;  // drivePower = kP_drive * distanceInches

    // Minimum effective power to actually rotate the drivetrain. Below this,
    // static friction wins and the wheels just sit there while kP_turn*tx
    // keeps shrinking toward the deadband edge -- the robot never physically
    // closes the last couple degrees and hasLockedOn never flips true. This
    // floors the turn command so it can't stall out right where it matters.
    private static final double MIN_TURN_POWER = 0.12;

    // Fallback forward power used once locked-on if a distance reading isn't
    // available yet for this cycle. Keeps the robot creeping in instead of
    // hard-stopping every time getDistance() returns null for a tick.
    private static final double LOCKED_FALLBACK_POWER = 0.15;

    double tx = 0;
    double lastKnownTx = 0; // last non-null tx we saw, used to pick a search direction
    Double distanceInches = null;
    boolean valid = false;

    private LimelightDistance ballDistance;
    private MotorGroup leftDrive;
    private MotorGroup rightDrive;
    private boolean tracking = false;
    // True once tx has entered TX_DEADBAND. Once true, stays true and the robot
    // drives straight forward regardless of tx -- only reset by losing tracking
    // or losing the target entirely (!valid). No steering correction is applied
    // once locked; alignment only happens before lock.
    private boolean hasLockedOn = false;
    private boolean readingStarted = false; // guards onStop() from tearing down a reader that never started
    // If hardware init fails partway through, the rest of onInit() and every
    // onUpdate() call becomes a no-op instead of NPEing on a null field. The
    // driver still sees why via telemetry instead of a crash screen.
    private boolean initFailed = false;
    private String initError = null;

    @Override
    public void onInit() {
        try {
            ballDistance = new LimelightDistance(hardwareMap, "limelight3A");
            ballDistance.setTelemetry(telemetry);

            // Split left/right so turning is differential (opposite-signed power on
            // each side actually rotates the robot) instead of a single MotorGroup
            // where every motor gets the same value and the robot can only drive
            // straight forward/backward. fr/br stay .reversed() to compensate for
            // physical mounting -- if the robot drives backward when commanded
            // forward, reverse the LEFT side instead of flipping this back, so you
            // don't end up with individual wheels fighting each other.
            leftDrive = new MotorGroup(
                    new MotorEx((DcMotorEx) hardwareMap.get("fl")),
                    new MotorEx((DcMotorEx) hardwareMap.get("bl"))
            );
            rightDrive = new MotorGroup(
                    new MotorEx((DcMotorEx) hardwareMap.get("fr")).reversed(),
                    new MotorEx((DcMotorEx) hardwareMap.get("br")).reversed()
            );
        } catch (Exception e) {
            // Most common real-world cause of "robot crashes on init": a hardware
            // config name here (fl/bl/fr/br/limelight3A) doesn't match the active
            // robot config on the Driver Station. Previously this threw straight out
            // of onInit() and the SDK shows a bare exception. Now it's caught, the
            // OpMode goes into a safe no-op state, and the message is on telemetry.
            initFailed = true;
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
            telemetry.addLine("INIT FAILED — check hardware config names match: fl, bl, fr, br, limelight3A");
            telemetry.addData("Error", initError);
            telemetry.update();
        }
    }

    @Override
    public void onStartButtonPressed() {
        if (initFailed || ballDistance == null) return;
        ballDistance.startReading();
        readingStarted = true;
    }

    @Override
    public void onStop() {
        // Without this, LimelightDistance's background reader thread keeps running
        // after this OpMode ends. The next OpMode's onInit() then creates a second
        // LimelightDistance/thread on top of it, so two threads can end up calling
        // getLatestResult() concurrently, and the old thread may throw once the
        // hardware map it was built against gets torn down.
        //
        // Guarded on readingStarted: if the driver hits Init then Stop without ever
        // pressing Start, startReading() (and whatever thread/state it sets up) never
        // ran, so calling stopReading() here would be tearing down something that
        // was never created.
        if (readingStarted && ballDistance != null) {
            ballDistance.stopReading();
        }
    }

    @Override
    public void onUpdate() {
        // If hardware init failed, leftDrive/rightDrive/ballDistance may be null.
        // Bail out early instead of NPEing every loop tick, which is what a "robot
        // crashes" report after a bad hardware config name usually actually is --
        // init throws, OpMode limps into onUpdate() anyway, and it NPEs on the very
        // first line that touches a null hardware object.
        if (initFailed) {
            telemetry.addLine("INIT FAILED — see Error below. Stop and fix config, then re-init.");
            telemetry.addData("Error", initError);
            telemetry.update();
            return;
        }

        // Adds LimelightDistance's own diagnostic telemetry (status, tx/ty/ta,
        // angles, distance) to the telemetry buffer. Data freshness doesn't
        // depend on this call -- tx/ty/distance getters below always read the
        // latest volatile values pushed by the background polling thread
        // regardless of whether/when update() runs. It no longer flushes
        // telemetry itself, so it's safe to call here and still add our own
        // lines below, all sent together in the single telemetry.update() at
        // the bottom of this method.
        ballDistance.update();

        // --- Read from extractor (thread-safe, never calls getLatestResult() directly here) ---
        valid = ballDistance.isTargetVisible();
        Double txReading = ballDistance.getTx();
        if (txReading != null) {
            tx = txReading;
            if (valid) {
                lastKnownTx = txReading;
            }
        }
        distanceInches = ballDistance.getDistance();

        if (gamepad1.a) {
            tracking = true;
        }
        if (gamepad1.b) {
            tracking = false;
        }

        boolean withinTightDeadband = Math.abs(tx) < TX_DEADBAND;

        double leftPower;
        double rightPower;

        if (!tracking) {
            // Operator has tracking off: stop and idle, and drop lock so the next
            // tracking-on cycle re-aligns from scratch instead of assuming lock.
            leftPower = 0.0;
            rightPower = 0.0;
            hasLockedOn = false;
            Intaker.INSTANCE.stop();
        } else if (!valid) {
            // Tracking is on but there is currently no target at all: rotate in place
            // toward whichever side we last saw the target on, instead of sitting
            // dead. Differential (opposite-signed) power actually turns the robot.
            // Intake stays off while searching. Losing the target entirely also
            // drops lock -- this is now the ONLY way lock is lost besides tracking
            // being turned off; drifting out of alignment while locked no longer
            // matters.
            double turnPower = (lastKnownTx < 0) ? -SEARCH_POWER : SEARCH_POWER;
            turnPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, turnPower));
            leftPower = turnPower;
            rightPower = -turnPower;
            hasLockedOn = false;
            Intaker.INSTANCE.stop();
        } else if (!hasLockedOn && !withinTightDeadband) {
            // Not locked on yet and outside the tight deadband: turn in place to
            // align. Plain proportional turn (no PID) -- turnPower = kP_turn * tx.
            // This is the ONLY place kP_turn is used -- once locked on, we stop
            // steering entirely and drive straight (see below).
            double turnPower = kP_turn * tx;
            turnPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, turnPower));
            // Floor the magnitude so a weak proportional command near the
            // deadband edge can't stall out from static friction -- without
            // this the robot can sit forever just outside TX_DEADBAND,
            // commanding real but too-small-to-move power, and hasLockedOn
            // never flips true even though tracking is on.
            if (Math.abs(turnPower) > 1e-6 && Math.abs(turnPower) < MIN_TURN_POWER) {
                turnPower = Math.copySign(MIN_TURN_POWER, turnPower);
            }
            leftPower = turnPower;
            rightPower = -turnPower;
            Intaker.INSTANCE.stop();
        } else {
            // Locked on (or just entered the deadband) and target visible: drive
            // straight forward and intake continuously, with NO steering
            // correction -- once locked, tx is ignored entirely for driving
            // purposes. Only losing tracking or losing the target (!valid) resets
            // hasLockedOn; drifting out of tx alignment no longer matters.
            hasLockedOn = true;
            double drivePower = (distanceInches != null)
                    ? kP_drive * distanceInches
                    : LOCKED_FALLBACK_POWER; // no distance reading yet this cycle -- creep forward instead of stopping
            drivePower = Math.max(-MAX_POWER, Math.min(MAX_POWER, drivePower));
            leftPower = drivePower;
            rightPower = drivePower;
            Intaker.INSTANCE.forward();
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        telemetry.addData("Tracking", tracking);
        telemetry.addData("Locked On", hasLockedOn);
        telemetry.addData("Valid", valid);
        telemetry.addData("tx", tx);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Distance", distanceInches);
        telemetry.update(); // flush buffered telemetry to the Driver Station; addData alone doesn't send it
    }
}