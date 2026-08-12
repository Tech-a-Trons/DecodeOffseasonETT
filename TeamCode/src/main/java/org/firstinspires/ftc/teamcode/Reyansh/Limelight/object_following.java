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
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

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
    private static final double TX_DEADBAND = 3;      // degrees, alignment tolerance
    private static final double SEARCH_POWER = 0.2;      // power used when searching for target
    private static final double MAX_POWER = 0.7;          // clamp

    // PID gains for turn-to-target (error = tx, goal = 0)
    private static final double kP_turn = 0.03;
    private static final double kI_turn = 0.0;
    private static final double kD_turn = 0.001;

    // PID gains for drive-toward-target (error = distanceInches, goal = 0 -> drive until reached)
    private static final double kP_drive = 0.02;
    private static final double kI_drive = 0.0;
    private static final double kD_drive = 0.0005;

    double tx = 0;
    double lastKnownTx = 0; // last non-null tx we saw, used to pick a search direction
    Double distanceInches = null;
    boolean valid = false;

    private LimelightDistance ballDistance;
    private MotorGroup drivetrain;
    private ControlSystem turnController;
    private ControlSystem driveController;
    private boolean tracking = false;
    private boolean readingStarted = false; // guards onStop() from tearing down a reader that never started

    @Override
    public void onInit() {
        ballDistance = new LimelightDistance(hardwareMap, "limelight3A");
        ballDistance.setTelemetry(telemetry);

        // All four drive motors are driven with the SAME signed power value via
        // drivetrain.setPower(power) below. fr/br are wrapped in .reversed() here
        // purely to compensate for the fact that they are physically mounted facing
        // the opposite way from fl/bl. With that compensation in place, a single
        // positive power value drives the whole robot straight forward, and a single
        // negative value drives it straight backward -- there is no independent
        // left/right power split in this file, so "power" does not currently turn
        // the robot in place, it only changes forward/backward drive speed.
        //
        // If the robot drives backward when you command forward power:
        //   1. Do NOT just flip the reversed() call to a different pair of motors --
        //      that only masks the symptom and can leave individual wheels fighting
        //      each other.
        //   2. Instead, verify each wheel's direction one at a time (spin each motor
        //      alone at a small positive power and confirm the wheel drives the robot
        //      forward-that-wheel's-corner). Reversed() should end up applied to
        //      exactly the motors that are physically mounted backward relative to
        //      the others, nothing else.
        //   3. If you later split this into separate left/right MotorGroups so that
        //      turnController can actually rotate the robot in place, keep the sign
        //      convention consistent: left = drive - turn, right = drive + turn (or
        //      vice versa) -- reversing the "wrong" side after that split will invert
        //      turning direction instead of fixing straight-line direction.
        drivetrain = new MotorGroup(
                new MotorEx((DcMotorEx) hardwareMap.get("fl")),
                new MotorEx((DcMotorEx) hardwareMap.get("bl")),
                new MotorEx((DcMotorEx) hardwareMap.get("fr")).reversed(),
                new MotorEx((DcMotorEx) hardwareMap.get("br")).reversed()
        );

        // PID drives tx toward 0 (i.e. target centered in frame)
        turnController = ControlSystem.builder()
                .posPid(kP_turn, kI_turn, kD_turn)
                .build();
        turnController.setGoal(new KineticState(0.0));

        // PID drives distanceInches toward 0 (i.e. keep closing the gap to the target)
        driveController = ControlSystem.builder()
                .posPid(kP_drive, kI_drive, kD_drive)
                .build();
        driveController.setGoal(new KineticState(0.0));
    }

    @Override
    public void onStartButtonPressed() {
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
        if (readingStarted) {
            ballDistance.stopReading();
        }
    }

    @Override
    public void onUpdate() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        // --- Read from extractor ---
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

        boolean aligned = Math.abs(tx) < TX_DEADBAND;

        double power;

        if (!tracking) {
            // Operator has tracking off: stop and idle.
            power = 0.0;
            Intaker.INSTANCE.stop();
        } else if (!valid) {
            // Tracking is on but there is currently no target at all: creep/rotate
            // at SEARCH_POWER toward whichever side we last saw the target on,
            // instead of sitting dead. Intake stays off while searching.
            power = (lastKnownTx < 0) ? -SEARCH_POWER : SEARCH_POWER;
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
            Intaker.INSTANCE.stop();
        } else if (!aligned) {
            // Target visible, not yet aligned: PID toward tx = 0, intake off until aligned.
            // This only needs tx, so it runs whether or not distanceInches is known yet.
            power = turnController.calculate(new KineticState(tx));
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
            Intaker.INSTANCE.stop();
        } else if (distanceInches == null) {
            // Aligned, target visible, but the extractor hasn't produced a distance
            // reading yet (e.g. ty out of the valid range for this cycle). Hold
            // position rather than driving blind or searching away from a target
            // we can actually see and are centered on.
            power = 0.0;
            Intaker.INSTANCE.stop();
        } else {
            // Aligned and target visible: PID drives toward the target using distance
            // from the extractor, intake runs the whole time. Keeps closing the gap
            // until the target disappears from view (intaked) or tracking is turned off.
            power = driveController.calculate(new KineticState(distanceInches));
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
            Intaker.INSTANCE.forward();
        }

        drivetrain.setPower(power);

        telemetry.addData("Tracking", tracking);
        telemetry.addData("Aligned", aligned);
        telemetry.addData("Power", power);
        telemetry.addData("Distance", distanceInches);
        ballDistance.update();
        telemetry.update(); // flush buffered telemetry to the Driver Station; addData alone doesn't send it
    }
}