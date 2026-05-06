package org.firstinspires.ftc.teamcode.Reyansh;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.Intaker;
import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.TurretPID;
import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.TurretTrack;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Reyansh's TeleOp")
public class Teleop extends NextFTCOpMode {
    float SlowModeMultiplier = 1;
    ElapsedTime shoottime = new ElapsedTime();

    boolean intaketoggle = false;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    boolean IsShooting = false;
    public Teleop() {

        // Where you add code components that will be used in your opmode, and by putting things here it is reusable
        addComponents(
                // This is where all of the subsystems are called
                new SubsystemComponent(
                        ColorSensor.INSTANCE,
                        Intaker.INSTANCE,
                        Transfer.INSTANCE,
                        Hood.INSTANCE,
                        TurretPID.INSTANCE,
                        TurretTrack.INSTANCE

                ),
                // Allows hardware calls to be done in one instant
                BulkReadComponent.INSTANCE,
                // The Next FTC binding system to attach commands to buttons
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Intaker.INSTANCE.init(hardwareMap);
        Transfer.INSTANCE.init(hardwareMap);
        ColorSensor.INSTANCE.init(hardwareMap);
        TurretPID.INSTANCE.init(hardwareMap);
        TurretTrack.init(hardwareMap);
        Hood.INSTANCE.init(hardwareMap);

    }

    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {
        follower.startTeleopDrive();

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                            intaketoggle = true;

                        }


                );

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                            Intaker.INSTANCE.stop();
                            intaketoggle = false;
                            Transfer.INSTANCE.stop();
                            TurretPID.INSTANCE.stop();

                        }
                );
        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenTrue(
                        () -> {
                            TurretPID.INSTANCE.FlyWheelsOn();

                        });
        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenBecomesTrue(
                        () -> {
                            Transfer.INSTANCE.forward();
                            Intaker.INSTANCE.forward();
                            IsShooting = true;
                            shoottime.reset();
                        });


        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> {
                    follower.setPose(new Pose(72, 72, Math.toRadians(270.0)));
                });


        // Kill Switches
        Gamepads.gamepad2().rightTrigger().greaterThan(0.2)
                .whenTrue(
                        () -> {
                            SlowModeMultiplier = 0;

                        });
        Gamepads.gamepad2().rightTrigger().greaterThan(0.2)
                .whenFalse(
                        () -> {
                            SlowModeMultiplier = 1;

                        });

    }

    @Override
    public void onUpdate() {
        follower.update();
        telemetryM.update();
        if (intaketoggle) {
            Intaker.INSTANCE.forward();
            Transfer.INSTANCE.slight();
            ColorSensor.INSTANCE.IncountBalls();
        } else {
            Intaker.INSTANCE.stop();
            Transfer.INSTANCE.stop();
        }


        if (shoottime.milliseconds() > 200 && IsShooting) {
            Transfer.INSTANCE.stop();
            Intaker.INSTANCE.stop();
            TurretPID.INSTANCE.stop();
            IsShooting = false;
        }
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * SlowModeMultiplier,
                -gamepad1.left_stick_x * SlowModeMultiplier,
                -gamepad1.right_stick_x * SlowModeMultiplier,
                true // Robot Centric
        );
    }

    @Override
    public void onStop() {

    }
}

