package org.firstinspires.ftc.teamcode.Pranav;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.teamcode.Pranav.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Pranav.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Pranav.Subsystems.ShooterPID;
import org.firstinspires.ftc.teamcode.Pranav.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Pranav.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.threeten.bp.Instant;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Pranav Tele")
public class TeleOpProgram extends NextFTCOpMode {
    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Transfer.INSTANCE, ShooterPID.INSTANCE, Turret.INSTANCE, Hood.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower), new LoopTimeComponent()
                );
    }

    public static Pose sPose; //See ExampleAuto to understand how to use this
//    Transfer transfer;
//    Intake intake;
//    ShooterPID shooterPID;
//
//    Turret turret;
//    Hood hood;
    Follower follower;
//    Pose start;

    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
    private final MotorEx frontRightMotor = new MotorEx("fr");
    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
    private final MotorEx backRightMotor = new MotorEx("br");


    @Override
    public void onInit() {
        Intake.INSTANCE.init(hardwareMap);
        Transfer.INSTANCE.init(hardwareMap);
        ShooterPID.INSTANCE.init("red");
        Hood.INSTANCE.init("red");
        try {
            follower = PedroComponent.follower();
            follower.setStartingPose(sPose == null ? new Pose() : sPose);
            follower.update();
        } catch (Exception e) {
            telemetry.addData("Error", "Follower init failed: " + e.getMessage());
        }

        // Register ALL bindings here, safely outside the update loop
//        Gamepads.gamepad1().leftBumper()
//                .whenBecomesTrue(ShooterPID.INSTANCE.shoot());
//
//        Gamepads.gamepad1().a()
//                .whenBecomesTrue(() -> {
//                    if (gamepad1.left_bumper) {
//                        Intake.INSTANCE.into();
//                        Transfer.INSTANCE.into();
//                    }
//                });
//
//        Gamepads.gamepad1().leftTrigger().greaterThan(0.01)
//                .whenBecomesTrue(() -> {
//                    ShooterPID.INSTANCE.stop();
//                    Intake.INSTANCE.off();
//                    Transfer.INSTANCE.off();
//                });
//
////        Gamepads.gamepad1().rightBumper()
////                .whenBecomesTrue(() -> {
////                    Intake.INSTANCE.into();
////                    Transfer.INSTANCE.little();
////                });
////
////        Gamepads.gamepad1().rightTrigger().greaterThan(0.01)
////                .whenBecomesTrue(() -> {
////                    if (gamepad1.right_bumper) {
////                        Transfer.INSTANCE.off();
////                    }
////                });
//
//        Gamepads.gamepad1().dpadRight()
//                .whenBecomesTrue(() -> {
//                    ShooterPID.INSTANCE.repel(); // fix this too once you see Transfer
//                    Intake.INSTANCE.out();
//                    Transfer.INSTANCE.out();
//                });
    }

    @Override
    public void onStartButtonPressed() {
        PedroComponent.follower().setStartingPose(new Pose(72, 72, 270));
        PedroComponent.follower().startTeleOpDrive();
        PedroComponent.follower().update();

        // Right bumper: start on press, stop on release
//        Gamepads.gamepad1().rightBumper()
//                .whenBecomesTrue(() -> {
//                    Intake.INSTANCE.into();
//                    Transfer.INSTANCE.little();
//                });
//                .whenBecomesFalse(() -> {
//                    Intake.INSTANCE.off();
//                    Transfer.INSTANCE.off();
//                });

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                    ShooterPID.INSTANCE.shoot();
                });

        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> {
                    if (ShooterPID.INSTANCE.velo != 0) {
                        Intake.INSTANCE.into();
                        Transfer.INSTANCE.into();
                    }
                });

        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> {
                    PedroComponent.follower().setPose(new Pose(72, 72, 270));
                });

//        Gamepads.gamepad1().rightTrigger().greaterThan(0.01)
//                .whenBecomesTrue(() -> {
//                            if (gamepad1.right_bumper) {
//                                Transfer.INSTANCE.off();
//                            }
//                                }

                        //);

        Gamepads.gamepad1().leftTrigger().greaterThan(0.0)
                .whenBecomesTrue(() -> {
                    ShooterPID.INSTANCE.stop();
                    Intake.INSTANCE.off();
                    Transfer.INSTANCE.off();
                });

        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(() -> {
                    ShooterPID.INSTANCE.repel();
                    Intake.INSTANCE.out();
                    Transfer.INSTANCE.out();
                });
    }

    // Add this as a class field
//    private boolean bumperWasPressed = false;

    @Override
    public void onUpdate() {
        if (gamepad1.right_bumper) {
            Intake.INSTANCE.into();
            if (gamepad1.right_trigger > 0.01) {
                Transfer.INSTANCE.off();
            } else {
                Transfer.INSTANCE.little();
            }
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
    }
}
