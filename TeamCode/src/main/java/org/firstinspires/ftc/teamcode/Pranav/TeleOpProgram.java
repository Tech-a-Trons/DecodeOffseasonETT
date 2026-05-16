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

    Transfer transfer;
    Intake intake;
    ShooterPID shooterPID;
    Turret turret;
    Hood hood;
    Follower follower;
    Pose start;

    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
    private final MotorEx frontRightMotor = new MotorEx("fr");
    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
    private final MotorEx backRightMotor = new MotorEx("br");


    @Override
    public void onInit() {
        transfer = new Transfer();
        intake = new Intake();
        shooterPID = new ShooterPID("red");
        hood = new Hood("red");
        turret = new Turret("red");
        intake.init(hardwareMap);
        transfer.init(hardwareMap);
    }

    @Override
    public void onStartButtonPressed() {
        PedroComponent.follower().setStartingPose(new Pose(72, 0, 0));
        PedroComponent.follower().startTeleOpDrive();
        PedroComponent.follower().update();
//        if (gamepad1.right_bumper) {
//            shooterPID.INSTANCE.close();
//            if (gamepad1.dpad_up) {
//                shooterPID.INSTANCE.close();
//                intake.INSTANCE.into();
//                transfer.INSTANCE.into();
//            }
//        }

        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

//        if (gamepad1.left_bumper) {
//            shooterPID.INSTANCE.shoot();
//            if (gamepad1.dpad_up) {
//                shooterPID.INSTANCE.shoot();
//                intake.INSTANCE.into();
//                transfer.INSTANCE.into();
//            }
//        }

        Gamepads.gamepad1().leftBumper()
                        .whenBecomesTrue(() -> {
                            ShooterPID.INSTANCE.shoot();
                            Gamepads.gamepad1().a()
                                    .whenBecomesTrue(() -> {
                                        Intake.INSTANCE.into();
                                        Transfer.INSTANCE.into();
                                            }

                                    );
                                }

                        );

        Gamepads.gamepad1().leftTrigger().greaterThan(0.01)
//                .whenBecomesTrue(ShooterPID.INSTANCE.stop())
//                .whenBecomesTrue(Intake.INSTANCE.off())
//                .whenBecomesTrue(Transfer.INSTANCE.off());
        .whenBecomesTrue(() -> {
            ShooterPID.INSTANCE.stop();
            Intake.INSTANCE.off();
            Transfer.INSTANCE.off();
        });


//        if (gamepad1.right_bumper) {
//            intake.INSTANCE.into();
//            transfer.INSTANCE.little();
//            if (gamepad1.right_trigger_pressed) {
//                transfer.INSTANCE.off();
//                intake.INSTANCE.into();
//            }
//        }

        Gamepads.gamepad1().rightBumper()
//                        .whenBecomesTrue(Intake.INSTANCE.into())
//                                .whenBecomesTrue(Transfer.INSTANCE.little())
        .whenBecomesTrue(() -> {
            Intake.INSTANCE.into();
            Transfer.INSTANCE.little();
            Gamepads.gamepad1().rightTrigger().greaterThan(0.01)
                    .whenBecomesTrue(() -> {
                        Transfer.INSTANCE.off();
                            }
                    );
                }
        );
//                Gamepads.gamepad1().rightTrigger().greaterThan(0.01)
//                                .whenBecomesTrue(Transfer.INSTANCE.off())
//                                        .whenBecomesTrue(Intake.INSTANCE.into());

//        if (gamepad1.dpad_right) {
//            shooterPID.INSTANCE.repel();
//            intake.INSTANCE.out();
//            transfer.INSTANCE.out();
//        }

//        Gamepads.gamepad1().dpadRight()
//                .whenBecomesTrue(ShooterPID.INSTANCE.repel())
//                .whenBecomesTrue(Intake.INSTANCE.out())
//                .whenBecomesTrue(Transfer.INSTANCE.out());
        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(() -> {
                    ShooterPID.INSTANCE.repel();
                    Intake.INSTANCE.out();
                    Transfer.INSTANCE.out();
                        }
                );
    }

    @Override
    public void onUpdate() {
        Hood.INSTANCE.setHood();
//        Turret.INSTANCE.goalface();
    }
}
