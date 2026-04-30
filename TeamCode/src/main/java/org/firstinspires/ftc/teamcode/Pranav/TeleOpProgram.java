package org.firstinspires.ftc.teamcode.Pranav;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pranav.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Pranav.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Pranav.Subsystems.ShooterPID;
import org.firstinspires.ftc.teamcode.Pranav.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Pranav.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Pranav Tele")
public class TeleOpProgram extends NextFTCOpMode {
    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Transfer.INSTANCE, ShooterPID.INSTANCE, Turret.INSTANCE, Hood.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    Transfer transfer;
    Intake intake;
    ShooterPID shooterPID;
    Turret turret;
    Hood hood;
    Follower follower;
    Pose start;

    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start == null ? new Pose() : start);
        follower.update();
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);
        shooterPID = new ShooterPID("red");
        hood = new Hood("red");
        turret = new Turret("red");
    }

    @Override
    public void onStartButtonPressed() {
//        if (gamepad1.right_bumper) {
//            shooterPID.INSTANCE.close();
//            if (gamepad1.dpad_up) {
//                shooterPID.INSTANCE.close();
//                intake.INSTANCE.into();
//                transfer.INSTANCE.into();
//            }
//        }

        if (gamepad1.left_bumper) {
            shooterPID.INSTANCE.shoot();
            if (gamepad1.dpad_up) {
                shooterPID.INSTANCE.shoot();
                intake.INSTANCE.into();
                transfer.INSTANCE.into();
            }
        }

        if (gamepad1.left_trigger_pressed) {
            shooterPID.INSTANCE.stop();
            intake.INSTANCE.off();
            transfer.INSTANCE.off();
        }

        if (gamepad1.right_bumper) {
            intake.INSTANCE.into();
            transfer.INSTANCE.little();
            if (gamepad1.right_trigger_pressed) {
                transfer.INSTANCE.off();
                intake.INSTANCE.into();
            }
        }

        if (gamepad1.dpad_right) {
            shooterPID.INSTANCE.repel();
            intake.INSTANCE.out();
            transfer.INSTANCE.out();
        }
    }

    @Override
    public void onUpdate() {
        follower.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        hood.setHood();
    }
}
