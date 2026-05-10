package org.firstinspires.ftc.teamcode.Reyansh.TestTeles;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Turret test 2")
public class TurretPIDTele extends NextFTCOpMode {
    public TurretPIDTele() {
        addComponents(new PedroComponent(Constants::createFollower));
    }

    private ControlSystem controller;
    public MotorGroup outtake;
    Pose CachedPose = null;
    @Override
    public void onInit() {
        if (hardwareMap != null) {
            outtake = new MotorGroup(
                    new MotorEx((DcMotorEx) hardwareMap.get("outtakeleft")),
                    new MotorEx((DcMotorEx) hardwareMap.get("outtakeright")).reversed()
            );
        } else {
            telemetry.addData("Error", "HardwareMap is null! Check configuration.");
        }
        controller = ControlSystem.builder()
                .velPid(0.001, 0.0, 0.0)
                .basicFF(0.003, 0.08, 0.0)
                .build();

        controller.setGoal(new KineticState(0.0, 0.0));
    }
    double yt = 121 - 72;
    double xt = 121 - 72;
    @Override
    public void onUpdate() {
        PedroComponent.follower().setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );


            Pose cachedPose = PedroComponent.follower().getPose();

        double x = cachedPose.getY() - 72;
        double y = cachedPose.getX() - 72;

        double distance = Math.sqrt(Math.pow(yt-y, 2)  + Math.pow(xt-x, 2));
        double Velocity = (distance * 200) + 200;
        telemetry.addData(String.valueOf(Velocity), "Velocity");
        telemetry.addData(String.valueOf(x), "x");
        telemetry.addData(String.valueOf(y), "y");
        telemetry.addData(String.valueOf(distance), "distance");
        telemetry.addData(String.valueOf(outtake.getVelocity()), "distance");


        controller.setGoal(new KineticState(0.0, Velocity));
//        if (gamepad1.aWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 2000.0));
//        } else if (gamepad1.bWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 0.0));
//        } else if (gamepad1.xWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 1000.0));
//        }
        outtake.setPower(controller.calculate(new KineticState(
                outtake.getCurrentPosition(),
                outtake.getVelocity()))
        );
    }



}