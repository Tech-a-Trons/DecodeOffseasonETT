package org.firstinspires.ftc.teamcode.Reyansh.TestTeles;

import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.ServoEx;
@TeleOp(name = "HOod test")

public class HoodTele extends NextFTCOpMode {
    public HoodTele() {
        addComponents(new PedroComponent(Constants::createFollower));
    }

    ServoEx hood;
    Pose CachedPose = null;
    double yt = 121 - 72;
    double xt = 121 - 72;
    private ControlSystem controller;

    @Override
    public void onInit() {
        if (hardwareMap != null) {
            hood = new ServoEx(hardwareMap.get(Servo.class, "hood"));
        } else {
            telemetry.addData("Error", "HardwareMap is null! Check configuration.");
        }

        controller = ControlSystem.builder()
                .posPid(0.001, 0.0, 0.0)
                .basicFF(0.003, 0.08, 0.0)
                .build();

        controller.setGoal(new KineticState(0.0));
    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().update();
        telemetry.update();

        PedroComponent.follower().setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );

        Pose cachedPose = PedroComponent.follower().getPose();

        double x = cachedPose.getY() - 72;
        double y = cachedPose.getX() - 72;

        double distance = Math.sqrt(Math.pow(yt - y, 2) + Math.pow(xt - x, 2));
        double Position = (distance * 0.01) + 0.2;
        telemetry.addData(String.valueOf(Position), "Position");

//        Position = clamp(Position, 0.0, 1.0);
        telemetry.addData(String.valueOf(x), "x");
        telemetry.addData(String.valueOf(y), "y");
        telemetry.addData(String.valueOf(distance), "distance");
//        telemetry.addData(String.valueOf(hood.getVelocity()), "distance");


        controller.setGoal(new KineticState(Position));
//        if (gamepad1.aWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 2000.0));
//        } else if (gamepad1.bWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 0.0));
//        } else if (gamepad1.xWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 1000.0));
//        }
        hood.setPosition(controller.calculate(new KineticState(
                hood.getPosition()))
        );
    }


}