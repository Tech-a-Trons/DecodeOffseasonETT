package org.firstinspires.ftc.teamcode.Reyansh.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.ServoEx;

public class Hood implements Subsystem {
    public static final Hood INSTANCE = new Hood();
    private ControlSystem controller;
    ServoEx hood = hardwareMap.get(ServoEx.class, "hood");

    public void init(HardwareMap hardwareMap) {
        // initialization logic (runs on init)
        ServoEx hood = hardwareMap.get(ServoEx.class, "hood");
    }
    Pose CachedPose = null;

    public void configure() {

        controller = ControlSystem.builder()
                .posPid(0.001, 0.0, 0.0)
                .basicFF(0.003, 0.08, 0.0)
                .build();

        controller.setGoal(new KineticState(0.0));
    }

    double yt = 121 - 72;
    double xt = 121 - 72;


    public void HoodOn() {
        follower.update();
//        telemetry.update();


        Pose cachedPose = PedroComponent.follower().getPose();

        double x = cachedPose.getY() - 72;
        double y = cachedPose.getX() - 72;

        double distance = Math.sqrt(Math.pow(yt-y, 2)  + Math.pow(xt-x, 2));
        double Position = (distance * 0.01) + 0.2;
//        telemetry.addData(String.valueOf(Position), "Position");
//
////        Position = clamp(Position, 0.0, 1.0);
//        telemetry.addData(String.valueOf(x), "x");
//        telemetry.addData(String.valueOf(y), "y");
//        telemetry.addData(String.valueOf(distance), "distance");
////        telemetry.addData(String.valueOf(hood.getVelocity()), "distance");


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

    //    public void off() {
//        controller.setGoal(new KineticState(0.0, 0.0));
//        hood.setPosition(controller.calculate(new KineticState(
//                hood.getPosition()))
//
//        );
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
        HoodOn();
    }
    }


