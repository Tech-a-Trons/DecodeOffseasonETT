package org.firstinspires.ftc.teamcode.Reyansh.Subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static java.lang.Math.atan2;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;

public class TurretTrack implements Subsystem {

    public static final TurretTrack INSTANCE = new TurretTrack();

    private ControlSystem controller;
    ServoGroup turret = new ServoGroup(
            new ServoEx("turret1"),
            new ServoEx("turret2")
    );

    Pose CachedPose = null;

    public static void init(HardwareMap hardwareMap) {
        ServoGroup turret = new ServoGroup(
                new ServoEx("turret1"),
                new ServoEx("turret2")
        );
    }

    public void configure() {
        controller = ControlSystem.builder()
                .posPid(0.001, 0.0, 0.0)
                .basicFF(0.003, 0.08, 0.0)
                .build();

        controller.setGoal(new KineticState(0.0));
    }

    double yt = 121 - 72;
    double xt = 121 - 72;


    public void TurretTrack() {
        follower.update();

        Pose cachedPose = PedroComponent.follower().getPose();

        double x = cachedPose.getY() - 72;
        double y = cachedPose.getX() - 72;
        double heading = (cachedPose.getHeading() + 360) % 360;
        double angle = atan2(yt - y, xt - x);
        angle = angle - heading;
        double targetangle = ((angle + 360) % 360);
        double Position = targetangle / 360;

        controller.setGoal(new KineticState(Position));
        turret.setPosition(controller.calculate(new KineticState(
                turret.getPosition()))
        );

    }
//        telemetry.addData(String.valueOf(Position), "Position");
//
////        Position = clamp(Position, 0.0, 1.0);
//        telemetry.addData(String.valueOf(x), "x");
//        telemetry.addData(String.valueOf(y), "y");
//        telemetry.addData(String.valueOf(heading), "heading");
//        telemetry.addData(String.valueOf(targetangle), "targetangle");

    /// /        telemetry.addData(String.valueOf(hood.getVelocity()), "distance");


//        if (gamepad1.aWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 2000.0));
//        } else if (gamepad1.bWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 0.0));
//        } else if (gamepad1.xWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 1000.0));
//        }
    @Override
    public void periodic() {
        TurretTrack();
    }



}