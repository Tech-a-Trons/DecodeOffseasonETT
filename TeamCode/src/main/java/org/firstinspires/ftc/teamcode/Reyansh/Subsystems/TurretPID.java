package org.firstinspires.ftc.teamcode.Reyansh.Subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

public class TurretPID implements Subsystem {

    public static TurretPID INSTANCE = new TurretPID();

    private ControlSystem controller;

    public static MotorGroup outtake = new MotorGroup(
            new MotorEx("outtakeleft").reversed(),
            new MotorEx("outtakeright")
    );

    Pose CachedPose = null;

    public void init(HardwareMap hardwareMap) {
        MotorGroup outtake = new MotorGroup(
                new MotorEx("outtakeleft").reversed(),
                new MotorEx("outtakeright")
        );


    }

    public void Configure() {
        controller = ControlSystem.builder()
                .velPid(0.001, 0.0, 0.0)
                .basicFF(0.003, 0.08, 0.0)
                .build();

        controller.setGoal(new KineticState(0.0, 0.0));
    }

    double yt = 121 - 72;
    double xt = 121 - 72;


    public void FlyWheelsOn() {
        follower.update();
//        telemetry.update();
        Pose cachedPose = PedroComponent.follower().getPose();

        double x = cachedPose.getY() - 72;
        double y = cachedPose.getX() - 72;
        double distance = Math.sqrt(Math.pow(yt-y, 2)  + Math.pow(xt-x, 2));
        double Velocity = (distance * 200) + 200;
        controller.setGoal(new KineticState(0.0, Velocity));
        outtake.setPower(controller.calculate(new KineticState(
                outtake.getCurrentPosition(),
                outtake.getVelocity()))
        );
//        telemetry.addData(String.valueOf(Velocity), "Velocity");
//        telemetry.addData(String.valueOf(x), "x");
//        telemetry.addData(String.valueOf(y), "y");
//        telemetry.addData(String.valueOf(distance), "distance");
//        telemetry.addData(String.valueOf(outtake.getVelocity()), "distance");


//        if (gamepad1.aWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 2000.0));
//        } else if (gamepad1.bWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 0.0));
//        } else if (gamepad1.xWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 1000.0));
//        }

    }


    public void stop() {
        controller.setGoal(new KineticState(0.0, 0.0));
        outtake.setPower(controller.calculate(new KineticState(
                outtake.getCurrentPosition(),
                outtake.getVelocity()))
        );
    }
}