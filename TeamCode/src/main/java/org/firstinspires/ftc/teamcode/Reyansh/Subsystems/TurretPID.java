package org.firstinspires.ftc.teamcode.Reyansh.Subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    private MotorGroup outtake;

    Pose CachedPose = null;

    public void init(HardwareMap hardwareMap) {
        outtake = new MotorGroup(
                new MotorEx((DcMotorEx) hardwareMap.get("outtakeleft")).reversed(),
                new MotorEx((DcMotorEx) hardwareMap.get("outtakeright"))
        );

        controller = ControlSystem.builder()
                .velPid(0.0016, 0.0, 0.004)
                .basicFF(0.0001, 0.0, 0.0)
                .build();

        controller.setGoal(new KineticState(0.0, 0.0));
    }

    double yt = 121 - 72;
    double xt = 121 - 72;
    double x = 0;
    double y = 0;




    public void FlyWheelsOn() {
        PedroComponent.follower().update();
        Pose cachedPose = PedroComponent.follower().getPose();

        double x = cachedPose.getY() - 72;
        double y = cachedPose.getX() - 72;
        double distance = Math.sqrt(Math.pow(yt - y, 2) + Math.pow(xt - x, 2));
        double Velocity = (distance * 6);
        controller.setGoal(new KineticState(0.0, Velocity));
    }


    public void stop() {
        if (controller == null) return;
        controller.setGoal(new KineticState(0.0, 0.0));
    }

    @Override
    public void periodic() {
        if (controller == null || outtake == null) return;
        outtake.setPower(controller.calculate(new KineticState(
                outtake.getCurrentPosition(),
                outtake.getVelocity()))
        );
    }
//
//    public double  getX()                 { return x; }
//    public double  getY()                 { return y; }
//    public double  getHeading()           { return heading; }
}