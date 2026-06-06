package org.firstinspires.ftc.teamcode.Pranav.Subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Objects;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class ShooterPID implements Subsystem {

    public static final ShooterPID INSTANCE = new ShooterPID();
    public ControlSystem controller;

    public static MotorGroup shoot = new MotorGroup(
            new MotorEx("outtakeright"),
            new MotorEx("outtakeleft").reversed()
    );

//    public void cont() {
//        controller = ControlSystem.builder()
//                .velPid(0.00025, 0.0, 0.0)
//                .basicFF(0.0001, 0, 0.0)
//                .build();
//    }

    double goalx = 0;
    double goaly = 0;

    public void init(String Alliance) {
        MotorGroup shoot = new MotorGroup(
                new MotorEx("outtakeright"),
                new MotorEx("outtakeleft").reversed()
        );

        controller = ControlSystem.builder()
                .velPid(0.0055, 0.0, 0.5)
                .basicFF(0.0001, 0.0, 0.0)
                .build();

        controller.setGoal(new KineticState(0.0, 0.0));

        if (Objects.equals(Alliance, "blue")) {
            goalx = 0;
            goaly = 144;
        } else if (Objects.equals(Alliance, "red")) {
            goalx = 144;
            goaly = 144;
        } else if (Objects.equals(Alliance, "null")) {
            goalx = 0;
            goaly = 0;
        }
    }

//    public void run() {
//        flywheelMotor.setVelocity(controller.calculate(new KineticState(
//                flywheelMotor.getCurrentPosition(),
//                flywheelMotor.getVelocity())));
//    }

    public double velo;

    public void shoot() {
        PedroComponent.follower().update();
        Pose pose = PedroComponent.follower().getPose();
        double cx = pose.getX() - 72;
        double cy = pose.getY() - 72;
        double distance = Math.sqrt(Math.pow(goalx - cx, 2) + Math.pow(goaly - cy, 2));

        // Remove "double" here — assign to the FIELD, not a new local variable
        if (distance >= 123) {
        velo = (4.5 * distance);
        } else {
            velo = (4.5 * distance)-3000;
        }

        controller.setGoal(new KineticState(0.0, velo));
        shoot.setPower(controller.calculate(new KineticState(
                shoot.getCurrentPosition(),
                shoot.getVelocity())));
    }


//       double cvelo = (8.605087243 * distance) + 150;
//
////       controller.setGoal(new KineticState(0.0, cvelo));
////
////       shoot.setPower(controller.calculate(new KineticState(
////                shoot.getCurrentPosition(),
////                shoot.getVelocity()))
////        );
//



//    public Command far() {
//
//        follower.update();
//
//        Pose pose = PedroComponent.follower().getPose();
//
//        double cx = pose.getX();
//        double cy = pose.getY();
//
//        double distance = Math.sqrt(Math.pow(goalx-cx,2)+Math.pow(goaly-cy,2));
//
//        double fvelo = (8.605087243 * distance) + 200;
//
////        controller.setGoal(new KineticState(0.0, fvelo));
////
////        shoot.setPower(controller.calculate(new KineticState(
////                shoot.getCurrentPosition(),
////                shoot.getVelocity()))
////        );
//
//        return new RunToVelocity(controller,fvelo).requires(this);
//    }

public void repel() {
    controller.setGoal(new KineticState(0.0, -1000));
    shoot.setPower(controller.calculate(new KineticState(
            shoot.getCurrentPosition(),
            shoot.getVelocity())));
}

    public void stop() {
        velo = 0;
        controller.setGoal(new KineticState(0.0, 0.0));
        shoot.setPower(0);
    }
}