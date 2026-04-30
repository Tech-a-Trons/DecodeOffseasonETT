package org.firstinspires.ftc.teamcode.Pranav.Subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;

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

    public static final ShooterPID INSTANCE = new ShooterPID("null");
    public ControlSystem controller;

    public static MotorGroup shoot = new MotorGroup(
            new MotorEx("outtakeright"),
            new MotorEx("outtakeleft").reversed()
    );

    public void cont() {
        controller = ControlSystem.builder()
                .velPid(0.001, 0.0, 0.0)
                .basicFF(0.003, 0, 0.0)
                .build();
    }

    double goalx = 0;
    double goaly = 0;

    public ShooterPID(String Alliance) {
        MotorGroup shoot = new MotorGroup(
                new MotorEx("outtakeright"),
                new MotorEx("outtakeleft").reversed()
        );

        controller = ControlSystem.builder()
                .velPid(0.001, 0.0, 0.0)
                .basicFF(0.003, 0, 0.0)
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

    double velo;

    public Command shoot() {

       follower.update();

       Pose pose = PedroComponent.follower().getPose();

       double cx = pose.getX();
       double cy = pose.getY();

       double distance = Math.sqrt(Math.pow(goalx-cx,2)+Math.pow(goaly-cy,2));

       if (distance >= 123) {
           double velo = (8.605087243 * distance) + 150;
       } else if (distance <= 123) {
           double velo = (8.605087243 * distance) + 200;
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

       return new RunToVelocity(controller,velo).requires(this);
    }

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

    public Command repel() {
        return new RunToVelocity(controller,-1000).requires(this);
    }

    public Command stop() {

        return new RunToVelocity(controller,0).requires(this);

    }
}