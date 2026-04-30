package org.firstinspires.ftc.teamcode.Pranav.Subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static java.lang.Math.atan2;

import java.util.Objects;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret("null");

    public ControlSystem controller;

    public static ServoGroup turret = new ServoGroup(
            new ServoEx("turretleft"),
            new ServoEx("turretright")
    );

    public void cont() {
        controller = ControlSystem.builder()
                .velPid(0.001, 0.0, 0.0)
                .basicFF(0.003, 0, 0.0)
                .build();
    }
    double goalx = 0;
    double goaly = 0;

    public Turret(String Alliance) {
       turret = new ServoGroup(
               new ServoEx("turretServo1"),
               new ServoEx("turretServo2")
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

    double x;
    double y;
    double heading;
    double angle;
    double fin;

    public void goalface() {
        follower.update();

        x = follower.getPose().getX();
        y = follower.getPose().getY();

        heading = follower.getPose().getHeading() * (180/Math.PI);

        angle = atan2((goaly-y),(goalx-x));

        fin = angle * (180/Math.PI);

        double pos = fin/360;

        controller.setGoal(new KineticState(pos));
        turret.setPosition(controller.calculate(new KineticState(
                turret.getPosition()))
        );
    }
}
