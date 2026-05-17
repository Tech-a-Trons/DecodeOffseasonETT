package org.firstinspires.ftc.teamcode.Pranav.Subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import java.io.PipedOutputStream;
import java.util.Objects;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;

public class Hood implements Subsystem {
    public static final Hood INSTANCE = new Hood();

    public ControlSystem controller;

    public static ServoGroup hood = new ServoGroup(
            new ServoEx("hood")
    );

    public void cont() {
        controller = ControlSystem.builder()
                .velPid(0.001, 0.0, 0.0)
                .basicFF(0.003, 0, 0.0)
                .build();
    }

    double goalx = 0;
    double goaly = 0;

    public void init (String Alliance) {
        hood = new ServoGroup(
                new ServoEx("hood")
        );

        controller = ControlSystem.builder()
                .velPid(0.001, 0.0, 0.0)
                .basicFF(0.003, 0, 0.0)
                .build();

        controller.setGoal(new KineticState(0.1, 0.0));

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
    double distance;
    double pos;

    public void setHood() {
        PedroComponent.follower().update();

        x = PedroComponent.follower().getPose().getX();
        y = PedroComponent.follower().getPose().getY();

        distance = Math.sqrt(Math.pow((goalx-x),2)+Math.pow((goaly-y),2));

        pos = (0.00382793 * distance) - 0.0209811;

        controller.setGoal(new KineticState(pos));
        hood.setPosition(controller.calculate(new KineticState(
                hood.getPosition()))
        );

    }
}
