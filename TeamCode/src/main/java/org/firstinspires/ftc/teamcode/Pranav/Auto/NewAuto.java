package org.firstinspires.ftc.teamcode.Pranav.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public class NewAuto extends NextFTCOpMode {
    private Follower follower;

    public NewAuto() {
        addComponents(
                new SubsystemComponent(CompliantIntake.INSTANCE, TurretPID.INSTANCE, Transfer.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }


    public PathChain p1, p2, p3, p4;

    public void buildPaths() {
        p1 = follower.pathBuilder()
        .addPath(
                new BezierLine(
                        new Pose(56.000, 8.000),
                        new Pose(6.025, 32.936)
                )
        )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
        p2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(6.025, 32.936),
                                new Pose(29.522, 39.301),
                                new Pose(47.892, 30.561)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        p3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(47.892, 30.561),
                                new Pose(39.026, 6.518),
                                new Pose(8.035, 8.345)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        p4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.035, 8.345),
                                new Pose(115.842, 114.885)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

    }
}
