package org.firstinspires.ftc.teamcode.Pranav.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Pranav.Subsystems.ShooterPID;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BackBluePranav", group = "Autonomous")
@Configurable // Panels
public class BackBluePranav extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private ShooterPID shooter;
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Timer opmodeTimer;
    public static Pose startPose = new Pose(56.13248407643312, 8.254777070063692, Math.toDegrees(180)); // Start Pose of our robot.

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

//        shooter.init();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    private static Path shootPreload;
    public static PathChain intSide;
    public static PathChain gtsPre;
    public static PathChain int3rd;
    public static PathChain gts3rd;
    public static PathChain intG1;
    public static PathChain gtsG1;
    public static PathChain intG2;
    public static PathChain gtsG2;
    public static PathChain intG3;
    public static PathChain gtsG3;

    public static class Paths {

        public Paths(Follower follower) {
            shootPreload = new Path(new BezierLine(startPose,startPose));
            shootPreload.setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading());

            intSide = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.132, 8.255),
                                    new Pose(8.489, 8.668)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gtsPre = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.489, 8.668),
                                    new Pose(39.804, 8.458),
                                    new Pose(63.396, 14.539)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            int3rd = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.396, 14.539),
                                    new Pose(52.422, 36.700),
                                    new Pose(18.150, 35.546)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gts3rd = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.150, 35.546),
                                    new Pose(44.763, 22.001),
                                    new Pose(63.470, 14.492)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intG1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.470, 14.492),
                                    new Pose(39.830, 8.761),
                                    new Pose(8.521, 8.735)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gtsG1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.521, 8.735),
                                    new Pose(39.746, 8.618),
                                    new Pose(63.339, 14.389)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intG2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.339, 14.389),
                                    new Pose(39.831, 8.717),
                                    new Pose(8.546, 8.697)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gtsG2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.546, 8.697),
                                    new Pose(39.857, 8.627),
                                    new Pose(63.391, 14.391)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intG3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.391, 14.391),
                                    new Pose(39.802, 8.564),
                                    new Pose(8.437, 8.809)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gtsG3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.437, 8.809),
                                    new Pose(39.957, 8.611),
                                    new Pose(63.608, 14.231)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
//                follower.followPath(shootPreload);
//                shooter.farshoot();

                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(intSide,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gtsPre,true);
//                    shooter.farshoot();
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(int3rd,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gts3rd,true);
//                    shooter.farshoot();
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(intG1,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gtsG1, true);
//                    shooter.farshoot();
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an7 new paths */
                    follower.followPath(intG2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gtsG2, true);
//                    shooter.farshoot();
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an7 new paths */
                    follower.followPath(intG3, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gtsG3, true);
//                    shooter.farshoot();
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
        return 0;
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}