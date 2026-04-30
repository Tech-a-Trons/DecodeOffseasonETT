package org.firstinspires.ftc.teamcode.Pranav.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FrontRedPranav", group = "Autonomous")
@Configurable // Panels
public class FrontRedPranav extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Timer opmodeTimer;
    public static Pose startPose = new Pose(122.77197452229298, 123.75031847133758, Math.toDegrees(37)); // Start Pose of our robot.

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

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

    public static PathChain gotoshoot1;
    public static PathChain intake2nd;
    public static PathChain gotoshoot2;
    public static PathChain gate1;
    public static PathChain gotoshoot3;
    public static PathChain gate2;
    public static PathChain gotoshoot4;
    public static PathChain gate3;
    public static PathChain gotoshoot5;

    public static class Paths {

        public Paths(Follower follower) {
            gotoshoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(122.772, 123.750),
                                    new Pose(74.544, 84.284)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                    .build();

            intake2nd = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(74.544, 84.284),
                                    new Pose(98.016, 56.801),
                                    new Pose(124.589, 59.401)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            gotoshoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(124.589, 59.401),
                                    new Pose(97.912, 56.759),
                                    new Pose(74.466, 84.146)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            gate1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(74.466, 84.146),
                                    new Pose(102.741, 62.785),
                                    new Pose(131.567, 61.950)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();

            gotoshoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.567, 61.950),
                                    new Pose(102.805, 62.839),
                                    new Pose(74.629, 83.925)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            gate2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(74.629, 83.925),
                                    new Pose(102.932, 62.769),
                                    new Pose(131.619, 61.939)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();

            gotoshoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.619, 61.939),
                                    new Pose(102.332, 62.776),
                                    new Pose(74.567, 84.088)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            gate3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(74.567, 84.088),
                                    new Pose(102.354, 62.484),
                                    new Pose(131.992, 62.307)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();

            gotoshoot5 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.992, 62.307),
                                    new Pose(102.578, 62.806),
                                    new Pose(74.394, 84.088)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(gotoshoot1,true);
                    setPathState(1);
                }
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTime() > 0.5) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(intake2nd,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gotoshoot2,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTime() > 0.5) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(gate1,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gotoshoot3,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTime() > 0.5) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(gate2,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gotoshoot4, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTime() > 0.5) {
                    /* Set the state to a Case we won't use or define, so it just stops running an7 new paths */
                    follower.followPath(gate3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gotoshoot5, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(pathTimer.getElapsedTime() > 0.5) {
                    /* Set the state to a Case we won't use or define, so it just stops running any new paths */
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