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
public class FrontBluePranav extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Timer opmodeTimer;
    public static Pose startPose = new Pose(20.963057324840758, 123.38343949044587, Math.toDegrees(143)); // Start Pose of our robot.

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

    public static PathChain shootPre;
    public static PathChain int2nd;
    public static PathChain gts1;
    public static PathChain intG1;
    public static PathChain gts2;
    public static PathChain intG2;
    public static PathChain gts3;
    public static PathChain intG3;
    public static PathChain gts4;

    public static class Paths {

        public Paths(Follower follower) {
            shootPre = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(20.963, 123.383),
                                            new Pose(64.071, 87.913)
                                    )
                            )
                            .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                            .build();

                    int2nd = follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Pose(64.071, 87.913),
                                            new Pose(49.738, 55.984),
                                            new Pose(18.326, 59.790)
                                    )
                            )
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build();

                    gts1 = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(18.326, 59.790),
                                            new Pose(64.270, 88.330)
                                    )
                            )
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build();

                    intG1 = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(64.270, 88.330),
                                            new Pose(11.190, 60.902)
                                    )
                            )
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                            .build();

                    gts2 = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(11.190, 60.902),
                                            new Pose(64.186, 88.373)
                                    )
                            )
                            .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                            .build();

                    intG2 = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(64.186, 88.373),
                                            new Pose(11.098, 60.794)
                                    )
                            )
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                            .build();

                    gts3 = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(11.098, 60.794),
                                            new Pose(64.448, 88.003)
                                    )
                            )
                            .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                            .build();

                    intG3 = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(64.448, 88.003),
                                            new Pose(11.594, 60.912)
                                    )
                            )
                            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                            .build();

                    gts4 = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Pose(11.594, 60.912),
                                            new Pose(64.461, 88.135)
                                    )
                            )
                            .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                            .build();
                }
            }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(shootPre,true);
                    setPathState(1);
                }
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTime() > 0.5) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(int2nd,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gts1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTime() > 0.5) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(intG1,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gts2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTime() > 0.5) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(intG2,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gts3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTime() > 0.5) {
                    /* Set the state to a Case we won't use or define, so it just stops running an7 new paths */
                    follower.followPath(intG3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gts4, true);
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