package org.firstinspires.ftc.teamcode.Pranav.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class NewAuto extends NextFTCOpMode {
    private Follower follower;

    public NewAuto() {
        addComponents(
                new SubsystemComponent(CompliantIntake.INSTANCE, TurretPID.INSTANCE, Transfer.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    private final Pose startPose = new Pose(56.000, 8.00, Math.toDegrees(180)); // Start Pose of our robot. This is against the goal facing AWAY

    public PathChain SidePick, Strafe1, BackShoot1, RowSetup, RowPick, BackShoot2, GatePick1, Strafe2, BackShoot3, GatePick2, Strafe3, BackShoot4;

    private Timer pathTimer, opmodeTimer;
    private int pathState;

    public void buildPaths() {
        SidePick = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),
                                new Pose(7.697, 8.580)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Strafe1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(7.697, 8.580),
                                new Pose(7.916, 23.783)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        BackShoot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(7.916, 23.783),
                                new Pose(69.185, 21.246)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        RowSetup = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(69.185, 21.246),
                                new Pose(64.257, 38.085),
                                new Pose(44.985, 36.765)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        RowPick = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(44.985, 36.765),
                                new Pose(37.603, 34.761),
                                new Pose(10.417, 36.025)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        BackShoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.417, 36.025),
                                new Pose(69.424, 18.721)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        GatePick1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(69.424, 18.721),
                                new Pose(9.348, 19.627)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Strafe2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.348, 19.627),
                                new Pose(9.014, 30.079)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        BackShoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.014, 30.079),
                                new Pose(69.734, 17.637)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        GatePick2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(69.734, 17.637),
                                new Pose(9.083, 19.618)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Strafe3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.083, 19.618),
                                new Pose(8.683, 29.845)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        BackShoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.683, 29.845),
                                new Pose(70.934, 18.281)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }

    public void AutoPaths() {
        switch (pathState) {
            case 0:
                CompliantIntake.INSTANCE.on();
                follower.followPath(SidePick, true);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    Transfer.INSTANCE.nice();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Strafe1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    TurretPID.INSTANCE.setShooterSpeed(1000);
                    /* Grab Sample */
                    CompliantIntake.INSTANCE.off();
                    Transfer.INSTANCE.off();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(BackShoot1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    /* Grab Sample */


                    CompliantIntake.INSTANCE.on();
                    Transfer.INSTANCE.on();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(4);
                }
                break;

            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(RowSetup,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    CompliantIntake.INSTANCE.on();
                    Transfer.INSTANCE.nice();
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(RowPick,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    TurretPID.INSTANCE.setShooterSpeed(1000);
                    /* Score Sample */
                    CompliantIntake.INSTANCE.off();
                    Transfer.INSTANCE.off();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(BackShoot2,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    CompliantIntake.INSTANCE.on();
                    Transfer.INSTANCE.on();


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    CompliantIntake.INSTANCE.on();
                    Transfer.INSTANCE.nice();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(GatePick1,true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Strafe2,true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    TurretPID.INSTANCE.setShooterSpeed(1000);

                    /* Score Sample */
                    CompliantIntake.INSTANCE.off();
                    Transfer.INSTANCE.off();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(BackShoot3,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    CompliantIntake.INSTANCE.on();
                    Transfer.INSTANCE.on();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(12);
                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    CompliantIntake.INSTANCE.on();
                    Transfer.INSTANCE.nice();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(GatePick2,true);
                    setPathState(13);
                }
                break;
            case 13:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Strafe3,true);
                    setPathState(14);
                }
                break;
            case 14:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    TurretPID.INSTANCE.setShooterSpeed(1000);

                    /* Score Sample */
                    CompliantIntake.INSTANCE.off();
                    Transfer.INSTANCE.off();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(BackShoot4,true);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    CompliantIntake.INSTANCE.on();
                    Transfer.INSTANCE.on();
                    TurretPID.INSTANCE.setShooterSpeed(1000);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(16);
                }
                break;
            case 16:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running any new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void onStartButtonPressed(){
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void onUpdate() {
        follower.update();
        AutoPaths();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }
}
