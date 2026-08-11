package org.firstinspires.ftc.teamcode.Reyansh.Auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Reyansh.Auto.Auto_1.AutonomousProgram.follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Pranav.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.Intaker;
import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.util.Timer;

@Autonomous(name = "NextFTC Autonomous Program Java")
public class Auto_1 {
    private static Timer pathTimer;
    private Timer opmodeTimer;
    private static int pathState;

    public class AutonomousProgram extends NextFTCOpMode {
        public AutonomousProgram() {
            addComponents(
                    new SubsystemComponent(
                            Intaker.INSTANCE,
                            Turret.INSTANCE,
                            Transfer.INSTANCE

                    ),
                    BulkReadComponent.INSTANCE
            );
        }

        public static Follower follower;

        public class Paths {
            public PathChain Shooting;
            public PathChain PickUpPart1;
            public PathChain PickUpPart2;

            public void buildPaths() {
                Shooting = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(111.067, 136.439),
                                        new Pose(72.000, 72.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                        .build();

                PickUpPart1 = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(72.000, 72.000),
                                        new Pose(104.800, 83.364)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build();

                PickUpPart2 = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(104.800, 83.364),
                                        new Pose(133.336, 83.172)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build();

                Shooting = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(133.336, 83.172),
                                        new Pose(72.000, 72.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build();
            }

            public void autonomousPathUpdate() {
                switch (pathState) {
                    case 0:
                        follower.followPath(Shooting);
                        setPathState(1);
                        break;
                    case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                        if (!follower.isBusy()) {
                            /* Score Preload */
                            /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                            follower.followPath(PickUpPart1, true);
                            setPathState(2);
                        }
                        break;
                    case 2:
                        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                        if (!follower.isBusy()) {
                            /* Grab Sample */
                            /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                            follower.followPath(PickUpPart2, true);
                            setPathState(3);
                        }
                        break;
                    case 3:
                        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                        if (!follower.isBusy()) {
                            /* Score Sample */
                            /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                            follower.followPath(Shooting, true);
                            setPathState(4);
                        }
                        break;
                }
            }

            /**
             * These change the states of the paths and actions. It will also reset the timers of the individual switches
             **/
            public void setPathState(int pState) {
                pathState = pState;
                pathTimer.resetTimer();
            }


            public void onUpdate() {
                pathTimer = new Timer();
                opmodeTimer = new Timer();
                opmodeTimer.resetTimer();
                follower = Constants.createFollower(hardwareMap);
                buildPaths();
                waitForStart();
                //on start
                opmodeTimer.resetTimer();
                setPathState(0);
                    follower.update();
                    autonomousPathUpdate();

                    // Feedback to Driver Hub for debugging
                    telemetry.addData("path state", pathState);
                    telemetry.addData("x", follower.getPose().getX());
                    telemetry.addData("y", follower.getPose().getY());
                    telemetry.addData("heading", follower.getPose().getHeading());
                    telemetry.update();
                }
            }
        }
    }

