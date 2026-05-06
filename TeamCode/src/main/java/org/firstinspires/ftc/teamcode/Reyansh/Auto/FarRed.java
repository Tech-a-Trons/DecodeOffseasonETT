package org.firstinspires.ftc.teamcode.Reyansh.Auto;//package org.firstinspires.ftc.teamcode.Reyansh.Auto;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.TelemetryManager;
//import com.bylazar.telemetry.PanelsTelemetry;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import dev.nextftc.ftc.NextFTCOpMode;
//
//@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
//@Configurable // Panels
//public class FarRed extends NextFTCOpMode {
//    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
//    public Follower follower; // Pedro Pathing follower instance
//    private int pathState; // Current autonomous path state (state machine)
//    private Paths paths; // Paths defined in the Paths class
//
//    ElapsedTime pathTimer = new ElapsedTime();
//
//    @Override
//    public void onInit() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
//
//        paths = new Paths(follower); // Build paths
//
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
//        ElapsedTime pathTimer = new ElapsedTime();
//
//    }
//
//    @Override
//    public void onUpdate() {
//
//        follower.update(); // Update Pedro Pathing
//        pathState = autonomousPathUpdate(); // Update autonomous state machine
//
//        // Log values to Panels and Driver Station
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//        panelsTelemetry.update(telemetry);
//    }
//
//    public static class Paths {
//        public PathChain Path1;
//        public PathChain Path2;
//        public PathChain Path3;
//        public PathChain Path4;
//
//
//        public PathChain Path5;
//
//        public Paths(Follower follower) {
//            Path1 = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(80.000, 6.000),
//                                    new Pose(87.711, 30.245),
//                                    new Pose(107.324, 35.258),
//                                    new Pose(107.249, 35.160)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
//                    .build();
//
//            Path2 = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(107.249, 35.160),
//                                    new Pose(140.391, 35.384)
//                            )
//                    )
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            Path3 = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(140.391, 35.384),
//                                    new Pose(80.000, 6.000)
//                            )
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(0))
//                    .build();
//
//            Path4 = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(80.000, 6.000),
//                                    new Pose(139.842, 8.239)
//                            )
//                    )
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            Path5 = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(139.842, 8.239),
//                                    new Pose(80.000, 6.000)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build();
//        }
//    }
//
//    public int autonomousPathUpdate() {
//        // Add your state machine Here
//        // Access paths with paths.pathName
//        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
//        case 1:
//        switch (pathState += 1) {
//            setPathState(6);
//return();
//        }
//
//        return 0;
//    }
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.reset();
//    }
//}