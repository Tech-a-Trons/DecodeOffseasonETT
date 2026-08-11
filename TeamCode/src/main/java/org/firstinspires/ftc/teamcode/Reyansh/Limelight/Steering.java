package org.firstinspires.ftc.teamcode.Reyansh.Limelight;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.Intaker;
import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.TurretPID;
import org.firstinspires.ftc.teamcode.Reyansh.Subsystems.TurretTrack;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class Steering extends NextFTCOpMode {

    public Steering() {

        // Where you add code components that will be used in your opmode, and by putting things here it is reusable
        addComponents(
                // This is where all of the subsystems are called
                new SubsystemComponent(
//                        ColorSensor.INSTANCE,
                        Intaker.INSTANCE

                ),
                // Allows hardware calls to be done in one instant
                BulkReadComponent.INSTANCE,
                // The Next FTC binding system to attach commands to buttons
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    double tx = 0;
    private Limelight3A limelight3A;
    boolean tracking = false;
    boolean valid = false;

    @Override
    public void onInit() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight3A");
        limelight3A.pipelineSwitch(0);
    }
    @Override
    public void onStartButtonPressed() {
        limelight3A.start();

        if (gamepad1.a) {
            tracking = true;

        }
        if (gamepad1.b) {
            tracking = false;
            follower.setTeleOpDrive(
                    0,
                     0,
                    0,
                    true
            );        }
    }
        @Override
        public void onUpdate() {
            LLResult llResult = limelight3A.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                telemetry.addData("Target X offest", llResult.getTx());
                telemetry.addData("Target Y offset", llResult.getTy());
                telemetry.addData("Target Area offset", llResult.getTa());


                tx = llResult.getTx();
            }

            valid = (llResult != null && llResult.isValid());
            if (tracking && !valid) {
                follower.setTeleOpDrive(
                        0,
                        0,
                        3,
                        true
                );
            } else if (tracking && valid && !(tx < 0.1 && -0.1 < tx)) {
                follower.setTeleOpDrive(
                        0,
                        0,
                        3 * tx,
                        true
                );

            } else if (tracking && valid && (tx < 0.1 && -0.1 < tx)) {
                follower.setTeleOpDrive(
                        5,
                        0,
                        0,
                        true
                );
                Intaker.INSTANCE.forward();

            } else {
                follower.setTeleOpDrive(
                        0,
                        0,
                        0,
                        true
                );
                Intaker.INSTANCE.stop();

            }

        }
    }