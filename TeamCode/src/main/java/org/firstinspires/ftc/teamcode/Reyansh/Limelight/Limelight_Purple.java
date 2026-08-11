package org.firstinspires.ftc.teamcode.Reyansh.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.ftc.NextFTCOpMode;

public class Limelight_Purple extends NextFTCOpMode {
private Limelight3A limelight3A;
    @Override
    public void onInit() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight3A");
        limelight3A.pipelineSwitch(1);
    }

    @Override
    public void onStartButtonPressed(){
        limelight3A.start();

    }

    @Override
    public void onUpdate(){
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null & llResult.isValid()){
            telemetry.addData("Target X offset", llResult.getTx());
            telemetry.addData("Target Y offset", llResult.getTy());
            telemetry.addData("Target Area offset", llResult.getTa());
            telemetry.addData("Target Latency", llResult.getTargetingLatency());

        }
    }
}