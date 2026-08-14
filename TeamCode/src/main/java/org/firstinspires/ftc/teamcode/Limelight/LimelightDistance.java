package org.firstinspires.ftc.teamcode.Limelight;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class LimelightDistance {
    private static final double LIMELIGHT_HEIGHT_INCHES = 13.25;  // lens height off floor
    private static final double LIMELIGHT_ANGLE_DEGREES = 25.0;   // degrees mount is tilted AWAY from straight-down (0 = pointing straight at floor)
    private static final double BALL_HEIGHT_INCHES = 2.4;         // ball center height off floor (~radius)

    private Limelight3A limelight3A;
    private com.qualcomm.robotcore.util.ElapsedTime timer = new com.qualcomm.robotcore.util.ElapsedTime();
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    private LLResult lastResult;

    public LimelightDistance() {}

    public LimelightDistance(HardwareMap hardwareMap, String name) {
        limelight3A = hardwareMap.get(Limelight3A.class, name);
        limelight3A.pipelineSwitch(1);
    }

    public void setTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void startReading() {
        limelight3A.start();
    }

    public void stopReading() {
        limelight3A.stop();
    }

    public void update() {
        lastResult = limelight3A.getLatestResult();
    }

    public boolean isTargetVisible() {
        return lastResult != null && lastResult.isValid();
    }

    public Double getTx() {
        return isTargetVisible() ? lastResult.getTx() : null;
    }

    // Returns null when no target is visible, instead of a placeholder 0,
    // so callers can distinguish "no reading this cycle" from "target is
    // exactly at the camera's zero-distance point." object_following relies
    // on this null check to fall back to LOCKED_FALLBACK_POWER instead of
    // hard-stopping every time a target flickers out for one loop tick.
    public Double getDistance() {
        if (!isTargetVisible()) return null;
        return getDistance(lastResult.getTy());
    }

    public double getDistance(double ty) {
        // Total angle measured from straight-down (nadir), not from horizontal
        double angleFromVertical = LIMELIGHT_ANGLE_DEGREES + ty;
        double heightDifference = LIMELIGHT_HEIGHT_INCHES - BALL_HEIGHT_INCHES;

        // Ground distance from the point directly below the camera to the target
        return heightDifference * Math.tan(Math.toRadians(angleFromVertical));
    }
}