package org.firstinspires.ftc.teamcode.Reyansh.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intaker implements Subsystem {
    public static final Intaker INSTANCE = new Intaker();
    // put hardware, commands, etc here

    private MotorEx intake;

    public void init(HardwareMap hardwareMap) {
        // initialization logic (runs on init)
        intake = new MotorEx("in");
    }

    public void forward() {
        if (intake != null) intake.setPower(1);
    }

    public void backward() {
        if (intake != null) intake.setPower(-1);
    }

    public void slight() {
        if (intake != null) intake.setPower(0.35);
    }

    public void advance() {
        if (intake != null) intake.setPower(0.60);
    }

    public void stop() {
        if (intake != null) intake.setPower(0);
    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}