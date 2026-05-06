package org.firstinspires.ftc.teamcode.Reyansh.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intaker implements Subsystem {
    public static final Intaker INSTANCE = new Intaker();
    // put hardware, commands, etc here

    MotorEx intake = new MotorEx("intake");

    public void forward() {
        intake.setPower(1);
    }

    public void backward() {
        intake.setPower(-1);
    }

    public void slight() {
        intake.setPower(0.35);
    }

    public void advance() {
        intake.setPower(0.60);
    }

    public void stop() {
        intake.setPower(0);
    }


    public void init(HardwareMap hardwareMap) {
        // initialization logic (runs on init)
        MotorEx intake = new MotorEx("intake");
    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}