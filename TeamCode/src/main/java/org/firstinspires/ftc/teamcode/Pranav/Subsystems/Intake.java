package org.firstinspires.ftc.teamcode.Pranav.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake(hardwareMap);
    private MotorEx intake;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(MotorEx.class, "in");
    }

    public void into() { intake.setPower(0.6);}
    public void out() { intake.setPower(-0.6);}
    public void off() { intake.setPower(0.0);}

    @Override
    public void periodic() {

    }
}
