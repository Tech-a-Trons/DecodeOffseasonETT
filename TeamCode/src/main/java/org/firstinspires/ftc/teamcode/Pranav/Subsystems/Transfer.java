package org.firstinspires.ftc.teamcode.Pranav.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer(hardwareMap);
    private MotorEx motor;
    public Transfer(HardwareMap hardwareMap) {
        motor = hardwareMap.get(MotorEx.class, "transfer");
    }

    public void into() { motor.setPower(0.6);}
    public void out() { motor.setPower(-0.6);}
    public void off() { motor.setPower(0);}
    public void little() { motor.setPower(0.3);}

    @Override
    public void periodic() {

    }
}
