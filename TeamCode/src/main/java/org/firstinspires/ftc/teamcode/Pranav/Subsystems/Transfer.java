package org.firstinspires.ftc.teamcode.Pranav.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private DcMotorEx motor;
    public Transfer() {
    }

    public void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "transfer");
    }

    public void into()  { if (motor != null) motor.setPower(-0.6); }
    public void out()   { if (motor != null) motor.setPower(0.6); }
    public void off()   { if (motor != null) motor.setPower(0.0); }
    public void little(){ if (motor != null) motor.setPower(-0.3); }

    @Override
    public void periodic() {

    }
}
