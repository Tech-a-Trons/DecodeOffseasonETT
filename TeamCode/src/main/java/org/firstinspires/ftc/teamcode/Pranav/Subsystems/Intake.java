package org.firstinspires.ftc.teamcode.Pranav.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private DcMotorEx intake;

    public Intake() {


    }

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "in");
    }

    public void into() {
        if (intake != null){
            intake.setPower(0.6);
        }

    }
    public void out() { if (intake != null) intake.setPower(-0.6);

    }
    public void off() { if (intake != null) intake.setPower(0.0);
    }

    @Override
    public void periodic() {
    }
}
