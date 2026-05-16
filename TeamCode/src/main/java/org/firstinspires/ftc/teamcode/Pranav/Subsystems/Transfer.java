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

    public Command into() { motor.setPower(0.6);
        return null;
    }
    public Command out() { motor.setPower(-0.6);
        return null;
    }
    public Command off() { motor.setPower(0);
        return null;
    }
    public Command little() { motor.setPower(0.3);
        return null;
    }

    @Override
    public void periodic() {

    }
}
