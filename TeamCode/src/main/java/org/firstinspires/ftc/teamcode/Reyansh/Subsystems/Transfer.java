package org.firstinspires.ftc.teamcode.Reyansh.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Transfer implements Subsystem {
    public static Transfer INSTANCE = new Transfer();
    // put hardware, commands, etc here

    MotorEx transfer = new MotorEx("transfer");

    public void forward() {
        transfer.setPower(-1);
    }

    public void backward() {
        transfer.setPower(1);
    }

    public void slight() {
        transfer.setPower(-0.35);
    }
    public void advance() {
        transfer.setPower(-0.60);
    }

    public void stop() {
        transfer.setPower(0);
    }


    public void init(HardwareMap hardwareMap) {
        // initialization logic (runs on init)
        MotorEx transfer = new MotorEx("transfer");
    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}