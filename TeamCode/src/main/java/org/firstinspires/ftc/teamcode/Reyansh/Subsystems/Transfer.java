package org.firstinspires.ftc.teamcode.Reyansh.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Transfer implements Subsystem {
    public static Transfer INSTANCE = new Transfer();
    // put hardware, commands, etc here

    private MotorEx transfer;

    public void init(HardwareMap hardwareMap) {
        // initialization logic (runs on init)
        transfer = new MotorEx("transfer");
    }

    public void forward() {
        if (transfer != null) transfer.setPower(-1);
    }

    public void backward() {
        if (transfer != null) transfer.setPower(0.1);
    }

    public void slight() {
        if (transfer != null) transfer.setPower(-0.2);
    }
    public void advance() {
        if (transfer != null) transfer.setPower(-0.60);
    }

    public void stop() {
        if (transfer != null) transfer.setPower(0);
    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}