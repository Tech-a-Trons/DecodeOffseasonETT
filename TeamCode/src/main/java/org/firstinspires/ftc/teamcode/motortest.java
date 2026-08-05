package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class motortest extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();

        if (gamepad1.a) {
            motor.setPower(0.6);
        }

        if (gamepad1.b) {
            motor.setPower(-0.6);
        }

        if (gamepad1.x) {
            motor.setPower(0);
        }
    }
}
