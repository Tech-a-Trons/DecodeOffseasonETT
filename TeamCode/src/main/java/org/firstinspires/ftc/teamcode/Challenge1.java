package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Challenge1 extends LinearOpMode {


    DcMotor motor;
    DcMotor motor2;
    DcMotor motor3;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "fl");
        motor2 = hardwareMap.get(DcMotor.class, "fl");
        motor3 = hardwareMap.get(DcMotor.class, "br");
        waitForStart();
        motor.setPower(0.05);
        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor.setPower(1);
            }
            if (gamepad1.b) {
                motor.setPower(0);
            }
            if (gamepad1.x) {
                motor2.setPower(0);

                if (gamepad1.y) {
                    motor3.setPower(0);
                }

            }

        }
    }
}

