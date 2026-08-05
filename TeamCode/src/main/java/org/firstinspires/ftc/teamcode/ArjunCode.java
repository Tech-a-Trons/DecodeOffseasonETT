package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ArjunCode extends LinearOpMode {
    DcMotor fl,fr,bl,br;
    @Override
    public void runOpMode() throws InterruptedException {

        fl=hardwareMap.get(DcMotor.class,"Fl");
        fl=hardwareMap.get(DcMotor.class,"FR");
        fl=hardwareMap.get(DcMotor.class,"Bl");
        fl=hardwareMap.get(DcMotor.class,"BR");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if(isStopRequested())return;

        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

            double flPower = (y+x+rx)/denominator;
            double blpower = (y-x+rx)/denominator;
            double frpower = (y-x-rx)/denominator;
            double brpower = (y+x-rx)/denominator;

            fl.setPower(flPower);
            bl.setPower(blpower);
            fr.setPower(frpower);
            br.setPower(brpower);
        }
    }
}