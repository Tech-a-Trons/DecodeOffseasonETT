import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TwoDay extends LinearOpMode {

    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class,"motor1");
        motor2 = hardwareMap.get(DcMotor.class,"motor2");

        motor3 = hardwareMap.get(DcMotor.class,"motor3");
        motor4 = hardwareMap.get(DcMotor.class,"motor4");


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
            motor1.setPower(0.05);
            }

            if (gamepad1.b) {
            motor2.setPower(-0.05);
            }

            if (gamepad1.x) {
            motor3.setPower(0.75);
            }

            if (gamepad1.y) {
            motor4.setPower(-0.75);
            }
        }
    }
}
