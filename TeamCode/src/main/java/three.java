import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class three extends LinearOpMode {

    DcMotor motor1;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor1.setPower(0.9);
            }
        }

    }
}
