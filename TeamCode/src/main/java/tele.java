import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class tele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {

            telemetry.update();

            telemetry.addData("Status: ","Running");
            telemetry.addData("Time: ", getRuntime());
        }
    }
}
