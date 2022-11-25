package org.firstinspires.ftc.teamcode.Amin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous@Disabled

public class Test_auto extends LinearOpMode {
    DcMotor rf, lf, rr, lr;
    public double power = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");
        waitForStart();

        while (opModeIsActive()) {
            rf.setPower(-power);
            lf.setPower(-power);
            rr.setPower(power);
            lr.setPower(-power);
            sleep(2000);

            rf.setPower(0);
            lf.setPower(0);
            rr.setPower(0);
            lr.setPower(0);

            stop();
        }
    }
}
