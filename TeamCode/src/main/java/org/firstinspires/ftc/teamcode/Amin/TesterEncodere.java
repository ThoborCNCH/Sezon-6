package org.firstinspires.ftc.teamcode.Amin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
@Disabled
public class TesterEncodere extends LinearOpMode {

    public DcMotor brat, brat_pe_sub;


    @Override
    public void runOpMode() throws InterruptedException {

        brat = hardwareMap.dcMotor.get("brat");
        brat_pe_sub = hardwareMap.dcMotor.get("brat_pe_sub");

        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
//        while (opModeIsActive())
//        {
//            telemetry.addData("sus", String.valueOf(brat.getCurrentPosition()));
//            telemetry.addData("jos", String.valueOf(brat_pe_sub.getCurrentPosition()));
//            telemetry.update();
//        }

        int susTarget = brat.getTargetPosition() + 1000;
        int josTarget = brat_pe_sub.getTargetPosition() + 1000;

        brat.setTargetPosition(susTarget);
        brat_pe_sub.setTargetPosition(josTarget);

        brat.setPower(0.4);
        brat_pe_sub.setPower(0.4);

        while (opModeIsActive() && brat.isBusy() && brat_pe_sub.isBusy())
        {
            telemetry.addData("brat", String.valueOf(brat.getCurrentPosition()));
            telemetry.addData("brat_pe_sub", String.valueOf(brat_pe_sub.getCurrentPosition()));
            telemetry.update();
        }

        brat.setPower(0);
        brat_pe_sub.setPower(0);


//        brat.setPower(0);
//        brat.setPower(0);


    }
}
