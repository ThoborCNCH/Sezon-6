package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class bratHandler implements Runnable {

    DcMotor brat, brat_pe_sub;
    ElapsedTime timer;
    Telemetry telemetry;

    public bratHandler(DcMotor brat, DcMotor brat_pe_sub, Telemetry telemetry) {
        this.brat = brat;
        this.brat_pe_sub = brat_pe_sub;
//        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        brat_pe_sub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.telemetry = telemetry;
    }

    @Override
    public void run() {
        timer = new ElapsedTime();
        timer.reset();

        telemetry.addLine("1");
        telemetry.update();

        while (timer.seconds() <= 3) {
            se_ridica_brat(0.5);
            telemetry.addData("2: ", brat.getPower());
            telemetry.update();
        }

        se_ridica_brat(0);
        telemetry.addLine("3");
        telemetry.update();

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        timer.reset();
        while (timer.seconds() <= 3) {
            telemetry.addData("4: ", brat.getPower());
            se_ridica_brat(-0.5);
            telemetry.update();
        }

        se_ridica_brat(0);
        telemetry.addLine("5");
        telemetry.update();
    }

    public void se_ridica_brat(double power) {
        brat.setPower(power);
        brat_pe_sub.setPower(-power);
    }
}
