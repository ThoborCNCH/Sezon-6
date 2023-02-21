package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.ExecutionException;

public class TestThread extends Thread {

    DcMotor brat, brat_pe_sub;
    ElapsedTime timer;
    SampleMecanumDrive robot;

    public Runnable run = () -> {

        try {
            if (!isInterrupted()) {
                // we record the Y values in the main class to make showing them in telemetry
                // easier.
                brat_pe_sub = hardwareMap.dcMotor.get("mamataESula");


            }
        } catch (Exception e) {

        }

    };


    public TestThread(HardwareMap hardwareMap) {
        robot = new SampleMecanumDrive(hardwareMap);
        timer = new ElapsedTime();
        brat = hardwareMap.dcMotor.get("brat");
        brat_pe_sub = hardwareMap.dcMotor.get("brat_pe_sub");

    }

    public void caca_nu_e_voie() throws InterruptedException {


        this.start();

        timer.reset();
        while (timer.seconds() <= 2 ) {
            se_ridica_brat(0.7);
        }
        se_ridica_brat(0);

        sleep(500);
        timer.reset();

        while (timer.seconds() <= 2) {
            se_ridica_brat(-0.7);
        }

        se_ridica_brat(0);

    }

    private void se_ridica_brat(double putere) {
        brat.setPower(putere);
        brat_pe_sub.setPower(-putere);
    }
}
