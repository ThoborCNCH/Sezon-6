package org.firstinspires.ftc.teamcode.Amin;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TesterGlisiereEncoder extends LinearOpMode {

    SampleMecanumDrive robot;

    public void runOpMode(){

        robot = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        while(opModeIsActive())
        {

            int leftCount = robot.glisiera1.getCurrentPosition();
            int rightCount = robot.glisiera2.getCurrentPosition();

            telemetry.addData("Stanga", String.valueOf(leftCount));
            telemetry.addData("Dreapta", String.valueOf(rightCount));
            telemetry.update();

        }

    }

}
