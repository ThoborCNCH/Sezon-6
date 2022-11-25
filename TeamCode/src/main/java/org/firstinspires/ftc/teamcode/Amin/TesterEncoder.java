package org.firstinspires.ftc.teamcode.Amin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
@Disabled

public class TesterEncoder extends LinearOpMode {

    SampleMecanumDrive robot;

    @Override
    public void runOpMode(){

        robot = new SampleMecanumDrive(hardwareMap);


        robot.glisiera1.setDirection(DcMotor.Direction.REVERSE);
        robot.glisiera2.setDirection(DcMotor.Direction.REVERSE);

        robot.glisiera1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.glisiera2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.glisiera1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.glisiera2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        robot.glisieraEncoder(60);
        sleep(1000);
        robot.glisieraEncoder(-60);

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