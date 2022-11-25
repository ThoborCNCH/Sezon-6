package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.DISTANTA_GLISIERA;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.GLISIERA_COUNTS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class ClasaGlisiere {


    SampleMecanumDrive robot;

    public ClasaGlisiere(){
        robot = new SampleMecanumDrive(hardwareMap);

        robot.glisiera1.setDirection(DcMotor.Direction.REVERSE);
        robot.glisiera2.setDirection(DcMotor.Direction.REVERSE);

        robot.glisiera1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.glisiera2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void baga(double distanta)
    {
        robot = new SampleMecanumDrive(hardwareMap);

        robot.glisiera1.setDirection(DcMotor.Direction.REVERSE);
        robot.glisiera2.setDirection(DcMotor.Direction.REVERSE);

        robot.glisiera1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.glisiera2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.glisieraEncoder(distanta);
    }

    public void glisieraEncoder (double distance)
    {

        robot.glisiera1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.glisiera2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.glisiera1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.glisiera2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int counter = (int)(GLISIERA_COUNTS/DISTANTA_GLISIERA);

        int targetLeft = robot.glisiera1.getCurrentPosition() + (int)(distance * counter);
        int targetright = robot.glisiera2.getCurrentPosition() + (int)(distance * counter);

        robot.glisiera1.setTargetPosition(targetLeft);
        robot.glisiera2.setTargetPosition(targetright);

//        telemetry.addData("target", String.valueOf(targetLeft));
//        telemetry.update();

        robot.glisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.glisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setGliseraPower(1);
//
        while(robot.glisiera1.isBusy() || robot.glisiera2.isBusy())
        {
            //da

        }

        robot.glisiera1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.glisiera2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.setGliseraPower(0);

    }

}
