/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Amin.autoiasi;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name = "dr red && dr blue")
public class v3 extends LinearOpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    SampleMecanumDrive robot;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    static final double FEET_PER_METER = 3.28084;


    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    private  DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public  DcMotor glisiera1;
    public  DcMotor glisiera2;
    public  CRServo marus;

    public static double fata = 25;
    public static double unghi1 = 40;
    public static double unghi2 = -40;
    public static double strafe = 22;
    public static double splinex = -50;
    public static double spliney = 60;


    public static double poquito = 12;

    public static double timp = 1;

    public static double INITIALX = 12;
    public static double initialy = 64;
    public static double initialunghi = 180;



    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");

        glisiera1 = hardwareMap.get(DcMotor .class, "glisiera1");
        glisiera2 = hardwareMap.get(DcMotor.class, "glisiera2");

        marus = hardwareMap.get(CRServo .class, "marus");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        robot = new SampleMecanumDrive(hardwareMap);


        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }



        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            stanga();
        }else if(tagOfInterest.id == MIDDLE){
            mijloc();
        }else{
            dreapta();
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
    }

    private void mijloc() throws InterruptedException {

        Trajectory marian = robot.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(initialunghi)))
                .forward(fata)
                .build();
        Trajectory front = robot.trajectoryBuilder(marian.end().plus(new Pose2d(0,0,Math.toRadians(45))))
                .forward(poquito)
                .build();
        Trajectory back = robot.trajectoryBuilder(front.end())
                .back(poquito)
                .build();
        Trajectory stanga = robot.trajectoryBuilder(marian.end())
                .strafeLeft(2)
                .build();sleep(1000);
        robot.turn(Math.toRadians(180));
        robot.followTrajectory(marian);
        robot.turn(Math.toRadians(unghi1));
        robot.glisierautonom(1);
        robot.intakeautonom(-1);
        robot.followTrajectory(front);
        sleep(700);
        robot.intakeautonom(1);
        sleep(200);
        robot.followTrajectory(back);
        sleep(200);
        robot.glisierautonom(-1);
        robot.turn(Math.toRadians(unghi2));
//
    }
    private void stanga() throws InterruptedException {

        Trajectory marian = robot.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(initialunghi)))
                .forward(fata)
                .build();
        Trajectory front = robot.trajectoryBuilder(marian.end().plus(new Pose2d(0,0,Math.toRadians(45))))
                .forward(poquito)
                .build();
        Trajectory back = robot.trajectoryBuilder(front.end())
                .back(poquito)
                .build();
        Trajectory stanga = robot.trajectoryBuilder(marian.end())
                .strafeLeft(strafe)
                .build();sleep(1000);
        robot.turn(Math.toRadians(180));
        robot.followTrajectory(marian);
        robot.turn(Math.toRadians(unghi1));
        robot.glisierautonom(1);
        robot.intakeautonom(-1);
        robot.followTrajectory(front);
        sleep(700);
        robot.intakeautonom(1);
        sleep(200);
        robot.followTrajectory(back);
        sleep(200);
        robot.glisierautonom(-1);
        robot.turn(Math.toRadians(unghi2));
        robot.followTrajectory(stanga);

    }

    private void dreapta() throws InterruptedException {
        Trajectory marian = robot.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(initialunghi)))
                .forward(fata)
                .build();
        Trajectory front = robot.trajectoryBuilder(marian.end().plus(new Pose2d(0,0,Math.toRadians(45))))
                .forward(poquito)
                .build();
        Trajectory back = robot.trajectoryBuilder(front.end())
                .back(poquito)
                .build();
        Trajectory stanga = robot.trajectoryBuilder(marian.end())
                .strafeRight(strafe)
                .build();
        sleep(1000);
        robot.turn(Math.toRadians(180));
        robot.followTrajectory(marian);
        robot.glisierautonom(1);
        robot.intakeautonom(-1);
        robot.followTrajectory(front);
        sleep(700);
        robot.intakeautonom(1);
        sleep(200);
        robot.followTrajectory(back);
        sleep(200);
        robot.glisierautonom(-1);
        robot.turn(Math.toRadians(unghi2));
        robot.followTrajectory(stanga);
    }
}