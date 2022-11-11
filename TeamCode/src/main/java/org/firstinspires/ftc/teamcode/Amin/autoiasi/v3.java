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

        robot.setPoseEstimate(new Pose2d(-36, -68, Math.toRadians(-90)));

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

        waitForStart();
        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        telemetry.addLine(String.valueOf(tag));
                        telemetry.update();
                        tagOfInterest = tag;
                    }
                }
            }
            sleep(1000);
            switch (tagOfInterest.id) {
                case 1:
                    stanga();
                    break;
                case 2:
                    mijloc();
                    break;
                case 3:
                    dreapta();
                    break;
                default:
                    mijloc();
                    break;
            }


            telemetry.update();
            stop();
        }
    }

    private void mijloc() throws InterruptedException {
        Trajectory pozitionare = robot.trajectoryBuilder(new Pose2d(-36, -68, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-36, -38, Math.toRadians(140)))
                .addTemporalMarker(0, () -> {
                    robot.setGliseraPower(1);
                })
                .addTemporalMarker(1.5, () -> {
                    robot.setGliseraPower(0);
                })
                .build();
        robot.followTrajectory(pozitionare);

        Trajectory pune_ceva_macar_te_rog = robot.trajectoryBuilder(pozitionare.end())
                .forward(7.22)
//                .addDisplacementMarker(()->{
//                    robot.setIntake(-1);
//                })
                .build();
        robot.followTrajectory(pune_ceva_macar_te_rog);

        robot.setIntake(1);
        sleep(2000);
        robot.setIntake(0);

        Trajectory oleaka_inapoi = robot.trajectoryBuilder(pune_ceva_macar_te_rog.end())
                .lineToLinearHeading(new Pose2d(-36, -40, Math.toRadians(180)))
                .addTemporalMarker(1, () -> {
                    robot.setGliseraPower(-1);
                })
                .addTemporalMarker(5.5, () -> {
                    robot.setGliseraPower(0);
                })
//                .back(20)
                .build();
        robot.followTrajectory(oleaka_inapoi);



    }

    private void stanga() throws InterruptedException {
        Trajectory pozitionare = robot.trajectoryBuilder(new Pose2d(-36, -68, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-36, -38, Math.toRadians(140)))
                .addTemporalMarker(0, () -> {
                    robot.setGliseraPower(1);
                })
                .addTemporalMarker(1.5, () -> {
                    robot.setGliseraPower(0);
                })
                .build();
        robot.followTrajectory(pozitionare);

        Trajectory pune_ceva_macar_te_rog = robot.trajectoryBuilder(pozitionare.end())
                .forward(7.22)
//                .addDisplacementMarker(()->{
//                    robot.setIntake(-1);
//                })
                .build();
        robot.followTrajectory(pune_ceva_macar_te_rog);

        robot.setIntake(1);
        sleep(2000);
        robot.setIntake(0);

        Trajectory oleaka_inapoi = robot.trajectoryBuilder(pune_ceva_macar_te_rog.end())
                .lineToLinearHeading(new Pose2d(-36, -40, Math.toRadians(180)))
                .addTemporalMarker(1, () -> {
                    robot.setGliseraPower(-1);
                })
                .addTemporalMarker(5.5, () -> {
                    robot.setGliseraPower(0);
                })
//                .back(20)
                .build();
        robot.followTrajectory(oleaka_inapoi);

        Trajectory hai_du_te_la_parcare = robot.trajectoryBuilder(oleaka_inapoi.end())
                .forward(23)
                .build();
        robot.followTrajectory(hai_du_te_la_parcare);

    }

    private void dreapta() throws InterruptedException {
        Trajectory pozitionare = robot.trajectoryBuilder(new Pose2d(-36, -68, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-36, -38, Math.toRadians(140)))
                .addTemporalMarker(0, () -> {
                    robot.setGliseraPower(1);
                })
                .addTemporalMarker(1.5, () -> {
                    robot.setGliseraPower(0);
                })
                .build();
        robot.followTrajectory(pozitionare);

        Trajectory pune_ceva_macar_te_rog = robot.trajectoryBuilder(pozitionare.end())
                .forward(7.22)
//                .addDisplacementMarker(()->{
//                    robot.setIntake(-1);
//                })
                .build();
        robot.followTrajectory(pune_ceva_macar_te_rog);

        robot.setIntake(1);
        sleep(2000);
        robot.setIntake(0);

        Trajectory oleaka_inapoi = robot.trajectoryBuilder(pune_ceva_macar_te_rog.end())
                .lineToLinearHeading(new Pose2d(-36, -40, Math.toRadians(180)))
                .addTemporalMarker(1, () -> {
                    robot.setGliseraPower(-1);
                })
                .addTemporalMarker(5.5, () -> {
                    robot.setGliseraPower(0);
                })
//                .back(20)
                .build();
        robot.followTrajectory(oleaka_inapoi);

        Trajectory hai_du_te_la_parcare = robot.trajectoryBuilder(oleaka_inapoi.end())
                .back(23)
                .build();
        robot.followTrajectory(hai_du_te_la_parcare);

    }

}