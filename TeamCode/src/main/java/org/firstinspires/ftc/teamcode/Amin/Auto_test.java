package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous()
@Disabled
public class Auto_test extends LinearOpMode {

    double cx = 402.145;
    double cy = 221.506;
    double fx = 578.272;
    double fy = 578.272;

    // UNITS ARE METERS
    double tagsize = 0.166;
    AprilTagDetection tagOfInterest;


    @Override
    public void runOpMode() throws InterruptedException {
//        GlisiereHandler glisiera = new GlisiereHandler();
        ElapsedTime timer = new ElapsedTime();
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        robot.setPoseEstimate(START_DR_RED_BLUE);
        telemetry.addData("baterie:", String.valueOf(robot.batteryVoltageSensor.getVoltage()));
        telemetry.update();

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//
//            ;
//        });
//        tagOfInterest.id = 2;
//        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//        if (currentDetections.size() != 0) {
//            for (AprilTagDetection tag : currentDetections) {
//                if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
//                    telemetry.addLine(String.valueOf(tag));
//                    telemetry.update();
//                    tagOfInterest = tag;
//                }
//            }
//        }
//
//        waitForStart();
//        while (opModeIsActive()) {
//
//           sleep(1000);
//            switch (tagOfInterest.id) {
//                case 1:
//                    stanga();
//                    break;
//                case 3:
//                    dreapta();
//                    break;
//                default:
//                    mijloc();
//                    break;
//            }
//
////            telemetry.addData("baterie:", String.valueOf(robot.batteryVoltageSensor.getVoltage()));
////            telemetry.update();
//            stop();
//
//        }
        waitForStart();
        while (opModeIsActive()) {
            robot.apuca(poz_inchis_st, poz_inchis_dr);
            sleep(200);
            robot.se_ridica_brat(power_brat_dc);
//            sleep(10000);
            Trajectory go_pune = robot.trajectoryBuilder(robot.getPoseEstimate())
                    .lineToLinearHeading(PRE_POSITION_DR_RED_BLUE,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addTemporalMarker(time -> time * 0, () -> {
                        robot.se_ridica_brat(power_brat_dc);
                    })
                    .build();
            robot.followTrajectory(go_pune);

            sleep(200);

            Trajectory efectiv = robot.trajectoryBuilder(go_pune.end())
                    .splineToConstantHeading(JUNCTION_PUNE_DR_RED_BLUE_VECTOR, Math.toRadians(90))
                    .addTemporalMarker(time -> time * 0, () -> {
                        robot.se_ridica_brat(power_brat_dc);
                    })
                    .build();

            robot.followTrajectory(efectiv);
            sleep(1000);
            robot.se_ridica_brat(0);

            sleep(200);
            robot.apuca(poz_deschis_st_AUTO, poz_deschis_dr_AUTO);

            Trajectory reven = robot.trajectoryBuilder(efectiv.end(), true)
                    .splineToLinearHeading(INTRE_TOT_DR_RED_BLUE, Math.toRadians(0))
//                    .addTemporalMarker(time -> time * 0.8, () -> {
//                        robot.se_ridica_brat(-power_brat_dc);
//                    })
                    .build();
//        robot.se_ridica_brat(0);

            robot.followTrajectory(reven);

            double i = 0;
            while (i <= 10) {
                robot.se_ridica_brat(-power_brat_dc_cob);
                sleep(100);
                robot.se_ridica_brat(0);
                sleep(100);
                i++;
            }

            TrajectorySequence align_sa_ia = robot.trajectorySequenceBuilder(reven.end())
                    .turn(Math.toRadians(-90))
                    .splineToLinearHeading(STACK_DR_RED_BLUE, Math.toRadians(0),
                            SampleMecanumDrive.getVelocityConstraint(30, 30, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            robot.followTrajectorySequence(align_sa_ia);
            robot.apuca(poz_inchis_st, poz_inchis_dr);

            robot.se_ridica_brat(power_brat_dc);
            sleep(2500);

            TrajectorySequence backToJunc = robot.trajectorySequenceBuilder(align_sa_ia.start())
                    .back(30)
                    .turn(45)
                    .forward(15)
                    .build();
//            robot.followTrajectorySequence(backToJunc);

            robot.apuca(poz_deschis_st_AUTO, poz_deschis_dr_AUTO);

            Trajectory park = robot.trajectoryBuilder(backToJunc.end())
                    .splineToLinearHeading(INTRE_TOT_DR_RED_BLUE, Math.toRadians(0))
                    .build();

            robot.followTrajectory(park);

//            TrajectorySequence restu = robot.trajectorySequenceBuilder(reven.end())
////                    .lineToLinearHeading(JUNCTION_PUNE_DR_RED_BLUE)
//                    .turn(Math.toRadians(-90))
//                    .splineToLinearHeading(STACK_DR_RED_BLUE, Math.toRadians(0))
////                    .strafeLeft(3)
////                    .lineToLinearHeading(INTRE_TOT_DR_RED_BLUE)
////                    .lineToLinearHeading(JUNCTION_PUNE_DR_RED_BLUE)
//                    .back(30)
//                    .turn(45)
//                    .forward(15)
//                    .build();
//            robot.followTrajectorySequence(restu);
            Pose2d pose = robot.getPoseEstimate();
            telemetry.addData("", pose.getX());
            telemetry.addData("", pose.getY());
            telemetry.update();
        }
    }

    private void stanga() throws InterruptedException {

    }

    private void mijloc() throws InterruptedException {
//        robot.apuca(poz_inchis_st, poz_inchis_dr);
//        sleep(200);
//        robot.se_ridica_brat(power_brat_dc);
//        sleep(10000);
//        Trajectory go_pune = robot.trajectoryBuilder(robot.getPoseEstimate())
//                .lineToLinearHeading(PRE_POSITION_DR_RED_BLUE,
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(time -> time * 0, () -> {
//                    robot.se_ridica_brat(power_brat_dc);
//                })
//                .build();
////        robot.followTrajectory(go_pune);
//
//        sleep(200);
//
//        Trajectory efectiv = robot.trajectoryBuilder(go_pune.end())
//                .splineToConstantHeading(JUNCTION_PUNE_DR_RED_BLUE_VECTOR, Math.toRadians(90))
//                .addTemporalMarker(time -> time * 0, () -> {
//                    robot.se_ridica_brat(power_brat_dc);
//                })
//                .build();
//
////        robot.followTrajectory(efectiv);
//        sleep(2000);
//        robot.se_ridica_brat(0);
//
//        sleep(200);
//        robot.apuca(poz_deschis_st_AUTO, poz_deschis_dr_AUTO);
//
//
//        Trajectory reven = robot.trajectoryBuilder(efectiv.end(), true)
//                .splineToLinearHeading(INTRE_TOT_DR_RED_BLUE, Math.toRadians(0))
//
//                .addTemporalMarker(time -> time * 0.8, () -> {
//                    robot.se_ridica_brat(-power_brat_dc);
//                })
//                .build();
////        robot.se_ridica_brat(0);
//
////        robot.followTrajectory(reven);
//
//        TrajectorySequence restu = robot.trajectorySequenceBuilder(reven.end())
////                    .lineToLinearHeading(JUNCTION_PUNE_DR_RED_BLUE)
//                .turn(Math.toRadians(-90))
//                .splineToLinearHeading(STACK_DR_RED_BLUE, Math.toRadians(0))
//                .strafeLeft(3)
//                .lineToLinearHeading(INTRE_TOT_DR_RED_BLUE)
//                .lineToLinearHeading(JUNCTION_PUNE_DR_RED_BLUE)
//                .build();
////        robot.followTrajectorySequence(restu);
//        Pose2d pose = robot.getPoseEstimate();
//        telemetry.addData("", pose.getX());
//        telemetry.addData("", pose.getY());
//        telemetry.update();

    }


    private void dreapta() throws InterruptedException {

    }
}