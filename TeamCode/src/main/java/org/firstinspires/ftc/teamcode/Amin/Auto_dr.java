package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.INTRE_TOT_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.JUNCTION_PUNE_INAINTE_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.JUNCTION_PUNE_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.JUNCTION_PUNE_DR_RED_BLUE_VECTOR;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.JUNCTION_PUNE_INAINTE_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.JUNCTION_PUNE_ST_RED_BLUE_VECTOR;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.PRE_POSITION_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.PRE_POSITION_DR_RED_BLUE2;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.PRE_POSITION_DR_RED_BLUE3;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.STACK_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.START_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_brat_dc;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_deschis_dr_AUTO;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_deschis_st_AUTO;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_inchis_dr;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_inchis_st;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.sleep_for_strafe;

import android.net.wifi.aware.ParcelablePeerHandle;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Auto_dr extends LinearOpMode {
    AprilTagDetection tagOfInterest;
    double cx = 402.145;
    double cy = 221.506;
    double fx = 578.272;
    double fy = 578.272;
    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;


    // UNITS ARE METERS
    double tagsize = 0.166;
    SampleMecanumDrive robot;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);
        robot.setPoseEstimate(START_DR_RED_BLUE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
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

        waitForStart();
        while (opModeIsActive()) {
            robot.top.setPower(0);
//            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
////            tagOfInterest.id = 2;
//
//            // If there's been a new frame...
//            if (detections != null) {
//                telemetry.addData("FPS", camera.getFps());
//                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
//                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
//
//                // If we don't see any tags
//                if (detections.size() == 0) {
//                    numFramesWithoutDetection++;
//
//                    // If we haven't seen a tag for a few frames, lower the decimation
//                    // so we can hopefully pick one up if we're e.g. far back
//                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
//                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
//                    }
//                }
//                // We do see tags!
//                else {
//                    numFramesWithoutDetection = 0;
//
//                    // If the target is within 1 meter, turn on high decimation to
//                    // increase the frame rate
//                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
//                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
//                    }
//                    for (AprilTagDetection tag : detections) {
//                        if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
//                            telemetry.addLine(String.valueOf(tag));
//                            telemetry.update();
//                            tagOfInterest = tag;
//                        }
//                    }
//
//                    switch (tagOfInterest.id) {
//                        case 1:
//                            stanga();
//                            break;
//                        case 3:
//                            dreapta();
//                            break;
//                        case 2:
//                            mijloc();
//                            break;
//                        default:
//                            mijloc();
//                            break;
//                    }

//                    stop();
//                }
            stanga();
            stop();
//            }
        }
    }

    private void stanga() {
        robot.apuca(poz_inchis_st, poz_inchis_dr);
        sleep(200);
        robot.se_ridica_brat(power_brat_dc);
//            sleep(10000);

        TrajectorySequence go_pune = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(PRE_POSITION_DR_RED_BLUE
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
//                .addTemporalMarker(time -> time * 0, () -> {
//                    robot.se_ridica_brat(power_brat_dc);
//                })
//                .forward(17)
//                .back(13)
                .build();
        robot.followTrajectorySequence(go_pune);
        robot.se_ridica_brat(power_brat_dc);

//        sleep(200);
        ;
//        robot.apuca(robot.gheara_stanga.getPosition(), robot.gheara_dreapta.getPosition());

//        Trajectory efectiv = robot.trajectoryBuilder(go_pune.end())
//                .splineToConstantHeading(JUNCTION_PUNE_DR_RED_BLUE_VECTOR, Math.toRadians(90))
//                .addTemporalMarker(time -> time * 0, () -> {
//                    robot.se_ridica_brat(power_brat_dc);
//                })

//                .build();
//        robot.followTrajectory(efectiv);

//        timer.reset();
//        while (timer.seconds() <= 4.5 && robot.getDistanceSensorJos() >= 15) {
//            robot.bagaViteza(-0.3, 0.3, 0.3, -0.3);
//        }
//        sleep(sleep_for_strafe);
//
//        robot.update();

        //        robot.setPoseEstimate(JUNCTION_PUNE_INAINTE_DR_RED_BLUE);
        Trajectory fata = robot.trajectoryBuilder(go_pune.end())
//                .lineToLinearHeading(
//                        new Pose2d(robot.getPoseEstimate().getX(),
//                                robot.getPoseEstimate().getY() + 2.6,
//                                Math.toRadians(90)))
                .forward(9, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(18))
//                .lineToConstantHeading(JUNCTION_PUNE_DR_RED_BLUE_VECTOR)
                .build();
        robot.followTrajectory(fata);

//        --------------------------------------------
//        IN CAZ DE SENZOR IMPLEMENTARE
//        timer.reset();
//        while(timer.seconds() <= 2 && robot.ia_distanta()>=30)
//        {
//            robot.bagaViteza(0.2, -0.2, 0.2, -0.2);
//        }
//        ------------------------------

        robot.se_ridica_brat(power_brat_dc);

//        sleep(1000);

//        robot.apuca(poz_deschis_st_AUTO, poz_deschis_dr_AUTO);


        TrajectorySequence reven = robot.trajectorySequenceBuilder(fata.end())
                .addTemporalMarker(0, () -> {
                    robot.apuca(poz_deschis_st_AUTO, poz_deschis_dr_AUTO);
                })
                .waitSeconds(0.3)
                .back(5)
//                .lineToLinearHeading(STACK_DR_RED_BLUE)
//                .strafeLeft(13)
                .build();
        robot.se_ridica_brat(0);

        robot.followTrajectorySequence(reven);


        while (robot.getDistanceSensorSus() >= 23) {
            robot.bagaViteza(-0.22, -0.22, 0.42, -0.22);
            if(robot.getDistanceSensorSus() >= 40){
                robot.se_ridica_brat(-0.6);
            }else{
                robot.se_ridica_brat(-0.27);
            }
        }
        while(robot.getDistanceSensorSus() <= 20) {
            robot.se_ridica_brat(0.27);
        }
        robot.se_ridica_brat(0.05);
        robot.bagaViteza(0, 0, 0, 0);
        robot.update();

        Trajectory rr = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(STACK_DR_RED_BLUE)
                .build();
        robot.followTrajectory(rr);

        timer.reset();
        while (robot.getDistanceSensorJos() >= 12 && timer.seconds() >= 2) {
//            robot.se_ridica_brat(-0.6);
            robot.bagaViteza(0.2, 0.2, 0.2, 0.2);
        }
        robot.bagaViteza(0, 0, 0, 0);


//        while (robot.getDistanceSensorJos() >= 8) {
//            robot.bagaViteza(0.2, 0.2, 0.2, 0.2);
//        }
//        robot.bagaViteza(0, 0, 0, 0);
//        robot.se_ridica_brat(0);

        sleep(100);

//        if (robot.getDistanceSensorJos() <= 11) {
            robot.black();
//        }
        sleep(300);

        robot.se_ridica_brat(1);
        robot.update();

        Trajectory back = robot.trajectoryBuilder(robot.getPoseEstimate())
//                .back(26)
                .lineToLinearHeading(PRE_POSITION_DR_RED_BLUE3)

                .build();
        robot.followTrajectory(back);
        robot.se_ridica_brat(1);
//
//        Trajectory pune = robot.trajectoryBuilder(back.end())
//                .build();
//        robot.followTrajectory(pune);
//        robot.se_ridica_brat(1);

        Trajectory fata2 = robot.trajectoryBuilder(back.end())
                .forward(10, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(18))
                .build();
        robot.followTrajectory(fata2);
        robot.se_ridica_brat(1);


        TrajectorySequence reven2 = robot.trajectorySequenceBuilder(fata2.end())
                .addTemporalMarker(0, () -> {
                    robot.apuca(poz_deschis_st_AUTO, poz_deschis_dr_AUTO);
                })
                .waitSeconds(0.2)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -10, Math.toRadians(90)), Math.toRadians(12))
                .build();
        robot.followTrajectorySequence(reven2);

        robot.se_ridica_brat(0);

    }

    private void dreapta() {

        robot.apuca(poz_inchis_st, poz_inchis_dr);
        sleep(100);
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

        sleep(100);

        Trajectory efectiv = robot.trajectoryBuilder(go_pune.end())
                .splineToConstantHeading(JUNCTION_PUNE_DR_RED_BLUE_VECTOR, Math.toRadians(90))
                .addTemporalMarker(time -> time * 0, () -> {
                    robot.se_ridica_brat(power_brat_dc);
                })
                .build();

        robot.followTrajectory(efectiv);
//        sleep(1000);
        robot.se_ridica_brat(0);

        sleep(200);
        robot.apuca(poz_deschis_st_AUTO, poz_deschis_dr_AUTO);

        TrajectorySequence reven = robot.trajectorySequenceBuilder(efectiv.end())
                .back(3)
                .strafeRight(34)

//                .splineToLinearHeading(INTRE_TOT_DR_RED_BLUE, Math.toRadians(90),
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                )
////                    .advtoqrhit lRhg 9qru 3 0IW3dTemporalMarker(time -> time * 0.8, () -> {
//                        robot.se_ridica_brat(-power_brat_dc);
//                    })
                .build();
        robot.se_ridica_brat(0);

        robot.followTrajectorySequence(reven);

//        Trajectory align_sa_ia = robot.trajectoryBuilder(reven.end())
//                .strafeLeft(25)
//                .build();
//        robot.followTrajectory(align_sa_ia);
    }

    private void mijloc() {

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

        TrajectorySequence reven = robot.trajectorySequenceBuilder(efectiv.end())
                .back(3)
                .strafeRight(14)
                .build();
        robot.se_ridica_brat(0);

        robot.followTrajectorySequence(reven);
    }

    private void pune_con(Pose2d pozitie) {
        TrajectorySequence reven = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .back(3)
                .strafeRight(14)
                .turn(90)
                .build();

        robot.followTrajectorySequence(reven);
//        return robot.
    }
}