package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.JUNCTION_THING_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.JUNCTION_THING_DR_RED_BLUE2;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.PRE_POSITION_DR_RED_BLUE_KKK;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.STACK_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.STACK_DR_RED_BLUE2;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.START_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.TIMER_SENZOR_DR;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_brat_dc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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

@Autonomous()
public class Nurofen_DR extends LinearOpMode {
    SampleMecanumDrive robot;
    public VoltageSensor batteryVoltageSensor;
    private DcMotor brat, brat_pe_sub;

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

    ElapsedTime timer;

    /*
     *
     *          _.-/`)                              .-.
     *         // / / )                           __| |__
     *      .=// / / / )                         [__   __]
     *     //`/ / / / /      __________             | |
     *    // /     ` /      /          \            | |
     *   ||         /       | WE PRAY  |            | |
     *    \\       /        | TO WORK  |            '-'
     *     ))    .'         \_________/
     *    //    /
     *         /
     *
     */

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();

        brat = hardwareMap.dcMotor.get("brat");
        brat_pe_sub = hardwareMap.dcMotor.get("brat_pe_sub");

        robot = new SampleMecanumDrive(hardwareMap);
        robot.setPoseEstimate(START_DR_RED_BLUE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        brat.setDirection(DcMotorSimple.Direction.REVERSE);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat_pe_sub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat_pe_sub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        tagOfInterest = new AprilTagDetection();

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode) {
                telemetry.addData("eroare: ", String.valueOf(errorCode));
                telemetry.update();
                tagOfInterest.id = 3;
            }
        });

        //detectie inainte de start
//        tagOfInterest.id = 3;

        //detectie
//        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
//        if (detections != null) {
//            telemetry.addData("FPS", camera.getFps());
//            telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
//            telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
//
//            // If we don't see any tags
//            if (detections.size() == 0) {
//                numFramesWithoutDetection++;
//
//                // If we haven't seen a tag for a few frames, lower the decimation
//                // so we can hopefully pick one up if we're e.g. far back
//                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
//                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
//                }
//            }
//            // We do see tags!
//            else {
//                numFramesWithoutDetection = 0;
//
//                // If the target is within 1 meter, turn on high decimation to
//                // increase the frame rate
//                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
//                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
//                }
//                for (AprilTagDetection tag : detections) {
//                    if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
//                        telemetry.addLine(String.valueOf(tag.id));
//                        telemetry.update();
//                        tagOfInterest = tag;
//                    }
//                }
//            }
//        }

        robot.strange();

        waitForStart();
        while (opModeIsActive() && opModeIsActive()) {

            //detectie dupa start

//            tagOfInterest = new AprilTagDetection();
//            tagOfInterest.id = 3;
//
//            //detectie
//            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
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
//                            telemetry.addLine(String.valueOf(tag.id));
//                            telemetry.update();
//                            tagOfInterest = tag;
//                        }
//                    }
//                }
//            }

            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }
                    for (AprilTagDetection tag : detections) {
                        if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                            telemetry.addLine(String.valueOf(tag.id));
                            telemetry.update();
                            tagOfInterest = tag;
                        }
                    }
                }
            }
            sleep(500);

            camera.closeCameraDevice();

            telemetry.addData("baterie: ", String.valueOf(batteryVoltageSensor.getVoltage()));
            telemetry.addData("id: ", String.valueOf(tagOfInterest.id));
            telemetry.update();

            //apuca con 1
//            sleep(400);
            robot.strange();

            //ridica brat 1
            sleep(300);
            se_ridica_brat(power_brat_dc);

            //traj junction 1
            TrajectorySequence go_pune = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(41.5, -40, Math.toRadians(0)))
                    .lineToLinearHeading(PRE_POSITION_DR_RED_BLUE_KKK,
                            SampleMecanumDrive.getVelocityConstraint(40,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .addTemporalMarker(time -> time * 0.4, () -> {
                        robot.strange();
                        robot.rotesteThing(1);
                    })
                    .build();
            robot.followTrajectorySequence(go_pune);
            //tine brat ridicat 1
            se_ridica_brat(power_brat_dc);

            //traj revenire 1 && lasa con 1
            TrajectorySequence reven = robot.trajectorySequenceBuilder(go_pune.end())
                    .waitSeconds(0.2)
                    .addTemporalMarker(0, () -> {
                        robot.deschide_gheara();
                    })
                    .waitSeconds(0.2)
                    .strafeRight(2)
                    .build();

            //opreste ridicare
            se_ridica_brat(0);
            robot.followTrajectorySequence(reven);

            timer.reset();
            //thing revenire 1
            while (opModeIsActive() && !robot.getMagnetAtingere()) {
                robot.rotesteThing(-1);
                if (timer.seconds() >= TIMER_SENZOR_DR)
                    break;
            }
            robot.rotesteThing(0);

            robot.bagaViteza(0, 0, 0, 0);

            //coboara brat 1
//            brat.setTargetPosition(-cob1);
//            brat_pe_sub.setTargetPosition(-cob1);

            double ticks = 320;

            brat.setTargetPosition(brat.getCurrentPosition() - 1455);
            brat_pe_sub.setTargetPosition(brat_pe_sub.getCurrentPosition() - 1455);
            brat.setPower(-0.6);
            brat_pe_sub.setPower(0.6);
//
            while (opModeIsActive() && opModeIsActive() && brat.isBusy() && brat_pe_sub.isBusy()) {
                telemetry.addData("brat", String.valueOf(brat.getCurrentPosition()));
                telemetry.addData("brat_pe_sub", String.valueOf(brat_pe_sub.getCurrentPosition()));
                telemetry.update();
            }

            brat.setPower(0);
            brat_pe_sub.setPower(0);

            brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brat_pe_sub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //traj to stack 1
            Trajectory rr = robot.trajectoryBuilder(reven.end())
                    .splineToLinearHeading(STACK_DR_RED_BLUE, Math.toRadians(0),
                            SampleMecanumDrive.getVelocityConstraint(
                                    35,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(
                                    35
                            )
                    )
                    .build();
            robot.followTrajectory(rr);

            sleep(100);

            //apuca con 2
            robot.strange();
            sleep(400);

            //ridica brat 2
            se_ridica_brat(power_brat_dc);

            //updateaza pozitia robot
            robot.update();

            //traj la junction con 2
            Trajectory back = robot.trajectoryBuilder(robot.getPoseEstimate())
                    .addTemporalMarker(time -> time * 0.4, () -> {
                        robot.rotesteThing(0.5);
                    })
//                    .lineToConstantHeading(new Vector2d(40, -5))
                    .lineToLinearHeading(JUNCTION_THING_DR_RED_BLUE,
                            SampleMecanumDrive.getVelocityConstraint(40,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .build();
            robot.followTrajectory(back);

            //tine brat ridicat  2
            se_ridica_brat(power_brat_dc);

            //opreste rotire thing
            robot.rotesteThing(0);

            //lasa con 2
            sleep(200);
            robot.deschide_gheara();
            sleep(200);

            timer.reset();
            //revenire thing pt alt con
            while ((opModeIsActive() && !robot.getMagnetAtingere())) {
                robot.rotesteThing(-1);
                if (timer.seconds() >= TIMER_SENZOR_DR)
                    break;
            }
            //opreste rotire thing
            robot.rotesteThing(0);

            //coboara brat 3
//            brat.setTargetPosition(cob2);
//            brat_pe_sub.setTargetPosition(cob2);

            brat.setTargetPosition(brat.getCurrentPosition() - 1682);
            brat_pe_sub.setTargetPosition(brat_pe_sub.getCurrentPosition() - 1682);

            brat.setPower(-0.6);
            brat_pe_sub.setPower(0.6);
//
            while (opModeIsActive() && opModeIsActive() && brat.isBusy() && brat_pe_sub.isBusy()) {
                telemetry.addData("brat", String.valueOf(brat.getCurrentPosition()));
                telemetry.addData("brat_pe_sub", String.valueOf(brat_pe_sub.getCurrentPosition()));
                telemetry.update();
            }

            brat.setPower(0);
            brat_pe_sub.setPower(0);

            brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brat_pe_sub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//speram sa fie bine sa nu fie rau
            brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//stim ce facem aici nu e o problema
            robot.bagaViteza(0, 0, 0, 0);//respectele mele ce pot sa zic aici decat speram sa mearga

            //traj la stack 2
            Trajectory rr2 = robot.trajectoryBuilder(robot.getPoseEstimate())
                    .splineToLinearHeading(STACK_DR_RED_BLUE2, Math.toRadians(0),
                            SampleMecanumDrive.getVelocityConstraint(
                                    35,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(
                                    30
                            ))
                    .build();
            robot.followTrajectory(rr2);

            //apuca con 3
            sleep(100);
            robot.strange();
            sleep(300);

            //ridica brat 3
            se_ridica_brat(power_brat_dc);
//            sleep(400);

            //miscare sasiu senzor dist 1
            robot.update();

            //traj la junction con 3
            Trajectory back2 = robot.trajectoryBuilder(robot.getPoseEstimate())
                    .addTemporalMarker(time -> time * 0.4, () -> {
                        robot.rotesteThing(0.5);
                    })
//                    .lineToConstantHeading(new Vector2d(39.2, -5))
                    .lineToLinearHeading(JUNCTION_THING_DR_RED_BLUE2,
                            SampleMecanumDrive.getVelocityConstraint(38,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30)) //era 35
                    .build();
            robot.followTrajectory(back2);

            //tine brat ridicat 3
            se_ridica_brat(power_brat_dc);

            //opreste rotire thing
            robot.rotesteThing(0);

            //lasa con 3
            sleep(200);
            robot.deschide_gheara();
            sleep(200);

            //parcare
            switch (tagOfInterest.id) {
                case 1:
                    stanga();
                    break;
                case 3:
                    dreapta();
                    break;
                case 2:
                    mijloc();
                    break;
                default:
                    dreapta();
                    break;
            }
            stop();
        }
    }

    private void stanga() {
        TrajectorySequence park = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .strafeRight(4)
                .back(16)
                .build();
        robot.followTrajectorySequence(park);
        timer.reset();
        while (opModeIsActive() && !robot.getMagnetAtingere())
            robot.rotesteThing(-1);
        robot.rotesteThing(0);
    }

    private void mijloc() {
        Trajectory park = robot.trajectoryBuilder(robot.getPoseEstimate())
                .forward(5)
                .build();
        robot.followTrajectory(park);
        timer.reset();
        while (opModeIsActive() && !robot.getMagnetAtingere())
            robot.rotesteThing(-1);
        robot.rotesteThing(0);
    }

    private void dreapta() {

        //revenire thing pt con 4
        timer.reset();
        while (opModeIsActive() && !robot.getMagnetAtingere()) {
            robot.rotesteThing(-1);
        }
        robot.rotesteThing(0);


        //coboara brat
        brat.setTargetPosition(brat.getCurrentPosition() - 1747);
        brat_pe_sub.setTargetPosition(brat_pe_sub.getCurrentPosition() - 1747);

        brat.setPower(-0.6);
        brat_pe_sub.setPower(0.6);
//
        while (opModeIsActive() && opModeIsActive() && brat.isBusy() && brat_pe_sub.isBusy()) {
            telemetry.addData("brat", String.valueOf(brat.getCurrentPosition()));
            telemetry.addData("brat_pe_sub", String.valueOf(brat_pe_sub.getCurrentPosition()));
            telemetry.update();
        }

        brat.setPower(0);
        brat_pe_sub.setPower(0);

        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat_pe_sub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.bagaViteza(0, 0, 0, 0);


        //mergi al stack
        TrajectorySequence rr2 = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(68.7, -9.8, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        robot.followTrajectorySequence(rr2);

        sleep(100);

        robot.strange();

        sleep(500);
        se_ridica_brat(0.9);
        sleep(120);
        robot.rotesteThing(-1);
        sleep(350);
        se_ridica_brat(0.01);
        sleep(320);

//        robot.rotesteThing(0);

        se_ridica_brat(0.01);

        //mergi la junction low
        Trajectory park = robot.trajectoryBuilder(robot.getPoseEstimate())
                .strafeTo(new Vector2d(robot.getPoseEstimate().getX() - 6.1, robot.getPoseEstimate().getY() - 2.5))
                .addDisplacementMarker(() -> {
                    robot.deschide_gheara();
                })
                .build();
        robot.followTrajectory(park);

        robot.rotesteThing(0.7);

        //parcheaza
        Trajectory ok = robot.trajectoryBuilder(park.end())
                .strafeTo(new Vector2d(robot.getPoseEstimate().getX() + 4, robot.getPoseEstimate().getY() + 4))
                .build();
        robot.followTrajectory(ok);

        //        sleep(50);
//        osama.rotesteThing(1);
//        while (  opModeIsActive() && !osama.getMagnetAtingere())
//            osama.rotesteThing(-1);
//        osama.rotesteThing(0);
    }

    private void se_ridica_brat(double putere) {
        brat.setPower(putere);
        brat_pe_sub.setPower(-putere);
    }
}
