package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.JUNCTION_THING_ST_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.JUNCTION_THING_ST_RED_BLUE2;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.PRE_POSITION_ST_RED_BLUE_KKK;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.STACK_ST_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.STACK_ST_RED_BLUE2;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.START_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.START_ST_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.TIMER_SENZOR_ST;
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
public class Nurofen_ST extends LinearOpMode {
    SampleMecanumDrive osama;
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

        osama = new SampleMecanumDrive(hardwareMap);
        osama.setPoseEstimate(START_ST_RED_BLUE);
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
            }
        });

        osama.black();

        waitForStart();
        while (opModeIsActive() && opModeIsActive()) {
            tagOfInterest = new AprilTagDetection();
            tagOfInterest.id = 3;

            //DETECTIE
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
                            telemetry.addLine(String.valueOf(tag));
                            telemetry.update();
                            tagOfInterest = tag;
                        }
                    }
                }
            }

            camera.closeCameraDevice();

            telemetry.addData("baterie: ", String.valueOf(batteryVoltageSensor.getVoltage()));
            telemetry.addData("id: ", String.valueOf(tagOfInterest.id));
            telemetry.update();

            //APUCA CON 1
//            sleep(400);
            osama.black();

            //RIDICA BRAT 1
            sleep(300);
            se_ridica_brat(power_brat_dc);

            //TRAJ JUNCTION 1
            TrajectorySequence go_pune = osama.trajectorySequenceBuilder(osama.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-41.5, -40, Math.toRadians(180)))
                    .lineToLinearHeading(PRE_POSITION_ST_RED_BLUE_KKK,
                            SampleMecanumDrive.getVelocityConstraint(40,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .addTemporalMarker(time -> time * 0.4, () -> {
                        osama.black();
                        osama.rotesteThing(-1);
                    })
                    .build();
            osama.followTrajectorySequence(go_pune);
            //tine brat ridicat 1
            se_ridica_brat(power_brat_dc);

            //traj revenire 1 && lasa con 1
            TrajectorySequence reven = osama.trajectorySequenceBuilder(go_pune.end())
                    .waitSeconds(0.2)
                    .addTemporalMarker(0, () -> {
                        osama.cerseste();
                    })
                    .waitSeconds(0.2)
                    .strafeLeft(2)
                    .build();

            //opreste ridicare
            se_ridica_brat(0);
            osama.followTrajectorySequence(reven);

            timer.reset();
            //thing revenire 1
            while (opModeIsActive() && !osama.getMagnetAtingere()) {
                osama.rotesteThing(0.4);
                if(timer.seconds() >= TIMER_SENZOR_ST)
                    break;
            }
            osama.rotesteThing(0);

            osama.bagaViteza(0, 0, 0, 0);

            //coboara brat 1
//            brat.setTargetPosition(-cob1);
//            brat_pe_sub.setTargetPosition(-cob1);

            double ticks = 320;

            brat.setTargetPosition(brat.getCurrentPosition() - 1471); //am facut de la 61 71
            brat_pe_sub.setTargetPosition(brat_pe_sub.getCurrentPosition() - 1471);
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
            Trajectory rr = osama.trajectoryBuilder(reven.end())
                    .splineToLinearHeading(STACK_ST_RED_BLUE, Math.toRadians(180),
                            SampleMecanumDrive.getVelocityConstraint(
                                    35,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(
                                    35
                    ))
                    .build();
            osama.followTrajectory(rr);

            //miscare sasiu senzor dist 1
//            while (opModeIsActive() && osama.getDistanceSensorJos() >= 10.9) {
//                osama.bagaViteza(0.2, 0.2, 0.2, 0.2);
//            }
//            osama.bagaViteza(0, 0, 0, 0);
//
//            while (opModeIsActive() && osama.getDistanceSensorJos() <= 4) {
//                osama.bagaViteza(-0.2, -0.2, -0.2, -0.2);
//            }
//            osama.bagaViteza(0, 0, 0, 0);
//
            /////////^^^^^^^^senzor nu mai
            sleep(100);

            //apuca con 2
            osama.black();
            sleep(400);

            //ridica brat 2
            se_ridica_brat(power_brat_dc);

            //updateaza pozitia robot
            osama.update();

            //traj la junction con 2
            Trajectory back = osama.trajectoryBuilder(osama.getPoseEstimate())
                    .addTemporalMarker(time -> time * 0.4, () -> {
                        osama.rotesteThing(-0.4);
                    })
//                    .lineToConstantHeading(new Vector2d(40, -5))
                    .lineToLinearHeading(JUNCTION_THING_ST_RED_BLUE,
                            SampleMecanumDrive.getVelocityConstraint(40,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .build();
            osama.followTrajectory(back);

            //tine brat ridicat  2
            se_ridica_brat(power_brat_dc);

            //opreste rotire thing
            osama.rotesteThing(0);

            //lasa con 2
            sleep(200);
            osama.cerseste();
            sleep(200);

            //revenire thing pt alt con
            timer.reset();
            while (opModeIsActive() && !osama.getMagnetAtingere()) {
                osama.rotesteThing(0.4);
                if(timer.seconds() >= TIMER_SENZOR_ST)
                    break;
            }
            //opreste rotire thing
            osama.rotesteThing(0);

            //coboara brat 3
//            brat.setTargetPosition(cob2);
//            brat_pe_sub.setTargetPosition(cob2);

            brat.setTargetPosition(brat.getCurrentPosition() - 1675);
            brat_pe_sub.setTargetPosition(brat_pe_sub.getCurrentPosition() - 1675);

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
            osama.bagaViteza(0, 0, 0, 0);

            //traj la stack 2
            Trajectory rr2 = osama.trajectoryBuilder(osama.getPoseEstimate())
                    .splineToLinearHeading(STACK_ST_RED_BLUE2, Math.toRadians(180),
                            SampleMecanumDrive.getVelocityConstraint(
                                    35,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(
                                    35
                            ))
                    .build();
            osama.followTrajectory(rr2);

            //miscare sasiu senzor dist 2
//            while (opModeIsActive() && osama.getDistanceSensorJos() >= 10) {
//                osama.bagaViteza(0.2, 0.2, 0.2, 0.2);
//            }
//            osama.bagaViteza(0, 0, 0, 0);
//
//            while (opModeIsActive() && osama.getDistanceSensorJos() <= 4) {
//                osama.bagaViteza(-0.2, -0.2, -0.2, -0.2);
//            }
//            osama.bagaViteza(0, 0, 0, 0);
            //////^^senzor nu mai

            //apuca con 3
            sleep(100);
            osama.black();
            sleep(300);

            //ridica brat 3
            se_ridica_brat(0.6);
//            sleep(400);

            //miscare sasiu senzor dist 1
            osama.update();

            //traj la junction con 3
            Trajectory back2 = osama.trajectoryBuilder(osama.getPoseEstimate())
                    .addTemporalMarker(time -> time * 0.4, () -> {
                        osama.rotesteThing(-0.4);
                    })
//                    .lineToConstantHeading(new Vector2d(39.2, -5))
                    .lineToLinearHeading(JUNCTION_THING_ST_RED_BLUE2,
                            SampleMecanumDrive.getVelocityConstraint(40,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .build();
            osama.followTrajectory(back2);

            //tine brat ridicat 3
            se_ridica_brat(power_brat_dc);

            //opreste rotire thing
            osama.rotesteThing(0);

            //lasa con 3
            sleep(200);
            osama.cerseste();
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
                    mijloc();
                    break;
            }
            stop();
        }
    }

    private void dreapta() {
        TrajectorySequence park = osama.trajectorySequenceBuilder(osama.getPoseEstimate())
                .strafeLeft(4)
                .back(18)
                .build();
        osama.followTrajectorySequence(park);

        timer.reset();
        while (opModeIsActive() && !osama.getMagnetAtingere()) {
            osama.rotesteThing(1);
            if(timer.seconds() >= TIMER_SENZOR_ST)
                break;
        }
        osama.rotesteThing(0);
    }

    private void mijloc() {
        Trajectory park = osama.trajectoryBuilder(osama.getPoseEstimate())
                .forward(4)
                .build();
        osama.followTrajectory(park);

        timer.reset();
        while (opModeIsActive() && !osama.getMagnetAtingere()) {
            osama.rotesteThing(1);
            if(timer.seconds() >= TIMER_SENZOR_ST)
                break;
        }
        osama.rotesteThing(0);
    }

    private void stanga () {
//        Trajectory park = osama.trajectoryBuilder(osama.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(62, -8.2, Math.toRadians(0)))
//                .build();
//        osama.followTrajectory(park);

        //revenire thing pt con 4
        timer.reset();
        while (opModeIsActive() && !osama.getMagnetAtingere()) {
            osama.rotesteThing(0.5);
            if(timer.seconds() >= TIMER_SENZOR_ST)
                break;
        }
        osama.rotesteThing(0);


        brat.setTargetPosition(brat.getCurrentPosition() - 1750);
        brat_pe_sub.setTargetPosition(brat_pe_sub.getCurrentPosition() - 1750);

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

        osama.bagaViteza(0, 0, 0, 0);

//
        TrajectorySequence rr2 = osama.trajectorySequenceBuilder(osama.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-60, -9, Math.toRadians(180)))
//                    .strafeRight(3)
//                .splineToLinearHeading(new Pose2d(50, -8, Math.toRadians(0)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(63, -8, Math.toRadians(0)), Math.toRadians(0))
//            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30)
//                    .lineToConstantHeading(new Vector2d(60, -8))
                .build();
        osama.followTrajectorySequence(rr2);

        while (opModeIsActive() && osama.getDistanceSensorJos() >= 9.8) {
            osama.bagaViteza(0.2, 0.2, 0.2, 0.2);
        }
        osama.bagaViteza(0, 0, 0, 0);

        while (opModeIsActive() && osama.getDistanceSensorJos() <= 4) {
            osama.bagaViteza(-0.2, -0.2, -0.2, -0.2);
        }
        osama.bagaViteza(0, 0, 0, 0);
        osama.update();

        sleep(100);

        osama.black();

        sleep(500);
        osama.rotesteThing(1);

        se_ridica_brat(0.8);
        sleep(550);
        se_ridica_brat(0.01);

        Trajectory park = osama.trajectoryBuilder(osama.getPoseEstimate())
                .strafeTo(new Vector2d(osama.getPoseEstimate().getX() + 4.8, osama.getPoseEstimate().getY() - 2.8))
                .addDisplacementMarker(() -> {
                    osama.cerseste();
                })
                .build();
        osama.followTrajectory(park);

        Trajectory ok = osama.trajectoryBuilder(park.end())
                .strafeTo(new Vector2d(osama.getPoseEstimate().getX() - 2.3, osama.getPoseEstimate().getY() + 4.7))
                .build();
        osama.followTrajectory(ok);

        //        sleep(50);
//        osama.rotesteThing(1);
//        while (  opModeIsActive() && !osama.getMagnetAtingere())
//            osama.rotesteThing(-1);
//        osama.rotesteThing(0);
    }

    private void se_ridica_brat(double putere) {
        brat.setPower(putere);//eu si stefanut ne bazam pe robot la regionala ca sa fim top 15 asa ca nu il stricati va kiss va alea
        brat_pe_sub.setPower(-putere); // daca nu ne calificam la nationala inseamna ca nu ati lucrat corect va kiss love va alea
    }
    //imi pare rau daca am stricat ceva sincer dar nu am vrut sa stiti ca nu am modificat nimic dar sper ca  nu am schimbat ceva din greseala sorry va kiss va alea

    //va omor, va kiss va alea
}
