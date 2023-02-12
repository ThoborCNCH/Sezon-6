package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.JUNCTION_THING_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.JUNCTION_THING_DR_RED_BLUE2;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.PRE_POSITION_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.PRE_POSITION_DR_RED_BLUE3;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.PRE_POSITION_DR_RED_BLUE_KKK;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.STACK_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.STACK_DR_RED_BLUE2;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.START_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_brat_dc;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_deschis_dr;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_deschis_st;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_inschis_dr_AUTO;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_inschis_st_AUTO;

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
public class Encodere extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();

        brat = hardwareMap.dcMotor.get("brat");
        brat_pe_sub = hardwareMap.dcMotor.get("brat_pe_sub");

        osama = new SampleMecanumDrive(hardwareMap);
        osama.setPoseEstimate(START_DR_RED_BLUE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        brat.setDirection(DcMotorSimple.Direction.REVERSE);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat_pe_sub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat_pe_sub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        osama.black();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("baterie: ", String.valueOf(batteryVoltageSensor.getVoltage()));
//            telemetry.addData("id: ", String.valueOf(tagOfInterest.id));
            telemetry.update();

            //apuca con 1
            sleep(400);
            osama.black();

            //ridica brat 1
            sleep(300);
            se_ridica_brat(power_brat_dc);

            //traj junction 1
            TrajectorySequence go_pune = osama.trajectorySequenceBuilder(osama.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(41.5, -40, Math.toRadians(0)))
                    .lineToLinearHeading(PRE_POSITION_DR_RED_BLUE_KKK,
                            SampleMecanumDrive.getVelocityConstraint(40,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .addTemporalMarker(time -> time * 0.4, () -> {
                        osama.rotesteThing(1);
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
                    .strafeRight(2)
                    .build();

            //opreste ridicare
            se_ridica_brat(0);
            osama.followTrajectorySequence(reven);

            //thing revenire 1
            while (!osama.getMagnetAtingere()) {
                osama.rotesteThing(-0.7);
            }
            osama.rotesteThing(0);

            osama.bagaViteza(0, 0, 0, 0);

            //coboara brat 1
            brat.setTargetPosition(-400);
            brat_pe_sub.setTargetPosition(-400);

            brat.setPower(-0.4);
            brat_pe_sub.setPower(0.4);
//
            while (opModeIsActive() && brat.isBusy() && brat_pe_sub.isBusy()) {
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
                    .splineToLinearHeading(STACK_DR_RED_BLUE, Math.toRadians(0))
                    .build();
            osama.followTrajectory(rr);

            //miscare sasiu senzor dist 1
            while (osama.getDistanceSensorJos() >= 10.9) {
                osama.bagaViteza(0.2, 0.2, 0.2, 0.2);
            }
            osama.bagaViteza(0, 0, 0, 0);

            while (osama.getDistanceSensorJos() <= 4) {
                osama.bagaViteza(-0.2, -0.2, -0.2, -0.2);
            }
            osama.bagaViteza(0, 0, 0, 0);

            sleep(100);

            //apuca con 2
            osama.black();
            sleep(500);

            //ridica brat 2
            se_ridica_brat(0.6);

            //updateaza pozitia robot
            osama.update();

            //traj la junction con 2
            Trajectory back = osama.trajectoryBuilder(osama.getPoseEstimate())
                    .addTemporalMarker(time -> time * 0.4, () -> {
                        osama.rotesteThing(0.5);
                    })
//                    .lineToConstantHeading(new Vector2d(40, -5))
                    .lineToLinearHeading(JUNCTION_THING_DR_RED_BLUE2,
                            SampleMecanumDrive.getVelocityConstraint(40,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .build();
            osama.followTrajectory(back);

            //tine brat ridicat  2
            se_ridica_brat(0.6);

            //opreste rotire thing
            osama.rotesteThing(0);

            //lasa con 2
            sleep(500);
            osama.cerseste();
            sleep(500);

            //revenire thing pt alt con
            while (!osama.getMagnetAtingere()) {
                osama.rotesteThing(-0.7);
            }
            //opreste rotire thing
            osama.rotesteThing(0);

            //coboara brat 2
            brat.setTargetPosition(-400);
            brat_pe_sub.setTargetPosition(-400);

            brat.setPower(-0.4);
            brat_pe_sub.setPower(0.4);
//
            while (opModeIsActive() && brat.isBusy() && brat_pe_sub.isBusy()) {
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
                    .splineToLinearHeading(STACK_DR_RED_BLUE2, Math.toRadians(0))
                    .build();
            osama.followTrajectory(rr2);

            //miscare sasiu senzor dist 2
            while (osama.getDistanceSensorJos() >= 10) {
                osama.bagaViteza(0.2, 0.2, 0.2, 0.2);
            }
            osama.bagaViteza(0, 0, 0, 0);

            while (osama.getDistanceSensorJos() <= 4) {
                osama.bagaViteza(-0.2, -0.2, -0.2, -0.2);
            }
            osama.bagaViteza(0, 0, 0, 0);

            //apuca con 3
            sleep(100);
            osama.black();
            sleep(500);

            //ridica brat 3
            se_ridica_brat(0.6);
            sleep(500);

            //miscare sasiu senzor dist 1
            osama.update();

            //traj la junction con 3
            Trajectory back2 = osama.trajectoryBuilder(osama.getPoseEstimate())
                    .addTemporalMarker(time -> time * 0.4, () -> {
                        osama.rotesteThing(0.5);
                    })
//                    .lineToConstantHeading(new Vector2d(39.2, -5))
                    .lineToLinearHeading(JUNCTION_THING_DR_RED_BLUE2,
                            SampleMecanumDrive.getVelocityConstraint(40,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .build();
            osama.followTrajectory(back2);

            //tine brat ridicat 3
            se_ridica_brat(0.6);

            //opreste rotire thing
            osama.rotesteThing(0);

            //lasa con 3
            sleep(500);
            osama.cerseste();
            sleep(500);

            //revenire thing pt con 3
            while (!osama.getMagnetAtingere()) {
                osama.rotesteThing(-0.5);
            }
            osama.rotesteThing(0);
//

            //parcare
//            switch (tagOfInterest.id) {
//                case 1:
//                    stanga();
//                    break;
//                case 3:
//                    dreapta();
//                    break;
//                case 2:
//                    mijloc();
//                    break;
//                default:
//                    mijloc();
//                    break;
//            }
            stop();
        }
    }

    private void stanga() {
        TrajectorySequence park = osama.trajectorySequenceBuilder(osama.getPoseEstimate())
                .strafeRight(4)
                .back(18)
                .build();
        osama.followTrajectorySequence(park);
        while (!osama.getMagnetAtingere())
            osama.rotesteThing(-1);
        osama.rotesteThing(0);
    }

    private void mijloc() {
        Trajectory park = osama.trajectoryBuilder(osama.getPoseEstimate())
                .forward(4)
                .build();
        osama.followTrajectory(park);
        while (!osama.getMagnetAtingere())
            osama.rotesteThing(-1);
        osama.rotesteThing(0);
    }

    private void dreapta() {
        Trajectory park = osama.trajectoryBuilder(osama.getPoseEstimate())
                .lineToLinearHeading(STACK_DR_RED_BLUE2)
                .build();
        osama.followTrajectory(park);
//        sleep(500);
//
//        while (!osama.getMagnetAtingere() && timer.seconds() <= 0.4) {
//            osama.rotesteThing(-0.5);
//        }
//        osama.rotesteThing(0);
////
//        while (osama.getDistanceSensorSus() >= 22) {
//            telemetry.addData("1: ", String.valueOf(osama.getDistanceSensorSus()));
//            telemetry.update();
////                osama.bagaViteza(0.2, 0.2, 0.2, 0.2);
//            if (osama.getDistanceSensorSus() >= 40) {
//                se_ridica_brat(-0.4);
//            } else {
//                se_ridica_brat(-0.14);
//            }
//        }
//
//        while (osama.getDistanceSensorSus() <= 13) {
//            telemetry.addData("2: ", String.valueOf(osama.getDistanceSensorSus()));
//            telemetry.update();
//            se_ridica_brat(0.14);
//        }
//        se_ridica_brat(0.05);
//        osama.bagaViteza(0, 0, 0, 0);
//        osama.update();
//
//        TrajectorySequence rr2 = osama.trajectorySequenceBuilder(osama.getPoseEstimate())
////                    .lineToLinearHeading(STACK_DR_RED_BLUE)
////                    .strafeRight(3)
//                .splineToLinearHeading(new Pose2d(50, -8, Math.toRadians(0)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(63, -8, Math.toRadians(0)), Math.toRadians(0))
////            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30)
////                    .lineToConstantHeading(new Vector2d(60, -8))
//                .build();
//        osama.followTrajectorySequence(rr2);
//
//        while (osama.getDistanceSensorJos() >= 9.8 && timer.seconds() <= 0.2) {
//            osama.bagaViteza(0.2, 0.2, 0.2, 0.2);
//        }
//        osama.bagaViteza(0, 0, 0, 0);
//
//        while (osama.getDistanceSensorJos() <= 4) {
//            osama.bagaViteza(-0.2, -0.2, -0.2, -0.2);
//        }
//        osama.bagaViteza(0, 0, 0, 0);
//        osama.update();
//
//        sleep(100);
//
//        osama.black();
//
//        sleep(500);
//
//        se_ridica_brat(0.6);
//        osama.rotesteThing(-1);
//
//        sleep(720);
//        se_ridica_brat(0.01);
//
//        Trajectory park = osama.trajectoryBuilder(osama.getPoseEstimate())
//                .strafeTo(new Vector2d(osama.getPoseEstimate().getX() - 5.5, osama.getPoseEstimate().getY() - 4.7))
//                .addDisplacementMarker(() -> {
//                    osama.cerseste();
//                })
//                .build();
//        osama.followTrajectory(park);
////        sleep(50);
////        osama.rotesteThing(1);
//        while (!osama.getMagnetAtingere())
//            osama.rotesteThing(-1);
//        osama.rotesteThing(0);
    }
    
    private void se_ridica_brat(double putere){
        brat.setPower(putere);
        brat_pe_sub.setPower(-putere);        
    }
}
