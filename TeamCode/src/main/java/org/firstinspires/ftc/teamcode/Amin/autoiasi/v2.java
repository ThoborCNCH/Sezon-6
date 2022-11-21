package org.firstinspires.ftc.teamcode.Amin.autoiasi;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.*;

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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name = "stg red && stg blue")
public class v2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    SampleMecanumDrive robot;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    static final double FEET_PER_METER = 3.28084;


    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public DcMotor glisiera1;
    public DcMotor glisiera2;
    public CRServo marus;

    public static double fata = 25;
    public static double unghi1 = -40;
    public static double unghi2 = 40;
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

    AprilTagDetection tagOfInterest = new AprilTagDetection();

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");

        glisiera1 = hardwareMap.get(DcMotor.class, "glisiera1");
        glisiera2 = hardwareMap.get(DcMotor.class, "glisiera2");

        marus = hardwareMap.get(CRServo.class, "marus");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        robot = new SampleMecanumDrive(hardwareMap);

        robot.setPoseEstimate(START_ST_RED_BLUE);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();
        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        telemetry.addLine(String.valueOf(tag.id));
                        telemetry.update();
                        tagOfInterest = tag;
                    }
                }
            } else {
                tagOfInterest.id = 2;
            }

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
            }


            telemetry.update();
            stop();
        }

    }

    private void mijloc() throws InterruptedException {
        Trajectory pozitionare = robot.trajectoryBuilder(START_ST_RED_BLUE)
                .lineToLinearHeading(POSITION_ST_RED_BLUE)
                .addTemporalMarker(0, () -> {
                    robot.setGliseraPower(1);
                })
                .addTemporalMarker(1.00, () -> {
                    robot.setGliseraPower(0);
                })
                .build();
        robot.followTrajectory(pozitionare);

        Trajectory pune_ceva_macar_te_rog = robot.trajectoryBuilder(pozitionare.end())
                .forward(6.8)
                .build();
        robot.followTrajectory(pune_ceva_macar_te_rog);

        robot.setIntake(1);
        sleep(2000);
        robot.setIntake(0);

        Trajectory oleaka_inapoi = robot.trajectoryBuilder(pune_ceva_macar_te_rog.end())
                .lineToLinearHeading(BACK_A_LITTLE_ST_RED_BLUE)
                .build();
        robot.followTrajectory(oleaka_inapoi);

//        DE AICI INCEPE AIA NOUA COPIAZA ANIMALE
        Trajectory mai_ia_unu = robot.trajectoryBuilder(oleaka_inapoi.end())
                .lineToLinearHeading(new Pose2d(-60.0, -45, Math.toRadians(180)))
                .build();
        robot.followTrajectory(mai_ia_unu);
        robot.setGliseraPower(-.67);
        sleep(415);
        robot.setGliseraPower(0);

        robot.setIntake(-1);
        sleep(2060);

//        DE ASTA MERGE IN FATA ---------------------
//        Trajectory mai_ia_unu2 = robot.trajectoryBuilder(mai_ia_unu.end())
//                .forward(1)
//                .build();
//        robot.followTrajectory(mai_ia_unu2);
        robot.setGliseraPower(.8);
        sleep(700);
        robot.setGliseraPower(0);

        Trajectory mai_ia_unu3 = robot.trajectoryBuilder(mai_ia_unu.end())
                .back(7)
                .build();
        robot.followTrajectory(mai_ia_unu3);

//        robot.turn(Math.toRadians(10));
//        robot.turn(Math.toRadians(-20));
//        robot.turn(Math.toRadians(20));
//        robot.turn(Math.toRadians(-10));

        TrajectorySequence btb = robot.trajectorySequenceBuilder(mai_ia_unu3.end())
                .lineToLinearHeading(new Pose2d(-36, -38, Math.toRadians(33)))
//                .addTemporalMarker(0, () -> {
//                    robot.setGliseraPower(1);
//                })
//                .addTemporalMarker(0.35, () -> {
//                    robot.setGliseraPower(0);
//                })
                .forward(6.6)
                .build();
        robot.followTrajectorySequence(btb);

        robot.setIntake(1);
        sleep(2000);
        robot.setIntake(0);

        Trajectory oleaka_inapoi_part2 = robot.trajectoryBuilder(btb.end())
                .lineToLinearHeading(BACK_A_LITTLE_ST_RED_BLUE)
                .build();
        robot.followTrajectory(oleaka_inapoi_part2);

    }

    private void stanga() throws InterruptedException {
        Trajectory pozitionare = robot.trajectoryBuilder(START_ST_RED_BLUE)
                .lineToLinearHeading(POSITION_ST_RED_BLUE)
                .addTemporalMarker(0, () -> {
                    robot.setGliseraPower(1);
                })
                .addTemporalMarker(1.4, () -> {
                    robot.setGliseraPower(0);
                })
                .build();
        robot.followTrajectory(pozitionare);

        Trajectory pune_ceva_macar_te_rog = robot.trajectoryBuilder(pozitionare.end())
                .forward(6.8)
                .build();
        robot.followTrajectory(pune_ceva_macar_te_rog);

        robot.setIntake(1);
        sleep(2000);
        robot.setIntake(0);

        Trajectory oleaka_inapoi = robot.trajectoryBuilder(pune_ceva_macar_te_rog.end())
                .lineToLinearHeading(BACK_A_LITTLE_ST_RED_BLUE)
                .build();
        robot.followTrajectory(oleaka_inapoi);

//        DE AICI INCEPE AIA NOUA COPIAZA ANIMALE
        Trajectory mai_ia_unu = robot.trajectoryBuilder(oleaka_inapoi.end())
                .lineToLinearHeading(new Pose2d(-60, -44, Math.toRadians(180)))
                .build();
        robot.followTrajectory(mai_ia_unu);
        robot.setGliseraPower(-.70);
        sleep(460);
        robot.setGliseraPower(0);

        robot.setIntake(-1);
        sleep(2000);

        Trajectory mai_ia_unu2 = robot.trajectoryBuilder(mai_ia_unu.end())
                .forward(1)
                .build();
        robot.followTrajectory(mai_ia_unu2);
        robot.setGliseraPower(.8);
        sleep(500);
        robot.setGliseraPower(0);

        Trajectory mai_ia_unu3 = robot.trajectoryBuilder(mai_ia_unu2.end())
                .back(7)
                .build();
        robot.followTrajectory(mai_ia_unu3);

        robot.turn(Math.toRadians(10));
        robot.turn(Math.toRadians(-20));
        robot.turn(Math.toRadians(20));
        robot.turn(Math.toRadians(-10));

        TrajectorySequence btb = robot.trajectorySequenceBuilder(mai_ia_unu3.end())
                .lineToLinearHeading(new Pose2d(-36, -38, Math.toRadians(32)))
                .addTemporalMarker(0, () -> {
                    robot.setGliseraPower(1);
                })
                .addTemporalMarker(0.8, () -> {
                    robot.setGliseraPower(0);
                })
                .forward(6.6)
                .build();
        robot.followTrajectorySequence(btb);

        robot.setIntake(1);
        sleep(2000);
        robot.setIntake(0);

        Trajectory oleaka_inapoi_part2 = robot.trajectoryBuilder(btb.end())
                .lineToLinearHeading(BACK_A_LITTLE_ST_RED_BLUE)
                .build();
        robot.followTrajectory(oleaka_inapoi_part2);
        Trajectory hai_du_te_la_parcare = robot.trajectoryBuilder(oleaka_inapoi_part2.end())
                .back(23)
                .build();
        robot.followTrajectory(hai_du_te_la_parcare);
    }

    private void dreapta() throws InterruptedException {
        Trajectory pozitionare = robot.trajectoryBuilder(START_ST_RED_BLUE)
                .lineToLinearHeading(POSITION_ST_RED_BLUE)
                .addTemporalMarker(0, () -> {
                    robot.setGliseraPower(1);
                })
                .addTemporalMarker(1.4, () -> {
                    robot.setGliseraPower(0);
                })
                .build();
        robot.followTrajectory(pozitionare);

        Trajectory pune_ceva_macar_te_rog = robot.trajectoryBuilder(pozitionare.end())
                .forward(6.8)
                .build();
        robot.followTrajectory(pune_ceva_macar_te_rog);

        robot.setIntake(1);
        sleep(2000);
        robot.setIntake(0);

        Trajectory oleaka_inapoi = robot.trajectoryBuilder(pune_ceva_macar_te_rog.end())
                .lineToLinearHeading(BACK_A_LITTLE_ST_RED_BLUE)
                .build();
        robot.followTrajectory(oleaka_inapoi);

//        DE AICI INCEPE AIA NOUA COPIAZA ANIMALE
        Trajectory mai_ia_unu = robot.trajectoryBuilder(oleaka_inapoi.end())
                .lineToLinearHeading(new Pose2d(-60, -44, Math.toRadians(180)))
                .build();
        robot.followTrajectory(mai_ia_unu);
        robot.setGliseraPower(-.70);
        sleep(460);
        robot.setGliseraPower(0);

        robot.setIntake(-1);
        sleep(2000);

        Trajectory mai_ia_unu2 = robot.trajectoryBuilder(mai_ia_unu.end())
                .forward(1)
                .build();
        robot.followTrajectory(mai_ia_unu2);
        robot.setGliseraPower(.8);
        sleep(500);
        robot.setGliseraPower(0);

        Trajectory mai_ia_unu3 = robot.trajectoryBuilder(mai_ia_unu2.end())
                .back(7)
                .build();
        robot.followTrajectory(mai_ia_unu3);

        robot.turn(Math.toRadians(10));
        robot.turn(Math.toRadians(-20));
        robot.turn(Math.toRadians(20));
        robot.turn(Math.toRadians(-10));

        TrajectorySequence btb = robot.trajectorySequenceBuilder(mai_ia_unu3.end())
                .lineToLinearHeading(new Pose2d(-36, -38, Math.toRadians(32)))
                .addTemporalMarker(0, () -> {
                    robot.setGliseraPower(1);
                })
                .addTemporalMarker(0.8, () -> {
                    robot.setGliseraPower(0);
                })
                .forward(6.6)
                .build();
        robot.followTrajectorySequence(btb);

        robot.setIntake(1);
        sleep(2000);
        robot.setIntake(0);

        Trajectory oleaka_inapoi_part2 = robot.trajectoryBuilder(btb.end())
                .lineToLinearHeading(BACK_A_LITTLE_ST_RED_BLUE)
                .build();
        robot.followTrajectory(oleaka_inapoi_part2);
        Trajectory hai_du_te_la_parcare = robot.trajectoryBuilder(oleaka_inapoi_part2.end())
                .forward(23)
                .build();
        robot.followTrajectory(hai_du_te_la_parcare);
    }

}