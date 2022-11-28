package org.firstinspires.ftc.teamcode.Amin;

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

@Config
@Autonomous(name = "stg red && stg blue")
public class v2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    SampleMecanumDrive glisieraHandle;
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

//        glisiera1 = hardwareMap.get(DcMotor.class, "glisiera1");
//        glisiera2 = hardwareMap.get(DcMotor.class, "glisiera2");

        marus = hardwareMap.get(CRServo.class, "marus");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

//        glisieraHandle = new SampleMecanumDrive(hardwareMap);
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

        robot.glisiera1.setDirection(DcMotor.Direction.REVERSE);
//        robot.glisiera2.setDirection(DcMotor.Direction.REVERSE);

//        robot.glisiera1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.glisiera2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
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
            robot.setIntake(-1);

            switch (tagOfInterest.id) {
                case 1:
                    telemetry.addLine(String.valueOf(tagOfInterest.id));
                    stanga();
                    break;
                case 2:
                    telemetry.addLine(String.valueOf(tagOfInterest.id));
                    mijloc();
                    break;
                case 3:
                    telemetry.addLine(String.valueOf(tagOfInterest.id));
                    dreapta();
                    break;
            }


            telemetry.update();
            stop();
        }

    }


    private void stanga() throws InterruptedException {
        Trajectory putin = robot.trajectoryBuilder(START_ST_RED_BLUE)
                .lineToLinearHeading(new Pose2d(-37, -31, Math.toRadians(0)))
                .addDisplacementMarker(t -> t * .1, () -> {
                    robot.setGliseraPower(1);
                })
                .addDisplacementMarker(t -> t * .8, () -> {
                    robot.setGliseraPower(0);
                })
                .build();
        robot.followTrajectory(putin);
        TrajectorySequence aproape_saPui = robot.trajectorySequenceBuilder(putin.end())
                .forward(6.7)
                .addTemporalMarker(1, () -> {
                    robot.setIntake(1);
                })
                .waitSeconds(1.75)
                .back(6)
                .build();
        robot.followTrajectorySequence(aproape_saPui);

        robot.setIntake(0);

        TrajectorySequence ia = robot.trajectorySequenceBuilder(aproape_saPui.end())
                .lineToLinearHeading(new Pose2d(-40, -20, Math.toRadians(90)))
//                .splineToLinearHeading(new Pose2d(-40, -20, Math.toRadians(180)), Math.toRadians(-20))
                .lineToLinearHeading(new Pose2d(-57.5, -22, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(26)
                )
                .addDisplacementMarker(t -> t * 0.3, () -> {
                    robot.setGliseraPower(-.32);
                })
                .addDisplacementMarker(t -> t * 0.379, () -> {
                    robot.setGliseraPower(0);
                })
                .build();
        robot.followTrajectorySequence(ia);

        robot.setIntake(-1);

        Trajectory fmm = robot.trajectoryBuilder(ia.end())
                .forward(2.7,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
//                .addDisplacementMarker(t -> t * 0.8, () -> {
//                    robot.setGliseraPower(-.474);
//                })
//                .addTemporalMarker(1.967, () -> {
//                    robot.setGliseraPower(0);
//                })
                .build();

        robot.followTrajectory(fmm);
        robot.setGliseraPower(-.474);
        sleep(600);
        robot.setGliseraPower(0);


        robot.setIntake(-1);
        sleep(1000);
//        robot.setIntake(0);

        robot.setGliseraPower(1);
        sleep(850);
        robot.setGliseraPower(0);

        Trajectory bb = robot.trajectoryBuilder(fmm.end())
                .back(20)
                .build();
        robot.followTrajectory(bb);

//        Trajectory pune_2 = robot.trajectoryBuilder(bb.end(), true)
//                .splineToLinearHeading(new Pose2d(-30, -33, Math.toRadians(10)), Math.toRadians(-40))
//                .build();
//        robot.followTrajectory(pune_2);

        TrajectorySequence pune_2 = robot.trajectorySequenceBuilder(bb.end())
                .lineToLinearHeading(new Pose2d(-40, -20, Math.toRadians(-33)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, 20, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(12)
                .build();
        robot.followTrajectorySequence(pune_2);


        robot.setIntake(1);
        sleep(2000);
        robot.setIntake(0);

        TrajectorySequence park = robot.trajectorySequenceBuilder(pune_2.end())
                .back(7)
                .lineToLinearHeading(BACK_A_LITTLE_ST_RED_BLUE)
                .forward(23)
                .build();
        robot.followTrajectorySequence(park);


    }

    private void mijloc() throws InterruptedException {
        Trajectory putin = robot.trajectoryBuilder(START_ST_RED_BLUE)
                .lineToLinearHeading(new Pose2d(-37, -31, Math.toRadians(0)))
                .addDisplacementMarker(t -> t * .1, () -> {
                    robot.setGliseraPower(1);
                })
                .addDisplacementMarker(t -> t * .8, () -> {
                    robot.setGliseraPower(0);
                })
                .build();
        robot.followTrajectory(putin);
        TrajectorySequence aproape_saPui = robot.trajectorySequenceBuilder(putin.end())
                .forward(6.7)
                .addTemporalMarker(1, () -> {
                    robot.setIntake(1);
                })
                .waitSeconds(1.75)
                .back(6)
                .build();
        robot.followTrajectorySequence(aproape_saPui);

        robot.setIntake(0);

        TrajectorySequence ia = robot.trajectorySequenceBuilder(aproape_saPui.end())
                .lineToLinearHeading(new Pose2d(-40, -20, Math.toRadians(90)))
//                .splineToLinearHeading(new Pose2d(-40, -20, Math.toRadians(180)), Math.toRadians(-20))
                .lineToLinearHeading(new Pose2d(-57.5, -22, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(26)
                )
                .addDisplacementMarker(t -> t * 0.3, () -> {
                    robot.setGliseraPower(-.32);
                })
                .addDisplacementMarker(t -> t * 0.379, () -> {
                    robot.setGliseraPower(0);
                })
                .build();
        robot.followTrajectorySequence(ia);

        robot.setIntake(-1);

        Trajectory fmm = robot.trajectoryBuilder(ia.end())
                .forward(2.7,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
//                .addDisplacementMarker(t -> t * 0.8, () -> {
//                    robot.setGliseraPower(-.474);
//                })
//                .addTemporalMarker(1.967, () -> {
//                    robot.setGliseraPower(0);
//                })
                .build();

        robot.followTrajectory(fmm);
        robot.setGliseraPower(-.474);
        sleep(600);
        robot.setGliseraPower(0);


        robot.setIntake(-1);
        sleep(1000);
//        robot.setIntake(0);

        robot.setGliseraPower(1);
        sleep(850);
        robot.setGliseraPower(0);

        Trajectory bb = robot.trajectoryBuilder(fmm.end())
                .back(20)
                .build();
        robot.followTrajectory(bb);

//        Trajectory pune_2 = robot.trajectoryBuilder(bb.end(), true)
//                .splineToLinearHeading(new Pose2d(-30, -33, Math.toRadians(10)), Math.toRadians(-40))
//                .build();
//        robot.followTrajectory(pune_2);

        TrajectorySequence pune_2 = robot.trajectorySequenceBuilder(bb.end())
                .lineToLinearHeading(new Pose2d(-40, -20, Math.toRadians(-33)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, 20, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(12)
                .build();
        robot.followTrajectorySequence(pune_2);


        robot.setIntake(1);
        sleep(2000);
        robot.setIntake(0);

        TrajectorySequence park = robot.trajectorySequenceBuilder(pune_2.end())
                .back(7)
                .lineToLinearHeading(BACK_A_LITTLE_ST_RED_BLUE)
                .build();
        robot.followTrajectorySequence(park);


    }

    private void dreapta() throws InterruptedException {
        Trajectory putin = robot.trajectoryBuilder(START_ST_RED_BLUE)
                .lineToLinearHeading(new Pose2d(-37, -31, Math.toRadians(0)))
                .addDisplacementMarker(t -> t * .1, () -> {
                    robot.setGliseraPower(1);
                })
                .addDisplacementMarker(t -> t * .8, () -> {
                    robot.setGliseraPower(0);
                })
                .build();
        robot.followTrajectory(putin);
        TrajectorySequence aproape_saPui = robot.trajectorySequenceBuilder(putin.end())
                .forward(6.7)
                .addTemporalMarker(1, () -> {
                    robot.setIntake(1);
                })
                .waitSeconds(1.75)
                .back(6)
                .build();
        robot.followTrajectorySequence(aproape_saPui);

        robot.setIntake(0);

        TrajectorySequence ia = robot.trajectorySequenceBuilder(aproape_saPui.end())
                .lineToLinearHeading(new Pose2d(-40, -20, Math.toRadians(90)))
//                .splineToLinearHeading(new Pose2d(-40, -20, Math.toRadians(180)), Math.toRadians(-20))
                .lineToLinearHeading(new Pose2d(-57.5, -22, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(26)
                )
                .addDisplacementMarker(t -> t * 0.3, () -> {
                    robot.setGliseraPower(-.32);
                })
                .addDisplacementMarker(t -> t * 0.379, () -> {
                    robot.setGliseraPower(0);
                })
                .build();
        robot.followTrajectorySequence(ia);

        robot.setIntake(-1);

        Trajectory fmm = robot.trajectoryBuilder(ia.end())
                .forward(2.7,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
//                .addDisplacementMarker(t -> t * 0.8, () -> {
//                    robot.setGliseraPower(-.474);
//                })
//                .addTemporalMarker(1.967, () -> {
//                    robot.setGliseraPower(0);
//                })
                .build();

        robot.followTrajectory(fmm);
        robot.setGliseraPower(-.474);
        sleep(600);
        robot.setGliseraPower(0);


        robot.setIntake(-1);
        sleep(1000);
//        robot.setIntake(0);

        robot.setGliseraPower(1);
        sleep(850);
        robot.setGliseraPower(0);

        Trajectory bb = robot.trajectoryBuilder(fmm.end())
                .back(20)
                .build();
        robot.followTrajectory(bb);

//        Trajectory pune_2 = robot.trajectoryBuilder(bb.end(), true)
//                .splineToLinearHeading(new Pose2d(-30, -33, Math.toRadians(10)), Math.toRadians(-40))
//                .build();
//        robot.followTrajectory(pune_2);

        TrajectorySequence pune_2 = robot.trajectorySequenceBuilder(bb.end())
                .lineToLinearHeading(new Pose2d(-40, -20, Math.toRadians(-33)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, 20, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(12)
                .build();
        robot.followTrajectorySequence(pune_2);


        robot.setIntake(1);
        sleep(2000);
        robot.setIntake(0);

        TrajectorySequence park = robot.trajectorySequenceBuilder(pune_2.end())
                .back(7)
                .lineToLinearHeading(BACK_A_LITTLE_ST_RED_BLUE)
                .build();
        robot.followTrajectorySequence(park);


        Trajectory hai_du_te_la_parcare = robot.trajectoryBuilder(park.end())
                .back(23)
                .build();
        robot.followTrajectory(hai_du_te_la_parcare);
    }

}