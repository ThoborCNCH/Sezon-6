package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous()
public class Auto_test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.setPoseEstimate(START_DR_RED_BLUE);

        waitForStart();
        while (opModeIsActive()) {
            TrajectorySequence go_pune = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
//                    .setReversed(true)
//                    .lineToLinearHeading(new Pose2d(-38, -33, Math.toRadians(90)))
//                    .lineToLinearHeading(new Pose2d(-27.08, 4.20, Math.toRadians(55)))
                    .lineToLinearHeading(PRE_POSITION_DR_RED_BLUE,
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .splineToConstantHeading(JUNCTION_PUNE_DR_RED_BLUE_VECTOR, Math.toRadians(90))
//                    .lineToLinearHeading(JUNCTION_PUNE_DR_RED_BLUE)
                    .setReversed(true)
                    .splineToLinearHeading(INTRE_TOT_DR_RED_BLUE, Math.toRadians(0))
                    .setReversed(false)
                    .turn(Math.toRadians(-90))
                    .splineToLinearHeading(STACK_DR_RED_BLUE, Math.toRadians(0))
                    .lineToLinearHeading(INTRE_TOT_DR_RED_BLUE)
                    .lineToLinearHeading(JUNCTION_PUNE_DR_RED_BLUE)
                    .build();
            robot.followTrajectorySequence(go_pune);
            stop();
            Pose2d pose = robot.getPoseEstimate();
            telemetry.addData("", pose.getX());
            telemetry.addData("", pose.getY());
            telemetry.update();
        }

    }
}
