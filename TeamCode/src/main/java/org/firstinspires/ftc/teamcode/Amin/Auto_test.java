package org.firstinspires.ftc.teamcode.Amin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous()
public class Auto_test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.setPoseEstimate(new Pose2d(41.5, -58, Math.toRadians(90)));

        waitForStart();
        while (opModeIsActive()) {
            TrajectorySequence go_pune = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
//                    .setReversed(true)
//                    .lineToLinearHeading(new Pose2d(-38, -33, Math.toRadians(90)))
//                    .lineToLinearHeading(new Pose2d(-27.08, 4.20, Math.toRadians(55)))
                    .lineToLinearHeading(new Pose2d(43, -20, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(35, -1.7, Math.toRadians(130)))
                    .setReversed(true)
                    .lineToLinearHeading(new Pose2d(41.5, -9, Math.toRadians(90)))
                    .setReversed(false)
                    .strafeRight(20)
                    .strafeLeft(20)
                    .lineToLinearHeading(new Pose2d(35, -1.7, Math.toRadians(130)))
                    .setReversed(true)
                    .lineToLinearHeading(new Pose2d(41.5, -9, Math.toRadians(90)))
                    .setReversed(false)
                    .strafeRight(20)
                    .strafeLeft(20)
                    .lineToLinearHeading(new Pose2d(35, -1.7, Math.toRadians(130)))
                    .setReversed(true)
                    .lineToLinearHeading(new Pose2d(41.5, -9, Math.toRadians(90)))
                    .setReversed(false)
                    .strafeRight(20)
                    .strafeLeft(20)
                    .lineToLinearHeading(new Pose2d(35, -1.7, Math.toRadians(130)))
                    .setReversed(true)
                    .lineToLinearHeading(new Pose2d(41.5, -9, Math.toRadians(90)))
                    .setReversed(false)
                    .strafeRight(20)
                    .strafeLeft(20)
                    .lineToLinearHeading(new Pose2d(35, -1.7, Math.toRadians(130)))
                    .setReversed(true)
                    .lineToLinearHeading(new Pose2d(41.5, -9, Math.toRadians(90)))
                    .setReversed(false)


//                    .turn()
//                    .splineTo(new Vector2d(-36, -30), Math.toRadians(90))
//                    .setReversed(false)
//                    .splineTo(new Vector2d(-38, -17), Math.toRadians(120))
//                    .splineTo(new Vector2d(0, 0), Math.toRadians(0))

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
