package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous()
public class Auto_test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        GlisiereHandler glisiera = new GlisiereHandler();
        ElapsedTime timer = new ElapsedTime();

        robot.setPoseEstimate(START_DR_RED_BLUE);
        telemetry.addData("baterie:", String.valueOf(robot.batteryVoltageSensor.getVoltage()));
        telemetry.update();

        waitForStart();
        telemetry.addData("baterie:", String.valueOf(robot.batteryVoltageSensor.getVoltage()));
        telemetry.update();
        while (opModeIsActive()) {
            robot.apuca(poz_inchis_st, poz_inchis_dr);
            sleep(200);
//            robot.se_ridica_brat(power_brat_dc); --------
            sleep(10000);
            Trajectory go_pune = robot.trajectoryBuilder(robot.getPoseEstimate())
                    .lineToLinearHeading(PRE_POSITION_DR_RED_BLUE,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .addTemporalMarker(time -> time * 0, () -> {
//                        robot.se_ridica_brat(power_brat_dc);
//                    }) -----------
                    .build();
            robot.followTrajectory(go_pune);

            sleep(200);

            Trajectory efectiv = robot.trajectoryBuilder(go_pune.end())
                    .splineToConstantHeading(JUNCTION_PUNE_DR_RED_BLUE_VECTOR, Math.toRadians(90))
//                    .addTemporalMarker(time -> time * 0, () -> {
//                        robot.se_ridica_brat(power_brat_dc);
//                    }) --------------
                    .build();

            robot.followTrajectory(efectiv);
            sleep(2000);
//            robot.se_ridica_brat(0); ---------------

            sleep(200);
            robot.apuca(poz_deschis_st, poz_deschis_dr);


            Trajectory reven = robot.trajectoryBuilder(efectiv.end(), true)
                    .splineToLinearHeading(INTRE_TOT_DR_RED_BLUE, Math.toRadians(0))

//                    .addTemporalMarker(time -> time * 0.8, () -> {
//                        robot.se_ridica_brat(-power_brat_dc);
//                    }) -----------
                    .build();
//            robot.se_ridica_brat(0);

            robot.followTrajectory(reven);

            TrajectorySequence restu = robot.trajectorySequenceBuilder(reven.end())
//                    .lineToLinearHeading(JUNCTION_PUNE_DR_RED_BLUE)
                    .turn(Math.toRadians(-90))
                    .splineToLinearHeading(STACK_DR_RED_BLUE, Math.toRadians(0))
                    .lineToLinearHeading(INTRE_TOT_DR_RED_BLUE)
                    .lineToLinearHeading(JUNCTION_PUNE_DR_RED_BLUE)
                    .build();
            robot.followTrajectorySequence(restu);
            Pose2d pose = robot.getPoseEstimate();
            telemetry.addData("", pose.getX());
            telemetry.addData("", pose.getY());
            telemetry.update();
            stop();

        }

    }
}
