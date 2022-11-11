package org.firstinspires.ftc.teamcode.Amin.autoiasi;

import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.LOW_POWER;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.MEDIUM_POWER;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_GLISIERA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_INTAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(name = "PistoletoBlindato")
public class fielcentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            while (gamepad1.dpad_down) {
                drive.bagaViteza(-LOW_POWER, -LOW_POWER, -LOW_POWER, -LOW_POWER);
            }
            while (gamepad1.dpad_right) {
                drive.bagaViteza(MEDIUM_POWER, -MEDIUM_POWER, -MEDIUM_POWER, MEDIUM_POWER);
            }
            while (gamepad1.dpad_up) {
                drive.bagaViteza(LOW_POWER, LOW_POWER, LOW_POWER, LOW_POWER);
            }
            while (gamepad1.dpad_left) {
                drive.bagaViteza(-MEDIUM_POWER, MEDIUM_POWER, MEDIUM_POWER, -MEDIUM_POWER);
            }

            // rotiri fine din triggere
            while (gamepad1.right_trigger != 0) {
                drive.bagaViteza(LOW_POWER, -LOW_POWER, LOW_POWER, -LOW_POWER);
            }
            while (gamepad1.left_trigger != 0) {
                drive.bagaViteza(-LOW_POWER, LOW_POWER, -LOW_POWER, LOW_POWER);
            }
            // glisiera
            if (gamepad2.left_bumper) {
                drive.setGliseraPower(POWER_GLISIERA);
            } else if (gamepad2.right_bumper) {
                drive.setGliseraPower(-POWER_GLISIERA);
            } else {
                drive.setGliseraPower(0);
            }

            // intake
            if (gamepad2.left_trigger!=0) {
                drive.setIntake(POWER_INTAKE);
            } else if (gamepad2.right_trigger !=0 ) {
                drive.setIntake(-POWER_INTAKE);
            } else {
                drive.setIntake(0);
            }
    // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}