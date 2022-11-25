/*
 *
 * ©Thobor 2021-2022
 *
 *           _
 *       .__(.)< (MEOW)
 *        \___)
 * ~~~~~~~~~~~~~~~~~~
 *
 *
 */
package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.LOW_POWER;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.MEDIUM_POWER;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_BRAT_MARKER;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_BRAT_TELEOP;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_GHEARA_MARKER;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_GLISIERA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_INTAKE;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_RATA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_ARUNCA_CUVA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_NORMAL_CUVA;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//disper sa moara mama
@TeleOp()
public class TeleOpulAlaBlana extends LinearOpMode {

    private double v1, v2, v3, v4;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

//        robot.glisiera1.setDirection(DcMotor.Direction.REVERSE);

        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        waitForStart();

        while (!opModeIsActive() && !isStopRequested()) {
        }

//        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // joysticks

            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.8,
                            -gamepad1.left_stick_x * 0.8,
                            -gamepad1.right_stick_x * 0.8
                    )
            );

            // miscari din dpad uri
            while (gamepad1.dpad_down) {
                robot.bagaViteza(-LOW_POWER, -LOW_POWER, -LOW_POWER, -LOW_POWER);
            }
            while (gamepad1.dpad_right) {
                robot.bagaViteza(MEDIUM_POWER, -MEDIUM_POWER, -MEDIUM_POWER, MEDIUM_POWER);
            }
            while (gamepad1.dpad_up) {
                robot.bagaViteza(LOW_POWER, LOW_POWER, LOW_POWER, LOW_POWER);
            }
            while (gamepad1.dpad_left) {
                robot.bagaViteza(-MEDIUM_POWER, MEDIUM_POWER, MEDIUM_POWER, -MEDIUM_POWER);
            }

            // rotiri fine din triggere
            while (gamepad1.right_trigger != 0) {
                robot.bagaViteza(LOW_POWER, -LOW_POWER, LOW_POWER, -LOW_POWER);
            }
            while (gamepad1.left_trigger != 0) {
                robot.bagaViteza(-LOW_POWER, LOW_POWER, -LOW_POWER, LOW_POWER);
            }
            // glisiera
            if (gamepad2.right_bumper) {
                robot.setGliseraPower(POWER_GLISIERA);
            } else if (gamepad2.left_bumper) {
                robot.setGliseraPower(-POWER_GLISIERA);
            } else {
                robot.setGliseraPower(0);
            }

            // intake
            if (gamepad2.left_trigger!=0) {
                robot.setIntake(POWER_INTAKE);
            } else if (gamepad2.right_trigger !=0 ) {
                robot.setIntake(-POWER_INTAKE);
            } else {
                robot.setIntake(0);
            }
        }
    }


}


