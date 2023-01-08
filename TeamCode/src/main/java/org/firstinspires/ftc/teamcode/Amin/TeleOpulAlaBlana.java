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

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_reven;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_top;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_deschis_dr;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_deschis_st;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_inchis_dr;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_inchis_st;
import static org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate.NuSeMaiUmbla.LOW_POWER;
import static org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate.NuSeMaiUmbla.MEDIUM_POWER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//disper sa moara mama
@TeleOp()
public class TeleOpulAlaBlana extends LinearOpMode {

    private double v1, v2, v3, v4;
    private boolean test = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setPoseEstimate(new Pose2d(-36, -68, Math.toRadians(-90)));


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

            while (gamepad1.right_bumper) {
                robot.se_ridica_brat(1);
                test = true;
            }

            while (gamepad1.left_bumper) {
                robot.se_ridica_brat(-1);
                test = false;
            }

            if (test) {
//                robot.se_ridica_brat(power_reven);

            }

            robot.se_ridica_brat(0);

            while (gamepad2.left_trigger != 0) {
                robot.rotesteThing(power_top);
            }
            while (gamepad2.right_trigger != 0) {
                robot.rotesteThing(-power_top);
            }
            robot.hoooma();

//            robot.rotesteThing(0);

            if (gamepad2.a) {
                robot.apuca(poz_deschis_st, poz_deschis_dr);
            }
            if (gamepad2.b) {
                robot.apuca(poz_inchis_st, poz_inchis_dr);
            }

        }

        robot.update();


    }


}

// vr sa mor
// simt cum disper