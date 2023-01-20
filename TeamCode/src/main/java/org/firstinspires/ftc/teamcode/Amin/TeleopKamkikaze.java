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

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_brat_dc;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_top;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_deschis_dr;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_deschis_st;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_inchis_dr;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.poz_inchis_st;
import static org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate.NuSeMaiUmbla.LOW_POWER;
import static org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate.NuSeMaiUmbla.MEDIUM_POWER;
import static org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer.WHEEL_RADIUS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//disper sa moara mama
@TeleOp()
@Disabled
public class TeleopKamkikaze extends LinearOpMode {

    private double v1, v2, v3, v4;
    private boolean test = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

//        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        robot.setPoseEstimate(new Pose2d(-36, -68, Math.toRadians(-90)));

        while (!opModeIsActive() && !isStopRequested()) {
        }

        while (opModeIsActive() && !isStopRequested()) {
            // joysticks

            boolean rb = gamepad2.right_bumper;
            double rt = gamepad2.right_trigger;
            boolean lb = gamepad2.left_bumper;
            double lt = gamepad2.left_trigger;
            boolean a = gamepad2.a;
            boolean b = gamepad2.b;

            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.8,
                            -gamepad1.left_stick_x * 0.8,
                            -gamepad1.right_stick_x * 0.8
                    )
            );


            // miscari din dpad uri
            if (gamepad1.dpad_down) {
                robot.bagaViteza(-LOW_POWER, -LOW_POWER, -LOW_POWER, -LOW_POWER);
            }
            if (gamepad1.dpad_right) {
                robot.bagaViteza(MEDIUM_POWER, -MEDIUM_POWER, -MEDIUM_POWER, MEDIUM_POWER);
            }
            if (gamepad1.dpad_up) {
                robot.bagaViteza(LOW_POWER, LOW_POWER, LOW_POWER, LOW_POWER);
            }
            if (gamepad1.dpad_left) {
                robot.bagaViteza(-MEDIUM_POWER, MEDIUM_POWER, MEDIUM_POWER, -MEDIUM_POWER);
            }

            // rotiri fine din triggere
            if (gamepad1.right_trigger != 0) {
                robot.bagaViteza(LOW_POWER, -LOW_POWER, LOW_POWER, -LOW_POWER);
            }
            if (gamepad1.left_trigger != 0) {
                robot.bagaViteza(-LOW_POWER, LOW_POWER, -LOW_POWER, LOW_POWER);
            }

//            while (gamepad2.right_bumper) {
//                robot.se_ridica_brat(power_brat_dc);
//                if (gamepad2.left_trigger != 0) {
//                    robot.rotesteThing(power_top);
//                }
//                if (gamepad2.right_trigger != 0) {
//                    robot.rotesteThing(-power_top);
//                }
//
//                robot.hoooma();
//                if (gamepad2.a) {
//                    robot.apuca(poz_deschis_st, poz_deschis_dr);
////                robot.gheara_stanga.setPosition(1);
//                }
//                if (gamepad2.b) {
////                robot.gheara_stanga.setPosition(-1);
//                    robot.apuca(poz_inchis_st, poz_inchis_dr);
//                }
//                test = true;
//            }

            if(rb){
//                robot.se_ridica_brat(power_brat_dc);
//                test = true;
//                robot.ridica_brat_special(1);
                robot.unstop_brat_special(1);
            }else
//            while (gamepad2.left_bumper) {
//                robot.se_ridica_brat(-power_brat_dc);
//                test = false;
//            }

            if(lb){
                robot.unstop_brat_special(-1);
//                robot.se_ridica_brat(-power_brat_dc);
//                test = false;
            } else{
                robot.ridica_brat_special(-1);
//                robot.hoooma();
            }

//                robot.se_ridica_brat(power_reven);
//            }

//            robot.se_ridica_brat(0);

            if (gamepad2.left_trigger != 0) {
                robot.rotesteThing(power_top);
            }
            if (gamepad2.right_trigger != 0) {
                robot.rotesteThing(-power_top);
            }
//            robot.hoooma();

//            robot.rotesteThing(0);

//            if (gamepad2.x) {
////                robot.apuca(poz_deschis_st, poz_deschis_dr);
//                robot.gheara_dreapta.setPosition(1);
//            }
//            if (gamepad2.y) {
//                robot.gheara_dreapta.setPosition(-1);
////                robot.apuca(poz_inchis_st, poz_inchis_dr);
//            }

            if (gamepad2.a) {
                robot.apuca(poz_deschis_st, poz_deschis_dr);
//                robot.gheara_stanga.setPosition(1);
            }
            if (gamepad2.b) {
//                robot.gheara_stanga.setPosition(-1);
                robot.apuca(poz_inchis_st, poz_inchis_dr);
            }

        }

        robot.update();
        Pose2d poseEstimate = robot.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

    }
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

}

// vr sa mor
// simt cum disper