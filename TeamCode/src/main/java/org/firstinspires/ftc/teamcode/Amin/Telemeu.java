package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_coborare;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_de_cosmin;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_thing_slow;
import static org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate.NuSeMaiUmbla.LOW_POWER;
import static org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate.NuSeMaiUmbla.MEDIUM_POWER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//AUIMA AUE AUIMA AUE IN THE JUNGLE THE MIGHTY JUNGLE THE LION SLEEP TOONIGHTTTTT



@TeleOp
public class Telemeu extends LinearOpMode {
    SampleMecanumDrive robot;
    double mana;
    double brat;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

//          MISCARE JOYSTICK
            if (!(gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down))
            {
//                double LF = 0;
//                double RF = 0;
//                double LR = 0;
//                double RR = 0;
//
//                // Get joystick values
//                double Y1 = gamepad1.left_stick_y; // invert so up is positive
//                double X1 = -gamepad1.left_stick_x;
//                double X2 = gamepad1.right_stick_x;
//
//                // Forward/back movement
//                LF -= Y1;
//                RF -= Y1;
//                LR -= Y1;
//                RR -= Y1;
//
//                // Side to side movement
//                LF -= X1;
//                RF += X1;
//                LR += X1;
//                RR -= X1;
//
//                // Rotation movement
//                LF -= X2;
//                RF += X2;
//                LR -= X2;
//                RR += X2;
//
//                double motorMax = 1;
//
//                // Clip motor power values to +-motorMax
//                LF = Math.max(-motorMax, Math.min(LF, motorMax));
//                RF = Math.max(-motorMax, Math.min(RF, motorMax));
//                LR = Math.max(-motorMax, Math.min(LR, motorMax));
//                RR = Math.max(-motorMax, Math.min(RR, motorMax));
//
//                // Send values to the motors
//                robot.bagaViteza(LF, RF, LR, RR);

                robot.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * 0.6,
                                -gamepad1.left_stick_x * 0.6,
                                -gamepad1.right_stick_x * 0.6//X ul trebuie
                        )
                );
            }
            // miscari din dpad uri
            if (gamepad1.dpad_up) {
                robot.bagaViteza(LOW_POWER, LOW_POWER, LOW_POWER, LOW_POWER);
//                robot.setWeightedDrivePower(
//                        new Pose2d(
//                                0.3,
//                                0,
//                                0
//                        )
//                );
            }
             else if (gamepad1.dpad_down) {
                robot.bagaViteza(-LOW_POWER, -LOW_POWER, -LOW_POWER, -LOW_POWER);
//                robot.setWeightedDrivePower(
//                        new Pose2d(
//                                -0.3,
//                                0,
//                                0
//                        )
//                );
            }
            else if (gamepad1.dpad_left) {
                robot.bagaViteza(-MEDIUM_POWER, MEDIUM_POWER, MEDIUM_POWER, -MEDIUM_POWER);
//                robot.setWeightedDrivePower(
//                        new Pose2d(
//                                0,
//                                -0.3,
//                                0
//                        )
//                );
            }
            else if (gamepad1.dpad_right) {
                robot.bagaViteza(MEDIUM_POWER, -MEDIUM_POWER, -MEDIUM_POWER, MEDIUM_POWER);
//                robot.setWeightedDrivePower(
//                        new Pose2d(
//                                0,
//                                0.3,
//                                0
//                        )
//                );
            } else if(!(gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0)){
                robot.bagaViteza(0, 0,0, 0);
            }
//            while(gamepad1.dpad_right) {
//                robot.bagaViteza(MEDIUM_POWER, -MEDIUM_POWER, -MEDIUM_POWER, MEDIUM_POWER);
//            }
//            while(gamepad1.dpad_up) {
//                robot.bagaViteza(LOW_POWER, LOW_POWER, LOW_POWER, LOW_POWER);
//            }
//            while(gamepad1.dpad_left) {
//                robot.bagaViteza(-MEDIUM_POWER, MEDIUM_POWER, MEDIUM_POWER, -MEDIUM_POWER);
//            }

            // rotiri fine din triggere
            while (gamepad1.right_trigger != 0) {
                robot.bagaViteza(LOW_POWER, -LOW_POWER, LOW_POWER, -LOW_POWER);
            }
            while (gamepad1.left_trigger != 0) {
                robot.bagaViteza(-LOW_POWER, LOW_POWER, -LOW_POWER, LOW_POWER);
            }

//            if (gamepad2.dpad_up || gamepad2.dpad_down) {
//                if (gamepad2.dpad_up)
//                    brat = -power_de_cosmin;
//                else
//                    brat = power_de_cosmin;
////                robot.se_ridica_brat(brat);
//            } else {
//                brat = 0;
////                robot.se_ridica_brat(brat);
//            }
//            robot.se_ridica_brat(brat);

//            ASTA NOUA
            if (gamepad2.dpad_right || gamepad2.dpad_left) {
                if (gamepad2.dpad_left)
                    mana = power_thing_slow;
                else if (gamepad2.dpad_right)
                    mana = -power_thing_slow;
                else
                    mana = 0;
                robot.rotesteThing(mana);
            } else
                robot.rotesteThing(0);

            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                if (gamepad2.left_bumper)
                    brat = -power_de_cosmin;
                else
                    brat = power_de_cosmin;
//                robot.se_ridica_brat(brat);

                if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0) {
                    if (gamepad2.left_trigger != 0)
                        mana = 1;
                    else if (gamepad2.right_trigger != 0)
                        mana = -gamepad2.right_trigger;
                    else
                        mana = 0;

                    robot.rotesteThing(mana);
                } else
                    robot.rotesteThing(0);

                if (gamepad2.a)
                    robot.apuca(NU_MAI_POT.poz_deschis_st, NU_MAI_POT.poz_deschis_dr);
                else if (gamepad2.b)
                    robot.apuca(NU_MAI_POT.poz_inchis_st, NU_MAI_POT.poz_inchis_dr);

            } else {
//                robot.se_ridica_brat(0);
                brat = 0;

                if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0) {
                    if (gamepad2.left_trigger != 0)
                        mana = 1;
                    else if (gamepad2.right_trigger != 0)
                        mana = -1;
                    else
                        mana = 0;

                    robot.rotesteThing(mana);
                } else
                    robot.rotesteThing(0);

                if (gamepad2.a)
                    robot.apuca(NU_MAI_POT.poz_deschis_st, NU_MAI_POT.poz_deschis_dr);
                if (gamepad2.b)
                    robot.apuca(NU_MAI_POT.poz_inchis_st, NU_MAI_POT.poz_inchis_dr);
            }
            robot.se_ridica_brat(brat);
        }
    }
}
