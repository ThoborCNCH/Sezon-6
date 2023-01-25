package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_coborare;
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

@TeleOp
public class TeleopLejereala extends LinearOpMode {

    boolean test = false;
    SampleMecanumDrive robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);

        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.setPoseEstimate(new Pose2d(-36, -68, Math.toRadians(-90)));


        while (!opModeIsActive() && !isStopRequested()) {
        }

        while (opModeIsActive() && !isStopRequested()) {
//          joy-stick
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
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

//            if (gamepad2.right_trigger == 0 && robot.brat.getCurrentPosition() > 0.3) {
//                robot.brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.brat.setTargetPosition(robot.brat.getCurrentPosition());
//                robot.brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.brat.setPower(1);
//                while(robot.isBusy()){}
//            }

            while (gamepad2.right_bumper) {
//                robot.brat.setPower(1);
//                test = true;
//                robot.ridica_brat_special(1);
                robot.se_ridica_brat(1);
                test = true;
                if (gamepad2.a) {
                    robot.apuca(poz_deschis_st, poz_deschis_dr);
                } else if (gamepad2.b) {
                    robot.apuca(poz_inchis_st, poz_inchis_dr);
                }


                robot.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
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
                while (gamepad2.left_trigger != 0) {
                    robot.rotesteThing(power_top);
                }
                while (gamepad2.right_trigger != 0) {
                    robot.rotesteThing(-power_top);
                }
                robot.hoooma();
            }
            while (gamepad2.left_bumper) {
                robot.se_ridica_brat(power_coborare);

                if (gamepad2.a) {
                    robot.apuca(poz_deschis_st, poz_deschis_dr);
                } else if (gamepad2.b) {
                    robot.apuca(poz_inchis_st, poz_inchis_dr);
                }


                robot.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
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
                while (gamepad2.left_trigger != 0) {
                    robot.rotesteThing(power_top);
                }
                while (gamepad2.right_trigger != 0) {
                    robot.rotesteThing(-power_top);
                }
                robot.hoooma();
            }
            robot.se_ridica_brat(0);

            if (gamepad2.a) {
                robot.apuca(poz_deschis_st, poz_deschis_dr);
            } else if (gamepad2.b) {
                robot.apuca(poz_inchis_st, poz_inchis_dr);
            }
            while (gamepad2.left_trigger != 0) {
                robot.rotesteThing(power_top);
            }
            while (gamepad2.right_trigger != 0) {
                robot.rotesteThing(-power_top);
            }
            robot.hoooma();
        }
        if (gamepad2.right_bumper) {
            robot.se_ridica_brat(power_reven);
        }
    }
}
