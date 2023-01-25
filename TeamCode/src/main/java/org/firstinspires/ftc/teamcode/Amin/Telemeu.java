package org.firstinspires.ftc.teamcode.Amin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class Telemeu extends LinearOpMode {

    SampleMecanumDrive robot;
    double mana;
    double brat, gheara;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {

//          MISCARE JOYSTICK
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if(gamepad2.left_bumper || gamepad2.right_bumper)
            {

                if(gamepad2.left_bumper)
                    brat = -1;
                else
                    brat = 1;
                robot.se_ridica_brat(brat);

                if(gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0)
                {
                    if(gamepad2.left_trigger != 0)
                        mana = -gamepad2.left_trigger;
                    else if(gamepad2.right_trigger != 0)
                        mana = gamepad2.right_trigger;
                    else
                        mana = 0;

                    robot.rotesteThing(mana);
                } else
                    robot.rotesteThing(0);

                if(gamepad2.a)
                    robot.apuca(NU_MAI_POT.poz_deschis_st, NU_MAI_POT.poz_deschis_dr);
                if(gamepad2.b)
                    robot.apuca(NU_MAI_POT.poz_inchis_st, NU_MAI_POT.poz_inchis_dr);

            } else {
                robot.se_ridica_brat(0);

                if(gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0)
                {
                    if(gamepad2.left_trigger != 0)
                        mana = -gamepad2.left_trigger;
                    else if(gamepad2.right_trigger != 0)
                        mana = gamepad2.right_trigger;
                    else
                        mana = 0;

                    robot.rotesteThing(mana);
                } else
                    robot.rotesteThing(0);

                if(gamepad2.a)
                    robot.apuca(NU_MAI_POT.poz_deschis_st, NU_MAI_POT.poz_deschis_dr);
                if(gamepad2.b)
                    robot.apuca(NU_MAI_POT.poz_inchis_st, NU_MAI_POT.poz_inchis_dr);

            }

        }


    }
}
