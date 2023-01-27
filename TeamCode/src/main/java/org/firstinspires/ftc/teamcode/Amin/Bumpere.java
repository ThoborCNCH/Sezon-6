package org.firstinspires.ftc.teamcode.Amin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class Bumpere extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            while (gamepad2.left_bumper || gamepad2.right_bumper) {
                if (gamepad2.right_bumper)
                    robot.se_ridica_brat(1);
                else
                    robot.se_ridica_brat(NU_MAI_POT.power_coborare);
            }
            robot.se_ridica_brat(0);
        }
    }
}
