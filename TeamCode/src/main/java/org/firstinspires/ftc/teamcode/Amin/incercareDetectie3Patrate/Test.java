package org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
@Config
public class Test extends LinearOpMode {
    DcMotor rf, lf, rr, lr;
    public double power = 0.2;
    @Override
    public void runOpMode() throws InterruptedException {

        lf= hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                rf.setPower(power);
            }else {
                rf.setPower(0);
            }
            if(gamepad1.dpad_down){
                lf.setPower(power);
            }else {
                lf.setPower(0);
            }
            if(gamepad1.dpad_right){
                rr.setPower(power);
            }else {
                rr.setPower(0);
            }if(gamepad1.dpad_left){
                lr.setPower(power);
            }else {
                lr.setPower(0);
            }


        }

    }
}
