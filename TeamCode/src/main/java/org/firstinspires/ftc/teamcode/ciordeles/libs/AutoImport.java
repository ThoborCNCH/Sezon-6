// The parent of all programs
// Handles all universal init processes and holds most functions used in auto programs

/*
TODO:
    - reconfigure webcam
    - fix imu
    - fix wall driving
 */

package org.firstinspires.ftc.teamcode.ciordeles.libs;

import static org.firstinspires.ftc.teamcode.ciordeles.libs.Globals.*;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

public class AutoImport extends LinearOpMode implements TeleAuto {
    // Defines vars
    protected DcMotor fr = null;
    protected DcMotor rr = null;
    protected DcMotor rl = null;
    protected DcMotor fl = null;
    protected DcMotor glisiera1 = null;
    protected DcMotor glisiera2 = null;
    protected CRServo marus = null;

    protected DcMotor spinner = null;

    protected ColorRangeSensor colorRange = null;
    protected CRServo intakeLight = null;

    protected SlamraDrive slauto = new SlamraDrive();

    protected FtcDashboard dashboard = FtcDashboard.getInstance();
    protected TelemetryPacket packet = new TelemetryPacket();

    protected ElapsedTime timer = new ElapsedTime();

    // vars used in program
    protected int elementPosition;
    protected int startingPoseX;
    protected int startingPoseY;
    protected int camera1X;
    protected int camera1Y;
    protected int camera2X;
    protected int camera2Y;
    protected int[] armYPositions = {-46, -69, -86, -106};
    protected int[] armYEnc = { 0, -1800, -2864, -3992};
    protected int[] armXEnc = {1279, 0, -1409}; // in order of left, mid, right

    /*protected AtomicBoolean isAsyncing = new AtomicBoolean(false);
    protected AtomicInteger targetDegree = new AtomicInteger();
    protected CompletableFuture driveUsingIMUReturn;

    protected AtomicBoolean isAsyncing2 = new AtomicBoolean(false);
    protected AtomicInteger targetDegree2 = new AtomicInteger();
    protected CompletableFuture driveUsingIMUReturn2;*/

    public AutoImport(int startX, int startY, int cam1X, int cam1Y, int cam2X, int cam2Y) {
        startingPoseX = startX;
        startingPoseY = startY;
        camera1X = cam1X;
        camera1Y = cam1Y;
        camera2X = cam2X;
        camera2Y = cam2Y;
    }

    public boolean driverAbort() {
        return false;
    }

    public void runOpMode() {
        // configures hardware
        System.out.println("::::configuring hardware");
        fl = hardwareMap.get(DcMotor.class, "lf");
        fr = hardwareMap.get(DcMotor.class, "rf");
        rl = hardwareMap.get(DcMotor.class, "lr");
        rr = hardwareMap.get(DcMotor.class, "rr");
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);

        glisiera1 = hardwareMap.get(DcMotor.class, "glisiera1");
        glisiera2 = hardwareMap.get(DcMotor.class, "glisiera2");
        marus = hardwareMap.get(CRServo.class, "marus");

        // initializes imu
        System.out.println("::::configuring imu");
        setupIMU(hardwareMap);
        telemetry.addLine("IMU Done");
        telemetry.update();

        // passes hardware to slamra class
        System.out.println("::::configuring slamra");
        DcMotor[] motors = {fr, rr, rl, fl};
        slauto.setUp(motors, telemetry);

        telemetry.addLine("Cameras Done");
        telemetry.update();

        // adds start telemetry
        telemetry.addLine("hardware configured");
        telemetry.update();
        packet.addLine("hardware configured");
        dashboard.sendTelemetryPacket(packet);

        // sets servos to starting positions


        while (!isStarted()) {
            // loops this until start is pressed
            packet.put("Position", elementPosition);
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("Position", elementPosition);
            telemetry.update();
        }
        packet.addLine("program started");
        dashboard.sendTelemetryPacket(packet);


        timer.reset();
    }

    // Function which runs the intake for a certain period of time. -1 = intake, 1 = outtake
    public void runIntake(double power, int timeout) {
        marus.setPower(power);
        sleep(timeout);
        marus.setPower(0);
    }
    public void runGlisiera(double power, int timeout) {
        glisiera1.setPower(power);
        glisiera2.setPower(power);
        sleep(timeout);
        glisiera1.setPower(0);
        glisiera2.setPower(0);
    }


    public void driveUntilFull(double power) {
        ElapsedTime et = new ElapsedTime();
        while (colorRange.getLightDetected() <= 0.11 && et.milliseconds() < 2000) {
            fl.setPower(power);
            fr.setPower(power);
            rl.setPower(power);
            rr.setPower(power);
        }
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }

    public void shimmy(double power, int amount, int delay) {
        for (int i = 0; i < amount; i++) {
            fl.setPower(power);
            fr.setPower(power);
            rl.setPower(power);
            rr.setPower(power);
            sleep(delay);
            fl.setPower(-power);
            fr.setPower(-power);
            rl.setPower(-power);
            rr.setPower(-power);
            sleep(delay);
        }
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }


    // Function which sets encoder values to 0, and waits until they have reset
    public void resetEnc(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (motor.getCurrentPosition() != 0) {
            sleep(10);
        }
    }

    public double wrap(double theta) {
        double newTheta = theta;
        while(abs(newTheta) > 180) {
            if (newTheta < -180) {
                newTheta += 360;
            } else {
                newTheta -= 360;
            }
        }
        return newTheta;
    }
}

