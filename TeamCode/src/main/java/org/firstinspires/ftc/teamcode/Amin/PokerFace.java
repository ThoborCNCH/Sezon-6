package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.START_DR_RED_BLUE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class PokerFace extends LinearOpMode {
    SampleMecanumDrive osama;
    SampleMecanumDrive scula;
    private DistanceSensor al_6_lea_simt;


    @Override
    public void runOpMode() throws InterruptedException {
        scula = new SampleMecanumDrive(hardwareMap);
        osama = new SampleMecanumDrive(hardwareMap);
//        al_6_lea_simt = hardwareMap.get(DistanceSensor.class, "la_gheara");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        osama.setPoseEstimate(START_DR_RED_BLUE);
        waitForStart();
        double power = 0.6;

//        scula.senzor_auto(30, 0.6);
//        telemetry.addData("", (int) al_6_lea_simt.getDistance(DistanceUnit.CM));
//        telemetry.update();


        while (opModeIsActive()) {
//            osama.se_ridica_brat(1);
//            osama.top.setPower(1);
//            if (osama.top.getPower() > 0)
//                osama.top.setPower(-1);
//            else
//                osama.top.setPower(1);

//            telemetry.addLine(String.valueOf(osama.getDistanceSensorJos()));
//            telemetry.addLine(String.valueOf(osama.getDistanceSensorSus()));
            telemetry.addLine(String.valueOf(osama.getMagnetAtingere()));
//            telemetry.addLine(String.valueOf(osama.getbuci1()));
            telemetry.update();

//            osama.cerseste();
////            osama.ridica_dana(600, -0.5);
//
//
//
//            scula.senzor_auto(22, -0.6);
//
//            telemetry.addData("", (int)scula.getDistanceSensorSus());
//            telemetry.update();
////            if (al_6_lea_simt.getDistance(DistanceUnit.CM)>=30.0 )
////                osama.se_ridica_brat(0.6);
////            else
////                osama.se_ridica_brat(0);
//
//            Trajectory twin_towers = osama.trajectoryBuilder(START_DR_RED_BLUE)
//                    .addTemporalMarker(0, () -> {
////                        osama.se_ridica_brat(-power);
////                        osama.pana_la_dana(0.5);
////                        osama.ridica_brat_special(1000);
//                    })
////                    .addTemporalMarker(0.8, () -> {
////                        osama.se_ridica_brat(0);
////                    })
//                    .forward(20, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(30))
//                    .build();
//            osama.followTrajectory(twin_towers);
//
////            scula.ridicaLa(30, -0.6);
//            sleep(500);
//            osama.black();
////            sleep(1000);
//            osama.se_ridica_brat(power);
//
//            Trajectory allqaida = osama.trajectoryBuilder(twin_towers.end())
//                    .addTemporalMarker(time -> time * .4, () -> {
//                        osama.se_ridica_brat(0);
//                    })
//                    .back(20)
//                    .build();
//            osama.followTrajectory(allqaida);
//            stop();
        }
    }
}
