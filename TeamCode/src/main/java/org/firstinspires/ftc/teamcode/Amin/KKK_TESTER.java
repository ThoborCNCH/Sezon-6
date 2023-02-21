package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.PRE_POSITION_DR_RED_BLUE_KKK;
import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.START_DR_RED_BLUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bratHandler;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@Autonomous
public class KKK_TESTER extends LinearOpMode {
    SampleMecanumDrive robot;
    TestThread threadKamikaze;

    ExecutorService threadpool;

//    Runnable runnableBratHandler;
//    Thread bratThread;

    bratHandler br;

    @Override
    public void runOpMode() throws InterruptedException {

        threadpool = Executors.newCachedThreadPool();

        robot = new SampleMecanumDrive(hardwareMap);
        robot.setPoseEstimate(START_DR_RED_BLUE);

//        runnableBratHandler = new bratHandler(robot.brat, robot.brat_pe_sub, telemetry);
//        bratThread = new Thread(runnableBratHandler);

        br = new bratHandler(robot.brat, robot.brat_pe_sub, telemetry);

//        threadKamikaze = new TestThread(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            TrajectorySequence go_pune = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(41.5, -40, Math.toRadians(0)))
                    .lineToLinearHeading(PRE_POSITION_DR_RED_BLUE_KKK,
                            SampleMecanumDrive.getVelocityConstraint(40,
                                    DriveConstants.MAX_ANG_VEL,
                                    DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .build();
            robot.followTrajectorySequence(go_pune);

//            threadKamikaze.caca_nu_e_voie();
//           br.run();
            Future<Void> futureTask = (Future<Void>) threadpool.submit(() -> se_ridica_brat(0.4));

            ///////////////////POATE CE ESTE MAI JOS -------------------------
//            new Thread(new Runnable() {
//                public void run(){
//                    //codul
//                }
//            }).start();
            ////////////////////POATE CE ESTE MAI SUS ------------------------

            TrajectorySequence reven = robot.trajectorySequenceBuilder(go_pune.end())
                    .strafeRight(2)
                    .back(50)
                    .build();

            robot.followTrajectorySequence(reven);

            threadpool.shutdown();

            stop();
        }
    }

    private void se_ridica_brat(double putere) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (timer.seconds()<=3)
        {
            robot.brat.setPower(putere);
            robot.brat_pe_sub.setPower(-putere);
        }
        robot.brat.setPower(0);
        robot.brat_pe_sub.setPower(0);

    }

    /* ce urmeaza este o solutie din chatGPT, nu rezist psihic

        ----------SEE CHATGPT--------------

         NR 1.
         TrajectorySequence go_pune = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
            .lineToLinearHeading(new Pose2d(41.5, -40, Math.toRadians(0)))
            .addDisplacementMarker(() -> {
                // This code will run when the robot reaches the end of the first trajectory
                Thread t = new Thread(() -> {
                    se_ridica_brat(1.0); // Or any other code you want to run
                });
                t.start();
            })
            .lineToLinearHeading(PRE_POSITION_DR_RED_BLUE_KKK,
                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(30))
            .build();
        robot.followTrajectorySequence(go_pune);

        NR 2.
        ExecutorService executorService = Executors.newSingleThreadExecutor();
        TrajectorySequence go_pune = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
            .lineToLinearHeading(new Pose2d(41.5, -40, Math.toRadians(0)))
            .addDisplacementMarker(() -> {
            // This code will run when the robot reaches the end of the first trajectory
            executorService.submit(() -> {
                se_ridica_brat(1.0); // Or any other code you want to run
            });
            })
            .lineToLinearHeading(PRE_POSITION_DR_RED_BLUE_KKK,
                SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(30))
            .build();
        robot.followTrajectorySequence(go_pune);

        ----------------poate si un pool.shutdown dupa ------------

     */
}
