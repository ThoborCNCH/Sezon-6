////package org.firstinspires.ftc.teamcode.Amin;
////
////import com.acmerobotics.roadrunner.geometry.Pose2d;
////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import com.qualcomm.robotcore.hardware.DcMotor;
////
////import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
////
////public class TeleopTeRog extends LinearOpMode {
////    SampleMecanumDrive robot;
////
////    @Override
////    public void runOpMode() throws InterruptedException {
////        robot = new SampleMecanumDrive(hardwareMap);
////
////        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//////        robot.setPoseEstimate(new Pose2d(-36, -68, Math.toRadians(-90)));
////
////
////        while (!opModeIsActive() && !isStopRequested()) {
////        }
////
////        while (opModeIsActive() && !isStopRequested()) {
//////          joy-stick
////            robot.setWeightedDrivePower(
////                    new Pose2d(
////                            -gamepad1.left_stick_y,
////                            -gamepad1.left_stick_x,
////                            -gamepad1.right_stick_x
////                    )
////            );
////        }
////    }
////}
//
//package org.firstinspires.ftc.teamcode.Amin;
//
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//
///**
// * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
// * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
// * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all linear OpModes contain.
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//
//@TeleOp(name="MECHANUM_Absortie", group="Test")
////@Disabled
//public class TeleopTeRog extends LinearOpMode {
//
////    private ElapsedTime runtime = new ElapsedTime();
////    DcMotor leftFrontMotor = null;
////    DcMotor rightFrontMotor = null;
////    DcMotor leftRearMotor = null;
////    DcMotor rightRearMotor = null;
////    double pozitie = 0;
////    public Servo fdr, fst;
////    DcMotor absortie1;
////    DcMotor absortie2;
////    DcMotor ridicare2;
////    Servo gheara;
////    Servo capstone;
////    DcMotor ridicare;
////    CRServo extindere = null;
////    public DigitalChannel buton;
////
////
////    //====================[servos]===================
////
////    //public Servo lateral = null;
////    // declare motor speed variables
////    double RF; double LF; double RR; double LR;
////    // declare joystick position variables
////    double X1; double Y1; double X2; double Y2;
////    // operational constants
////    double joyScale = 0.9;
////    double motorMax = 1; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
////
////    public double pow_axe = 0.3;
////    public double pow_rotire = 0.4;
////    public double full_power = 0;
////
////    public double ghearaStransa = 0;
////    double buttonReleased = 1;
////
////    public double lateralStransa = 0;
////    double LateralReleased = 1;
////
////    double placaReleased = 1;
////    public double placaStransa = 0;
////    double buton_absortie=0;
////    double buton_respingere=0;
////    double capstoneReleased = 1;
////    double capstoneStransa = 0;
////
////    @Override
////    public void runOpMode() {
////        telemetry.addData("Status", "Initialized");
////        telemetry.update();
////
////
////
////        //======================[motors]============================================================
////
////
////        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
////        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
////        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
////        rightRearMotor = hardwareMap.dcMotor.get("rightRear");
////        gheara = hardwareMap.get(Servo.class, "gheara");
////        fst = hardwareMap.get(Servo.class,"fst");
////        fdr = hardwareMap.get(Servo.class,"fdr");
////        ridicare = hardwareMap.dcMotor.get("ridicare");
////        ridicare2 = hardwareMap.dcMotor.get("ridicare2");
////        extindere   = hardwareMap.get(CRServo.class,"extindere");
////        absortie1 = hardwareMap.dcMotor.get("absortie1");
////        absortie2 = hardwareMap.dcMotor.get("absortie2");
////        buton = hardwareMap.get(DigitalChannel.class,"buton");
////        capstone = hardwareMap.get(Servo.class,"capstone");
////
////
////
////
////
////        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
////        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
////        rightRearMotor = hardwareMap.dcMotor.get("rightRear");
////        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
////        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
////        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
////        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
////        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
////        absortie2.setDirection(DcMotor.Direction.REVERSE);
////        ridicare2.setDirection(DcMotor.Direction.REVERSE);
////
////        buton.setMode(DigitalChannel.Mode.INPUT);
////        fst.setPosition(1);
////        fdr.setPosition(0);
////
////
////
////       /* leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */
////
////        // Wait for the game to start (driver presses PLAY)
////        waitForStart();
////        runtime.reset();
////
////
////        // run until the end of the match (driver presses STOP)
////        while (opModeIsActive()) {
////            telemetry.addData("Status", "Run Time: " + runtime.toString());
////
////            telemetry.update();
////
////            if(gamepad1.right_bumper || gamepad1.a)
////            {
////                pow_axe = 1;
////                joyScale = 1;
////                pow_rotire=1;
////                telemetry.addData("viteza pow_axe", pow_axe);
////            }
////            else
////            {
////                pow_axe = 0.4;
////                pow_rotire=0.4;
////            }
////            if(gamepad1.dpad_down)
////            {
////                leftFrontMotor.setPower(-pow_axe);
////                rightFrontMotor.setPower(-pow_axe);
////                leftRearMotor.setPower(-pow_axe);
////                rightRearMotor.setPower(-pow_axe);
////            } else if(gamepad1.dpad_right){
////                leftFrontMotor.setPower(+pow_axe);
////                rightFrontMotor.setPower(-pow_axe);
////                leftRearMotor.setPower(-pow_axe);
////                rightRearMotor.setPower(pow_axe);
////            } else if (gamepad1.dpad_up){
////                leftFrontMotor.setPower(pow_axe);
////                rightFrontMotor.setPower(pow_axe);
////                leftRearMotor.setPower(pow_axe);
////                rightRearMotor.setPower(pow_axe);
////            } else if (gamepad1.dpad_left){
////                leftFrontMotor.setPower(-pow_axe);
////                rightFrontMotor.setPower(pow_axe);
////                leftRearMotor.setPower(pow_axe);
////                rightRearMotor.setPower(-pow_axe);
////
////            } else if (gamepad1.left_trigger != 0){
////                leftFrontMotor.setPower(pow_rotire);
////                rightFrontMotor.setPower(-pow_rotire);
////                leftRearMotor.setPower(pow_rotire);
////                rightRearMotor.setPower(-pow_rotire);
////            } else if (gamepad1.right_trigger != 0){
////                leftFrontMotor.setPower(-pow_rotire);
////                rightFrontMotor.setPower(pow_rotire);
////                leftRearMotor.setPower(-pow_rotire);
////                rightRearMotor.setPower(pow_rotire);
////            } else {
////                leftFrontMotor.setPower(0);
////                rightFrontMotor.setPower(0);
////                leftRearMotor.setPower(0);
////                rightRearMotor.setPower(0);
////            }
////            // Reset speed variables
////            LF = 0;
////            RF = 0;
////            LR = 0;
////            RR = 0;
////
////            // Get joystick values
////            Y1 = gamepad1.left_stick_y * joyScale; // invert so up is positive
////            X1 = -gamepad1.left_stick_x * joyScale;
////            Y2 = -gamepad1.right_stick_y * joyScale; // Y2 is not used at present
////            X2 = gamepad1.right_stick_x * joyScale;
////
////            // Forward/back movement
////            LF -= Y1;
////            RF -= Y1;
////            LR -= Y1;
////            RR -= Y1;
////
////            // Side to side movement
////            LF -= X1;
////            RF += X1;
////            LR += X1;
////            RR -= X1;
////
////            // Rotation movement
////            LF -= X2;
////            RF += X2;
////            LR -= X2;
////            RR += X2;
////
////            // Clip motor power values to +-motorMax
////            LF = Math.max(-motorMax, Math.min(LF, motorMax));
////            RF = Math.max(-motorMax, Math.min(RF, motorMax));
////            LR = Math.max(-motorMax, Math.min(LR, motorMax));
////            RR = Math.max(-motorMax, Math.min(RR, motorMax));
////
////            // Send values to the motors
////            leftFrontMotor.setPower(LF);
////            rightFrontMotor.setPower(RF);
////            leftRearMotor.setPower(LR);
////            rightRearMotor.setPower(RR);
//
//
//
//            //----------[gheara]---------------------------------------------------------------------
//            if(gamepad2.x && buttonReleased == 1)
//            {
//                buttonReleased = 0;
//                if (ghearaStransa == 0) {
//                    ghearaStransa = 1;
//                    gheara.setPosition(1);
//                    telemetry.addData("Gheara", "s-a strans");
//                } else {
//                    ghearaStransa = 0;
//                    gheara.setPosition(0.15);
//                    telemetry.addData("Gheara", "s-a deschis");
//                }
//            }
//            if(!gamepad2.x) buttonReleased = 1;
//            //----------[capstone]---------------------------------------------------------------------
//            if(gamepad2.y && capstoneReleased == 1)
//            {
//                capstoneReleased = 0;
//                if (capstoneStransa == 0) {
//                    capstoneStransa = 1;
//                    capstone.setPosition(-0.80);
//                    telemetry.addData("Capstone", "s-a strans");
//                } else {
//                    capstoneStransa = 0;
//                    capstone.setPosition(0.40);
//                    telemetry.addData("Capstone", "s-a deschis");
//                }
//            }
//            if(!gamepad2.y) capstoneReleased = 1;
//
//            //======================[absortie]=========================
//
//
//
//            if(gamepad1.left_bumper)
//            {absortie1.setPower(0.8);
//                absortie2.setPower(0.8);}
//            else if(gamepad1.right_bumper)
//            {absortie1.setPower(-0.8);
//                absortie2.setPower(-0.8);
//
//            }
//            else
//            {absortie1.setPower(0);
//                absortie2.setPower(0);}
////--------------------------[extindere]-------------------------
//            if(gamepad2.right_trigger != 0)
//                extindere.setPower(gamepad2.right_trigger);
//            else if(gamepad2.left_trigger !=0)
//                extindere.setPower(-gamepad2.left_trigger);
//            else
//                extindere.setPower(0);
//
////--------------------------[ridicare]------------------------
//            if(gamepad2.left_stick_y >0)
//
//
//            {ridicare.setPower(gamepad2.left_stick_y);
//                ridicare2.setPower(-gamepad2.left_stick_y);  }
//
//            else if(gamepad2.left_stick_y <0)
//
//
//            {ridicare.setPower(gamepad2.left_stick_y*4/5);
//                ridicare2.setPower(-gamepad2.left_stick_y*4/5);  }
//            else
//            {ridicare.setPower(0);
//                ridicare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                ridicare2.setPower(0);
//                ridicare2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            }
//
//
//
//
//
//            //------------------[lateral]-----------------------------------------------------------
//            /*if(gamepad2.b && buttonReleased == 1)
//            {
//                buttonReleased = 0;
//                if (lateralStransa == 0) {
//                    lateralStransa = 1;
//                    lateral.setPower(0.6);
//                    telemetry.addData("Lateral", "a coborat");
//                } else {
//                    lateralStransa = 0;
//                    lateral.setPower(0.6);
//                    telemetry.addData("Lateral", "s-a ridicat");
//                }
//            }
//            if(!gamepad2.b) buttonReleased = 1;*/
//
//
//            //-------------------[placa]------------------------------------------------------------
//            if(gamepad1.b && placaReleased == 1)
//            {
//                placaReleased = 0;
//                if (placaStransa == 0) {
//                    placaStransa = 1;
//                    fst.setPosition(-.5);
//                    fdr.setPosition(1);
//                    telemetry.addData("Placa", "a coborat");
//                } else {
//                    placaStransa = 0;
//                    fst.setPosition(1);
//                    fdr.setPosition(0);
//                    telemetry.addData("Placa", "s-a ridicat");
//                }
//            }
//            if(!gamepad1.b) placaReleased = 1;
//            telemetry.update();
//            //================================================================================*/
//        }
//
//    }
//}