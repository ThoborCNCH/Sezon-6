// The program used in teleop

package org.firstinspires.ftc.teamcode.ciordeles.opmodes;

import static org.firstinspires.ftc.teamcode.ciordeles.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ciordeles.libs.AutoImport;
import org.firstinspires.ftc.teamcode.ciordeles.libs.FieldCentric;
import org.firstinspires.ftc.teamcode.ciordeles.libs.Globals;

@TeleOp(name="MainTele", group="teleop")
public class MainTele extends AutoImport {

    public MainTele() { super(31, -56, 225, 150, 255, 150); }
    FieldCentric drive = new FieldCentric();
    public boolean driverAbort() {
        return gamepad1.y;
    }

    @Override
    public void runOpMode() {
        super.runOpMode();

        int loops = 0;

        // adds start telemetry
        telemetry.addLine("hardware ready");
        telemetry.update();

        // Sets up motor configs
        drive.setUp(new DcMotor[] {fl, rl, fr, rr});

        // Configures prev1 & prev2
        Gamepad prev1 = new Gamepad();
        Gamepad prev2 = new Gamepad();
        Gamepad cur1 = new Gamepad();
        Gamepad cur2 = new Gamepad();

        // Set up variables
        boolean speedy = true;
        double robotAngle = getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        boolean imuExited = false;

        // Starting servo & motor positions

        waitForStart();
    
        while (opModeIsActive()) {
            // Updates cur1 & 2
            try {
                cur1.copy(gamepad1);
                cur2.copy(gamepad2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }

            // Gets IMUs
            robotAngle = getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // Gives FieldCentric the stick positions based off of speed setting
            if (cur1.right_bumper && !prev1.right_bumper ) {
                if (!speedy) {
                    speedy = true;
                    telemetry.addLine("Speedy: On");
                } else if (speedy){
                    speedy = false;
                    telemetry.addLine("Speedy: Off");
                }
                telemetry.update();
            }
            if (!speedy) {
                drive.Drive(
                        Range.clip(gamepad1.left_stick_x, -0.55, 0.55),
                        Range.clip(-gamepad1.left_stick_y, -0.5, 0.5),
                        Range.clip(gamepad1.right_stick_x, -0.25, 0.25));

            } else {
                drive.Drive(
                        Range.clip(gamepad1.left_stick_x, -0.95, 0.95),
                        -gamepad1.left_stick_y,
                        Range.clip(gamepad1.right_stick_x, -0.75, 0.75));

            }

            if(cur2.dpad_up)
                runGlisiera(1,1000);
            else
                if(cur2.dpad_down)
                    runGlisiera(-1,1000);
            if(cur2.dpad_right)
                runIntake(1,1000);
            else
                if(cur2.dpad_left)
                    runIntake(-1,1000);

            // Controls arm
            // Reset Field Centric button
            if (cur1.a && !prev1.a) {
                drive.newOffset();
            }

            // Updates prev1 & 2
            try {
                prev1.copy(cur1);
                prev2.copy(cur2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }
            loops++;
        }
    }
}
