// Holds all of the global variables and setup methods

package org.firstinspires.ftc.teamcode.ciordeles.libs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Globals {
    private static BNO055IMU imu;


    public static void setupIMU(HardwareMap hardwareMap) {
        if (imu == null) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            imu.initialize(params);
        }
    }

    public static BNO055IMU getImu() { return imu; }

}
