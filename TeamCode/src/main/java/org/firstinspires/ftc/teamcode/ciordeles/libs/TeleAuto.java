// Holds a global interface for things used elsewhere

package org.firstinspires.ftc.teamcode.ciordeles.libs;

public interface TeleAuto {
    boolean opModeIsActive();
    boolean driverAbort();
    void sleep(long milliseconds);
    void idle();
}
