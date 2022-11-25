package org.firstinspires.ftc.teamcode.Amin;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class NU_MAI_POT {
    public static double LF_DIRECTION = 1;
    public static double RF_DIRECTION = 0;
    public static double LR_DIRECTION = 1;
    public static double RR_DIRECTION = 0;

    public static boolean GLISIERE_ENCODER = true;
    public static double DISTANTA_GLISIERA = 73;
    public static double GLISIERA_COUNTS = 1900;

    public static Pose2d START_ST_RED_BLUE = new Pose2d(-36, -68, Math.toRadians(-90));
    public static Pose2d POSITION_ST_RED_BLUE = new Pose2d(-36, -38, Math.toRadians(30));
    public static Pose2d BACK_A_LITTLE_ST_RED_BLUE = new Pose2d(-36, -40, Math.toRadians(0));

    public static Pose2d START_DR_RED_BLUE = new Pose2d(36, -68, Math.toRadians(-90));
    public static Pose2d POSITION_DR_RED_BLUE = new Pose2d(36, -38, Math.toRadians(140));
    public static Pose2d BACK_A_LITTLE_DR_RED_BLUE = new Pose2d(36, -40, Math.toRadians(180));


}