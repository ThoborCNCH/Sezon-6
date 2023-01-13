package org.firstinspires.ftc.teamcode.Amin;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class NU_MAI_POT {
    public static double LF_DIRECTION = 1;
    public static double RF_DIRECTION = 0;
    public static double LR_DIRECTION = 1;
    public static double RR_DIRECTION = 0;

    public static double power_top = 0.3;
    public static double power_brat_dc = 1;
    public static double power_brat_cr = 1;

    public  static double power_reven = 0.1;
    public  static double poz_deschis_st = 0.8;
    public  static double poz_deschis_dr = 0.5;
    public  static double poz_inchis_st =0.66;
    public  static double poz_inchis_dr = 0.6;

    public static boolean GLISIERE_ENCODER = true;
    public static double DISTANTA_GLISIERA = 74; //CM
    public static int GLISIERA_COUNTS = 0;

    public static Pose2d START_ST_RED_BLUE = new Pose2d(-36, -68, Math.toRadians(-90));
    public static Pose2d POSITION_ST_RED_BLUE = new Pose2d(-36, -38, Math.toRadians(30));
    public static Pose2d BACK_A_LITTLE_ST_RED_BLUE = new Pose2d(-36, -40, Math.toRadians(0));

    public static Pose2d START_DR_RED_BLUE = new Pose2d(-36, -68, Math.toRadians(-90));
    public static Pose2d POSITION_DR_RED_BLUE = new Pose2d(-36, -38, Math.toRadians(140));
    public static Pose2d BACK_A_LITTLE_DR_RED_BLUE = new Pose2d(-36, -40, Math.toRadians(180));


}
