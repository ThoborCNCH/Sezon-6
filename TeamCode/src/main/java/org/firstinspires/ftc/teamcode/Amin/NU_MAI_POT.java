package org.firstinspires.ftc.teamcode.Amin;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class NU_MAI_POT {
    public static double LF_DIRECTION = 1;
    public static double RF_DIRECTION = 0;
    public static double LR_DIRECTION = 1;
    public static double RR_DIRECTION = 0;

    public static double fr = 1;
    public static double lr = 1;
    public static double rr = 0;


    public static double power_top = 0.3;
    public static double power_brat_dc = 1;
    public static double power_brat_cr = 1;

    public static double power_reven = 0.3;
    public static double poz_deschis_st = 0.3;
    public static double poz_deschis_dr = 0.6;
    public static double poz_inchis_st = 0.2;
    public static double poz_inchis_dr = 0.9;

    public static boolean GLISIERE_ENCODER = true;
    public static double DISTANTA_GLISIERA = 74; //CM
    public static int GLISIERA_COUNTS = 0;

    public static Pose2d START_ST_RED_BLUE = new Pose2d(-36, -68, Math.toRadians(-90));
    public static Pose2d POSITION_ST_RED_BLUE = new Pose2d(-36, -38, Math.toRadians(30));
    public static Pose2d BACK_A_LITTLE_ST_RED_BLUE = new Pose2d(-36, -40, Math.toRadians(0));

    public static Pose2d START_DR_RED_BLUE = new Pose2d(41.5, -58, Math.toRadians(90));
    public static Pose2d INTRE_TOT_DR_RED_BLUE = new Pose2d(43, -10, Math.toRadians(90));
    public static Vector2d INTRE_TOT_DR_RED_BLUE_VECTOR = new Vector2d(43, -10);
    public static Pose2d PRE_POSITION_DR_RED_BLUE = new Pose2d(43, -25, Math.toRadians(90));
    public static Pose2d JUNCTION_PUNE_DR_RED_BLUE = new Pose2d(38.7, -1.5, Math.toRadians(130));
    public static Vector2d JUNCTION_PUNE_DR_RED_BLUE_VECTOR = new Vector2d(32, -3);
    public static Pose2d STACK_DR_RED_BLUE = new Pose2d(68, -8.5, Math.toRadians(0));
    public static Vector2d STACK_DR_RED_BLUE_VECTOR = new Vector2d(68, -8.5);
    public static Pose2d BACK_A_LITTLE_DR_RED_BLUE = new Pose2d(-36, -40, Math.toRadians(180));


}
