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
    public static double power_brat_dc = .6;
    public static double power_brat_cr = 1;
    public static double power_coborare = -0.5;
    public static double power_brat_dc_cob = .7;
    public static double power_thing_slow = 0.3;

    public static double power_de_cosmin = 0.8;

    public static long sleep_for_strafe = 400;


    public static double power_reven = 0.2;
    public static double poz_deschis_st = 0.48; //0.18
    public static double poz_deschis_dr = 0.86; //0.65
    public static double poz_inchis_st = 0.35; //0
    public static double poz_inchis_dr = 1; //0.75

    public static double poz_deschis_st_AUTO = 0.7; //0.7
    public static double poz_deschis_dr_AUTO = 0.7; //0.4


    public static boolean GLISIERE_ENCODER = true;
    public static double DISTANTA_GLISIERA = 74; //CM
    public static int GLISIERA_COUNTS = 0;

    public static Pose2d START_DR_RED_BLUE = new Pose2d(41.5, -58, Math.toRadians(90));
    public static Pose2d INTRE_TOT_DR_RED_BLUE = new Pose2d(43, -10, Math.toRadians(90));
    public static Vector2d INTRE_TOT_DR_RED_BLUE_VECTOR = new Vector2d(43, -10);
    public static Pose2d PRE_POSITION_DR_RED_BLUE2 = new Pose2d(43, -10, Math.toRadians(90));

    public static Pose2d PRE_POSITION_DR_RED_BLUE = new Pose2d(43, -8, Math.toRadians(136.6));
    public static Pose2d PRE_POSITION_DR_RED_BLUE3 = new Pose2d(43, -8, Math.toRadians(136.6));

    public static Pose2d JUNCTION_PUNE_INAINTE_DR_RED_BLUE = new Pose2d(38.3, -4.5, Math.toRadians(90));
    public static Pose2d JUNCTION_PUNE_DR_RED_BLUE = new Pose2d(38.3, -1.5, Math.toRadians(130));
    public static Vector2d JUNCTION_PUNE_DR_RED_BLUE_VECTOR = new Vector2d(32, -3.5);
    public static Pose2d STACK_DR_RED_BLUE = new Pose2d(60, -8.5, Math.toRadians(0));
    public static Vector2d STACK_DR_RED_BLUE_VECTOR = new Vector2d(68, -8.5);
    public static Pose2d BACK_A_LITTLE_DR_RED_BLUE = new Pose2d(-36, -40, Math.toRadians(180));




    public static Pose2d START_ST_RED_BLUE = new Pose2d(-41.5, -58, Math.toRadians(90));
    public static Pose2d INTRE_TOT_ST_RED_BLUE = new Pose2d(-43, -10, Math.toRadians(90));
    public static Vector2d INTRE_TOT_ST_RED_BLUE_VECTOR = new Vector2d(-43, -10);
    public static Pose2d PRE_POSITION_ST_RED_BLUE = new Pose2d(-43, -10, Math.toRadians(90));
    public static Pose2d JUNCTION_PUNE_ST_RED_BLUE = new Pose2d(-38.7, -1.5, Math.toRadians(130));
//    public static Vector2d JUNCTION_PUNE_ST_RED_BLUE_VECTOR = new Vector2d(-32.5, -4.2); DE DINAINTE DE ACUM
    public static Vector2d JUNCTION_PUNE_ST_RED_BLUE_VECTOR = new Vector2d(-31, -3.5);
    public static Pose2d STACK_ST_RED_BLUE = new Pose2d(-67, -8.5, Math.toRadians(0));





}
