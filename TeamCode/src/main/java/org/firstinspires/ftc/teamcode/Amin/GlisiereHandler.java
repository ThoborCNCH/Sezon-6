package org.firstinspires.ftc.teamcode.Amin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class GlisiereHandler extends LinearOpMode {

    public DcMotor brat;

    @Override
    public void runOpMode()
    {
        initMotor();
    }

    public void initMotor()
    {
        brat = hardwareMap.get(DcMotor.class, "brat");
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void ridicaLa(int distance)
    {
        int positieActuala = brat.getCurrentPosition();
        int pozitieFinala = positieActuala + distance;

//        ASTA CARE SE OPRESTE CAND AUJUNEGE
//        SAU V2 IN CARE DOAR SCHIMB TARGETUL/POZITIA LUI SI EL
//        MERGE ACOLO FARA SA MAI SCHIMBE MODUL

         brat.setTargetPosition(pozitieFinala);
         brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         brat.setPower(1);
         while(brat.isBusy())
         {

         }

        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
