package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@TeleOp(name = "Shooter BUMBUM")
public class ShooterUIUIUI extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    HighMotor mTop , mBot;
    double kp,kd,ki,kf;
    double d1;
    public static double targetVelo = 5;
    @Override
    public void runOpMode() throws InterruptedException {
        mTop = new HighMotor(hardwareMap.get(DcMotorEx.class,"OMT"),HighMotor.RunMode.Velocity , false , true , false );
        mBot =  new HighMotor(hardwareMap.get(DcMotorEx.class,"OMT"),HighMotor.RunMode.Standard , true);
        mTop.setEncoderResolution(28);
        mTop.setPIDCoefficients(kp,kd,ki,kf);
        mTop.setWheelDiameter(0.048);
        waitForStart();
        while (opModeIsActive()){
            mTop.setTarget(targetVelo);
            mTop.update();
            mBot.setPower(mTop.getPower());
            mBot.update();
            telemetry.addData("",mTop.getVelocity());
        }
    }
}
