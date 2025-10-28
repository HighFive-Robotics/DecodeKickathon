package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Algorithms.VelocityPID;

@TeleOp
@Config
public class TestVelocity extends LinearOpMode {
    DcMotorEx motor;
    VelocityPID pid;
    public static double kp=0.005,ki=0.001,kd=0.002,kf=0.0035;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class , shooterMotorName);

        pid = new VelocityPID(kp,ki,kd,kf ,timer);
        double targetVelo = 1200;

        waitForStart();
        while (opModeIsActive()){
            pid.setSetPoint(targetVelo);
            motor.setPower(pid.calculate(motor.getVelocity()));
        }

    }
}
