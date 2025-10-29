package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.shooterMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Core.Algorithms.VelocityPID;

@TeleOp
@Config
public class TestVelocity extends LinearOpMode {

    ElapsedTime zaza = new ElapsedTime();
    DcMotorEx motor;
    VelocityPID pid;
    PIDFController pidf;
    FtcDashboard dashboard;
    public static double targetVelo = 5;
    public double velo = 0;

    public double currentVelo;
    public double delta = 0;
    public double lastx = 0;
    public static double kp=0.045,ki=0.045,kd=0.0000001,kf=0.00016;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer1 = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        timer1.reset();
        dashboard = FtcDashboard.getInstance();
        motor = hardwareMap.get(DcMotorEx.class , shooterMotorName);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pidf = new PIDFController(kp,ki,kd,kf);
        pidf.reset();
        pidf.setTolerance(0.2);
        pid = new VelocityPID(kp,ki,kd,kf ,timer);
        pid.reset();
        pid.setTolerance(20);
        telemetry = dashboard.getTelemetry();
        boolean useCustom = true;
        waitForStart();
        zaza.reset();
        while (opModeIsActive()){
            if(gamepad1.a && timer1.milliseconds() > 350){
                useCustom = !useCustom;
                timer1.reset();
            }
            delta = motor.getCurrentPosition() - lastx;
            lastx = motor.getCurrentPosition();
            currentVelo = 1000 * delta / zaza.milliseconds();
            zaza.reset();
            pid.setPIDF(kp,ki,kd,kf);
            pidf.setPIDF(kp,ki,kd,kf);
            pidf.setSetPoint(targetVelo);
            pid.setSetPoint(targetVelo);
//            velo = motor.getVelocity(AngleUnit.RADIANS) * 0.096*Math.PI;
            velo = (motor.getVelocity()/28)*0.096;
            if(useCustom){
                motor.setPower(pid.calculate(velo));
                telemetry.addData("Target" , targetVelo);
                telemetry.addData("Error" , pid.getVelocityError());
                telemetry.addData("Position" , motor.getCurrentPosition());
                telemetry.addData("Current Velo" , motor.getVelocity());
                telemetry.addData("Current Velo ++" , velo);
                telemetry.addLine("USES CUSTOM");
                telemetry.update();
            }else {
                motor.setPower(pidf.calculate(motor.getVelocity()));
                telemetry.addData("Target" , targetVelo);
                telemetry.addData("Error" , pidf.getVelocityError());
                telemetry.addData("Position" , motor.getCurrentPosition());
                telemetry.addData("Current Velo" , motor.getVelocity());
                telemetry.addData("Current Velo ++" , currentVelo);
                telemetry.addLine("USES FTC LIB");
                telemetry.update();
            }
        }

    }
}
