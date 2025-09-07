package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;

@TeleOp
public class TestMotor extends LinearOpMode {

    HighMotor m1,m2,m3,m4;

    @Override
    public void runOpMode() throws InterruptedException {
        m1 = new HighMotor(hardwareMap.get(DcMotorEx.class, "LFM"), HighMotor.RunMode.Time, false);
        m2 = new HighMotor(hardwareMap.get(DcMotorEx.class, "RFM"), HighMotor.RunMode.Time, false);
        m3 = new HighMotor(hardwareMap.get(DcMotorEx.class, "LBM"), HighMotor.RunMode.Time, false);
        m4 = new HighMotor(hardwareMap.get(DcMotorEx.class, "RBM"), HighMotor.RunMode.Time, false);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                m1.setPower(1, 1000);
                telemetry.addData("Motor ", "LFM works");
            }
            if(gamepad1.b){
                m2.setPower(1, 1000);
                telemetry.addData("Motor ", "RFM works");
            }
            if(gamepad1.x){
                m3.setPower(1, 1000);
                telemetry.addData("Motor ", "LBM works");
            }
            if(gamepad1.x){
                m4.setPower(1, 1000);
                telemetry.addData("Motor ", "RBM works");
            }
            telemetry.update();
            m1.update();
            m2.update();
            m3.update();
            m4.update();
        }
    }
}
