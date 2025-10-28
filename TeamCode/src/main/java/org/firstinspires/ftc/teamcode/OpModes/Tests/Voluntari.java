package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class Voluntari extends LinearOpMode {
    DcMotor FrontLeftMotor;
    DcMotor BackLeftMotor;
    DcMotor FrontRightMotor;
    DcMotor BackRightMotor;
    DcMotor IntakeMotor;
    @Override
    public void runOpMode() throws InterruptedException{
        FrontLeftMotor = hardwareMap.get(DcMotor.class,"LFM");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "LBM");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "RFM");
        BackRightMotor = hardwareMap.get(DcMotor.class, "RBM");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IM");
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive())
        {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            IntakeMotor.setPower(0);
            if(gamepad2.right_trigger >= 0.5){
                IntakeMotor.setPower(1);
            }
            if(gamepad2.left_trigger >= 0.5) {
                IntakeMotor.setPower(-1);
            }
            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx),1);
            double FLPower = (y+x+rx)/denominator;
            double BLPower = (y-x+rx)/denominator;
            double FRPower = (y-x-rx)/denominator;
            double BRPower = (y+x-rx)/denominator;
            FrontLeftMotor.setPower(FLPower);
            BackLeftMotor.setPower(BLPower);
            FrontRightMotor.setPower(FRPower);
            BackRightMotor.setPower(BRPower);
        }
    }
}
