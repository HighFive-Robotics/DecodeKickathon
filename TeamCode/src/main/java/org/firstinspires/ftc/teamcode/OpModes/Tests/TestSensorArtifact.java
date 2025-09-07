package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.trapSensorName;
import static org.firstinspires.ftc.teamcode.Constants.currentColor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Hardware.ArtifactSensor;

@TeleOp
public class TestSensorArtifact extends LinearOpMode {

    ArtifactSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sensor = new ArtifactSensor(hardwareMap, trapSensorName);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Color HSV: ", sensor.getHSVColorValues()[0]);
            telemetry.addData("Color HSV: ", sensor.getHSVColorValues()[1]);
            telemetry.addData("Color HSV: ", sensor.getHSVColorValues()[2]);
            telemetry.addData("Color : ", currentColor);
            telemetry.update();
            sensor.update();
        }
    }


}
