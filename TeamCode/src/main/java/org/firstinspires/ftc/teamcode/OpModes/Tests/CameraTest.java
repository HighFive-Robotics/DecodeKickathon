package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Module.Camera.Camera;

@Config
@TeleOp
public class CameraTest extends LinearOpMode {

    public static double x = 114, y = 279, rot = 2.35;

    public static Pose robotPose = new Pose(x, y, rot);
    Camera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new Camera(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                camera.update();
                telemetry.addData("Pose: ", camera.targetPose(robotPose).toString());
                telemetry.addData("Distance to AprilTag: ", camera.distanceToAprilTag().toString());
                telemetry.addData("Case: ", camera.getCase());
                telemetry.update();
            } else {
                telemetry.addLine("Nothing");
                telemetry.update();
            }
        }
    }
}
