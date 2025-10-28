package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Camera.Camera;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.HashMap;

@Config
@TeleOp
public class CameraTest extends LinearOpMode {
    Robot robot;
    public static double x = 0, y = 0, rot = 0;

    public static Pose robotPose = new Pose(x, y, rot);
    public Pose targetPose = new Pose(0,0,0);
//    Camera camera;
    HashMap<String, ElapsedTime> timers = new HashMap<>();

    @Override
    public void runOpMode() throws InterruptedException {
//        camera = new Camera(hardwareMap);
        robot = new Robot(hardwareMap , new Pose(0,0,0) , true , Constants.Color.Red , telemetry);
        timers.put("a" ,new ElapsedTime());
        timers.put("b" ,new ElapsedTime());
        timers.put("x" ,new ElapsedTime());
        timers.get("x").reset();
        timers.get("a").reset();
        timers.get("b").reset();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && timers.get("a").milliseconds() >= 350) {
                robot.camera.update();
                targetPose = robot.camera.targetPose(robot.drive.getPose());
                telemetry.addData("Pose: ", robot.camera.targetPose(robotPose).toString());
                telemetry.addData("Distance to AprilTag: ", robot.camera.distanceToAprilTag().toString());
                telemetry.addData("Case: ", robot.camera.getCase());
                telemetry.update();
                timers.get("a").reset();
            }
            if(gamepad1.b && timers.get("b").milliseconds() >= 350) {
                Path target = new Path(new BezierLine(robot.drive.getPose() , targetPose));
                robot.drive.activateAllPIDFs();
                target.setLinearHeadingInterpolation(robot.drive.getPose().getHeading() ,
                        targetPose.getHeading());
                robot.drive.followPath(target);
                timers.get("b").reset();
            }
            if(gamepad1.square && timers.get("x").milliseconds() >= 350){
                robot.drive.deactivateAllPIDFs();
                timers.get("x").reset();
            }
            robot.update();
        }
    }
}
