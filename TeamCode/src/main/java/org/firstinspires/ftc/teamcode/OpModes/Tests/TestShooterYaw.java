package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static java.lang.Math.atan2;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Robot;

@TeleOp
public class TestShooterYaw extends LinearOpMode {

    public Pose goalPose = new Pose(10,134), targetPose;
    int state = 1;

    Robot robot;
    Path path;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, new Pose(9,9,0), false, Constants.Color.Blue, telemetry);

        waitForStart();
        while(opModeIsActive()){
            switch (state){
                case 1:
                    robot.teleOpDrive.driveFieldCentric(gamepad1, robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

                    if(gamepad1.a){
                        robot.imu.resetYaw();
                    }
                break;
                case 2:
                    targetPose = new Pose(robot.drive.getPose().getX(), robot.drive.getPose().getY(), getAngle(robot.drive.getPose()));
                    BezierLine bezierLine = new BezierLine(robot.drive.getPose(), targetPose);
                    path = new Path(bezierLine);
                    path.setLinearHeadingInterpolation(robot.drive.getPose().getHeading(), targetPose.getHeading());

                    if(gamepad1.b){
                        robot.drive.followPath(path, true);
                    }
            }

            if(gamepad1.left_bumper){
                state = 1;
                robot.drive.deactivateAllPIDFs();
            }

            if(gamepad1.right_bumper){
                state = 2;
                robot.drive.activateAllPIDFs();
            }

            telemetry.addData("state", state);
            telemetry.addData("Pose target", targetPose);
            telemetry.addData("Pose current", robot.drive.getPose());
            telemetry.update();
            robot.update();
        }
    }

    public double getAngle(Pose pose){
        double angle;
        angle = atan2(goalPose.getY() - pose.getY(),goalPose.getX() - pose.getX());
        return angle;
    }
}
