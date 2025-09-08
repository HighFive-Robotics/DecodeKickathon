package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static org.firstinspires.ftc.teamcode.Constants.mixerColors;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous
public class AutoFarBlue extends LinearOpMode {

    public Robot robot;

    private int state = 1;

    public Pose startPose = new Pose(48, 10, 0);
    private final Pose scorePose = new Pose(32, 112, Math.toRadians(135));
    private final Pose parkPose = new Pose(44, 50, Math.toRadians(0));
    private ElapsedTime timer = new ElapsedTime();

    BezierLine scorePreload = new BezierLine(startPose, scorePose);
    BezierLine park = new BezierLine(scorePose, parkPose);
    Path path1, path2, path3;

    @Override
    public void runOpMode() throws InterruptedException {

        mixerColors[0] = Constants.Color.Purple;
        mixerColors[1] = Constants.Color.Green;
        mixerColors[2] = Constants.Color.Purple;

        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Blue, telemetry);
        path1 = new Path(scorePreload);
        path1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        path3 = new Path(park);
        path3.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        waitForStart();
        while (opModeIsActive()) {
            switch (state) {
                case 1:
                    if (robot.isDone() && robot.camera.getCase() != Constants.Case.None) {
                        telemetry.addData("Case is:", robot.camera.getCase());
                        telemetry.update();
                        timer.reset();
                        state++;
                    }
                    break;
                case 2:
                    if (robot.isDone() && timer.milliseconds() >= 13000) {
                        robot.drive.followPath(path1);
                        state++;
                    }
                    break;
                case 3:
                    if (robot.isDone()) {
                        robot.drive.followPath(path3, true);
                        timer.reset();
                        state++;
                    }
                    break;
            }
            robot.update();
        }
    }
}
