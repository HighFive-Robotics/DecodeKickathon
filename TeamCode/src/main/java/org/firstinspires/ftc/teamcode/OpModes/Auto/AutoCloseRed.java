package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous
public class AutoCloseRed extends LinearOpMode {
    public Robot robot;

    private int state = 1;
    private int Case = -1;

    public Pose startPose = new Pose(106, 134, Math.toRadians(180));
    private final Pose detectPose = new Pose(96, 110, Math.toRadians(130));
    private final Pose scorePose = new Pose(112, 112, Math.toRadians(45));
    private final Pose parkPose = new Pose(100, 50, Math.toRadians(180));
    private ElapsedTime timer = new ElapsedTime();

    BezierLine detectCase = new BezierLine(startPose, detectPose);
    BezierLine scorePreload = new BezierLine(detectPose, scorePose);
    BezierLine park = new BezierLine(scorePose, parkPose);
    Path path1, path2, path3;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, startPose, true, Constants.Color.Blue, telemetry);

        path1 = new Path(detectCase);
        path1.setLinearHeadingInterpolation(startPose.getHeading(), detectPose.getHeading());
        path2 = new Path(scorePreload);
        path2.setLinearHeadingInterpolation(detectPose.getHeading(), scorePose.getHeading());
        path3 = new Path(park);
        path3.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        waitForStart();
        while (opModeIsActive()) {
            switch (state) {
                case 1:
                    robot.drive.followPath(path1, true);
                    state++;
                    break;
                case 2:
                    if (robot.isDone() && robot.camera.getCase() != Constants.Case.None) {
                        telemetry.addData("Case is:", robot.camera.getCase());
                        telemetry.update();
                        timer.reset();
                        state++;
                    }
                    break;
                case 3:
                    if (robot.isDone()) {
                        robot.drive.followPath(path2, true);
                        timer.reset();
                        state++;
                    }
                    break;
                case 4:
                    if(robot.isDone()){
                        robot.drive.followPath(path3, true);
                    }
            }
            robot.update();
        }//
    }
}
