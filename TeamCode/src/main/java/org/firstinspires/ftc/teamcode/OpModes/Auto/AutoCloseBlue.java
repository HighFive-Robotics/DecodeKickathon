package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static org.firstinspires.ftc.teamcode.Constants.mixerColors;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Mixer;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Pusher;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Trap;
import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous
public class AutoCloseBlue extends LinearOpMode {

    public Robot robot;

    private int state = 1;
    private Constants.Case a = Constants.Case.None;

    public Pose startPose = new Pose(38, 134, 0);
    private final Pose detectPose = new Pose(48, 110, Math.toRadians(50));
    private final Pose scorePose = new Pose(40, 102, Math.toRadians(135));
    private final Pose parkPose = new Pose(45, 125, Math.toRadians(0));
    private ElapsedTime timer = new ElapsedTime();

    BezierLine detectCase = new BezierLine(startPose, detectPose);
    BezierLine scorePreload = new BezierLine(detectPose, scorePose);
    BezierLine park = new BezierLine(scorePose, parkPose);
    Path path1, path2, path3;

    @Override
    public void runOpMode() throws InterruptedException {

        mixerColors[0] = Constants.Color.Purple;
        mixerColors[1] = Constants.Color.Green;
        mixerColors[2] = Constants.Color.Purple;

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
                        a = robot.camera. getCase();
                        telemetry.update();
                        timer.reset();
                        state++;
                    }
                    break;
                case 3:
                    if(a == Constants.Case.Left){
                        robot.mixer.setState(Mixer.States.Slot2);
                        timer.reset();;
                        state++;

                    } else {
                        robot.mixer.setState(Mixer.States.Slot1);
                        timer.reset();;
                        state++;
                    }
                    break;
                case 4:
                    if (robot.isDone()) {
                        robot.drive.followPath(path2, false);
                        timer.reset();
                        state++;
                    }
                    break;
                case 5:
                    if (robot.isDone()) {
                        robot.trap.setState(Trap.States.Open);
                        robot.shooter.shoot();
                        timer.reset();
                        state++;
                    }
                    break;
                case 6:
                    if (timer.milliseconds() >= 2000) {
                        robot.pusher.setState(Pusher.States.Extended);
                        robot.trap.setState(Trap.States.Closed);
                        timer.reset();
                        state++;
                    }
                    break;
                case 7:
                    if (timer.milliseconds() >= 250) {
                        if(a == Constants.Case.Middle){
                            robot.mixer.setState(Mixer.States.Slot2);
                            timer.reset();;
                            state++;
                        } else {
                            robot.mixer.setState(Mixer.States.Slot3);
                            timer.reset();;
                            state++;
                        }
                    }
                    break;
                case 8:
                    if (robot.isDone()) {
                        robot.trap.setState(Trap.States.Open);
                        robot.shooter.shoot();
                        timer.reset();
                        state++;
                    }
                    break;
                case 9:
                    if (timer.milliseconds() >= 2000) {
                        robot.pusher.setState(Pusher.States.Extended);
                        robot.trap.setState(Trap.States.Closed);
                        timer.reset();
                        state++;
                    }
                    break;
                case 10:
                    if (timer.milliseconds() >= 250) {
                        if(a == Constants.Case.Right){
                            robot.mixer.setState(Mixer.States.Slot2);
                            timer.reset();;
                            state++;

                        } else if(a == Constants.Case.Left){
                            robot.mixer.setState(Mixer.States.Slot1);
                            timer.reset();;
                            state++;
                        } else if(a == Constants.Case.Middle){
                            robot.mixer.setState(Mixer.States.Slot3);
                            timer.reset();;
                            state++;
                        }
                    }
                    break;
                case 11:
                    if (robot.isDone()) {
                        robot.trap.setState(Trap.States.Open);
                        robot.shooter.shoot();
                        timer.reset();
                        state++;
                    }
                    break;
                case 12:
                    if (timer.milliseconds() >= 2000) {
                        robot.pusher.setState(Pusher.States.Extended);
                        robot.trap.setState(Trap.States.Closed);
                        timer.reset();
                        state++;
                    }
                    break;
                case 13:
                    if(robot.isDone()){
                        robot.drive.followPath(path3, true);
                    }
            }
            telemetry.addData("case", state);
            telemetry.addData("robot pose", robot.drive.getPose());
            robot.update();
        }
    }
}
