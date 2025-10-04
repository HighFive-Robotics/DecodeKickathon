package org.firstinspires.ftc.teamcode.Recode.Testing;

import static org.firstinspires.ftc.teamcode.Constants.currentColor;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Mixer;
import org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Pusher;
import org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Trap;
import org.firstinspires.ftc.teamcode.Recode.Module.Robot;

import java.util.HashMap;
@SuppressWarnings("All")
@TeleOp(name = "\uD83E\uDD50 TeleOp Demo \uD83E\uDD50" )
public class TeleOpDemo extends LinearOpMode {
    public Robot robot;
    public Constants.Color allianceColor = Constants.Color.Blue;
    public boolean isAuto = false;
    public HashMap<String , ElapsedTime> timers = new HashMap<>();
    private final double buttonTimer = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap , new Pose(0,0,0) , isAuto , allianceColor , telemetry);
        timers.put("circle" , new ElapsedTime());
        timers.put("square" , new ElapsedTime());
        timers.put("triangle" , new ElapsedTime());
        timers.put("cross" , new ElapsedTime());
        timers.put("up" , new ElapsedTime());
        timers.put("left" , new ElapsedTime());
        timers.put("down" , new ElapsedTime());
        timers.put("right" , new ElapsedTime());
        timers.put("rightBumper" , new ElapsedTime());
        timers.put("leftBumper" , new ElapsedTime());
        timers.put("rightTrigger" , new ElapsedTime());
        timers.put("leftTrigger" , new ElapsedTime());

        timers.get("circle").reset();
        timers.get("square").reset();
        timers.get("triangle").reset();
        timers.get("cross").reset();
        timers.get("up").reset();
        timers.get("left").reset();
        timers.get("down").reset();
        timers.get("right").reset();
        timers.get("rightBumper").reset();
        timers.get("leftBumper").reset();
        timers.get("rightTrigger").reset();
        timers.get("leftTrigger").reset();
        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {
            robot.teleOpDrive.driveFieldCentric(gamepad1 ,robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            if(gamepad2.circle && timers.get("a").milliseconds() >= 250){
                robot.setAction(Robot.Actions.WaitToBeFedUp);
                timers.get("a").reset();
            }
            if(gamepad2.cross && timers.get("cross").milliseconds() >= 250){
                robot.setAction(Robot.Actions.ColorPurple);
                timers.get("cross").reset();
            }
            if(gamepad2.square && timers.get("square").milliseconds() >= 250){
                robot.setAction(Robot.Actions.ColorGreen);
                timers.get("square").reset();
            }
            if(gamepad2.dpad_up && timers.get("up").milliseconds() >= 250){
                robot.mixer.setState(Mixer.States.Slot1);
                timers.get("up").reset();
            }
            if(gamepad2.dpad_left && timers.get("left").milliseconds() >= 250){
                robot.mixer.setState(Mixer.States.Slot1);
                timers.get("left").reset();
            }
            if(gamepad2.dpad_right && timers.get("right").milliseconds() >= 250){
                robot.mixer.setState(Mixer.States.Slot3);
                timers.get("right").reset();
            }
            if(gamepad2.left_bumper && timers.get("leftBumper").milliseconds() >= 250){
                robot.trap.setState(Trap.States.Open,0);
                timers.get("leftBumper").reset();
            }
            if(gamepad2.right_bumper && timers.get("rightBumper").milliseconds() >= 250){
                robot.trap.setState(Trap.States.Closed,0);
                timers.get("rightBumper").reset();
            }
            if(gamepad2.triangle && timers.get("triangle").milliseconds() >= 250){
                robot.shooter.shoot();
                timers.get("triangle").reset();
            }
            if(gamepad2.left_trigger > 0.4 && timers.get("leftTrigger").milliseconds() >= 250){
                robot.shooter.stop();
                timers.get("leftTrigger").reset();
            }
            if(gamepad2.dpad_down && timers.get("down").milliseconds() >= 250){
                robot.pusher.setState(Pusher.States.Extended,0);
                timers.get("down").reset();
            }
            if(gamepad2.right_trigger > 0.4 && timers.get("rightTrigger").milliseconds() >= 250){
                robot.pusher.setState(Pusher.States.Retracted,0);
                timers.get("rightTrigger").reset();
            }
            robot.update();
            telemetry.addData("Color HSV: ", robot.trap.sensor.getHSVColorValues()[0]);
            telemetry.addData("Color HSV: ",  robot.trap.sensor.getHSVColorValues()[1]);
            telemetry.addData("Color HSV: ",  robot.trap.sensor.getHSVColorValues()[2]);
            telemetry.addData("Color : ", currentColor);
            telemetry.addData("Mixer at target : ", robot.mixer.atTarget());
            telemetry.update();
        }
    }
}
