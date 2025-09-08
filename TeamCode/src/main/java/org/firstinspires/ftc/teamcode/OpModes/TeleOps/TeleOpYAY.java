package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import static org.firstinspires.ftc.teamcode.Constants.currentColor;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Mixer;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Pusher;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Trap;
import org.firstinspires.ftc.teamcode.Core.Robot;

@TeleOp
public class TeleOpYAY extends LinearOpMode {

    Robot robot;
    ElapsedTime time1 = new ElapsedTime(),time2 = new ElapsedTime(), time3 = new ElapsedTime(), time4 = new ElapsedTime(), time5 = new ElapsedTime(), time6 = new ElapsedTime();
    ElapsedTime time7 = new ElapsedTime(), time8 = new ElapsedTime(), time9 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, new Pose(), false, Constants.Color.Red, telemetry);

        waitForStart();

        while (opModeIsActive()){

            robot.teleOpDrive.driveFieldCentric(gamepad1, robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            if(gamepad1.a){
                robot.imu.resetYaw();
            }

            if(gamepad2.circle && time1.milliseconds() >= 250){
                robot.setAction(Robot.Actions.WaitToBeFedUp);
                time1.reset();
            }
            if(gamepad2.cross && time2.milliseconds() >= 250){
                robot.setAction(Robot.Actions.ColorPurple);
                time2.reset();
            }
            if(gamepad2.square && time3.milliseconds() >= 250){
                robot.setAction(Robot.Actions.ColorGreen);
                time3.reset();
            }
            if(gamepad2.dpad_up && time1.milliseconds() >= 250){
                robot.mixer.setState(Mixer.States.Slot2);
                time1.reset();
            }
            if(gamepad2.dpad_left && time2.milliseconds() >= 250){
                robot.mixer.setState(Mixer.States.Slot1);
                time2.reset();
            }
            if(gamepad2.dpad_right && time3.milliseconds() >= 250){
                robot.mixer.setState(Mixer.States.Slot3);
                time3.reset();
            }
            if(gamepad2.left_bumper && time4.milliseconds() >= 250){
                robot.trap.setState(Trap.States.Open);
                time4.reset();
            }
            if(gamepad2.right_bumper && time5.milliseconds() >= 250){
                robot.trap.setState(Trap.States.Closed);
                time5.reset();
            }
            if(gamepad2.triangle && time6.milliseconds() >= 250){
                robot.shooter.shoot();
                time6.reset();
            }
            if(gamepad2.left_trigger > 0.4 && time7.milliseconds() >= 250){
                robot.shooter.stop();
                time7.reset();
            }
            if(gamepad2.dpad_down && time8.milliseconds() >= 250){
                robot.pusher.setState(Pusher.States.Extended);
                time8.reset();
            }
            if(gamepad2.right_trigger > 0.4 && time9.milliseconds() >= 250){
                robot.pusher.setState(Pusher.States.Retracted);
                time7.reset();
            }
            telemetry.addData("Color HSV: ", robot.trap.sensor.getHSVColorValues()[0]);
            telemetry.addData("Color HSV: ",  robot.trap.sensor.getHSVColorValues()[1]);
            telemetry.addData("Color HSV: ",  robot.trap.sensor.getHSVColorValues()[2]);
            telemetry.addData("Color : ", currentColor);
            telemetry.addData("Mixer at target : ", robot.mixer.atTarget());
            telemetry.update();

            robot.update();
        }
    }
}
