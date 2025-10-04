package org.firstinspires.ftc.teamcode.Recode.Module;

import static org.firstinspires.ftc.teamcode.Core.Module.Outtake.Mixer.Slot2Pose;
import static org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Pusher.retractedPose;
import static org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Trap.closedPose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Drive;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.teamcode.Core.Module.Camera.Camera;
import org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Mixer;
import org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Pusher;
import org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Shooter;
import org.firstinspires.ftc.teamcode.Recode.Module.Outtake.Trap;

import java.util.List;

public class Robot implements HighModuleSimple {
    public enum Actions{
        None,
        ColorGreen,
        ColorPurple,
        WaitToBeFedUp
    }
    Telemetry telemetry;
    public Follower drive;
    public Drive teleOpDrive;
    public List<LynxModule> allHubs;
    protected HardwareMap hardwareMap;
    public Shooter shooter;
    public Trap trap;
    public Mixer mixer;
    public Pusher pusher;
    public Camera camera;
    public IMU imu;
    Constants.Color allianceColor;
    boolean isAuto , readyForShooting =false ,readyToPushGreen = false, readyToPushPurple = false, readyToStop= false;
    double timeTolleranceServo = 700;

    public Robot(HardwareMap hardwareMap, Pose startPose, boolean isAuto, Constants.Color allianceColor, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.isAuto = isAuto;
        camera = new Camera(hardwareMap);
        shooter = new Shooter(hardwareMap);
        trap = new Trap(hardwareMap, closedPose, isAuto);
        mixer = new Mixer(hardwareMap, Slot2Pose, isAuto);
        pusher = new Pusher(hardwareMap, retractedPose, isAuto);
        allHubs = hardwareMap.getAll(LynxModule.class);
        this.allianceColor = allianceColor;
        if(isAuto){
            drive = Constants.createFollower(hardwareMap);
            drive.setStartingPose(startPose);
        }else{
            teleOpDrive = new Drive(hardwareMap);
            imu = hardwareMap.get(IMU.class , "imu");
            RevHubOrientationOnRobot params = new RevHubOrientationOnRobot(
              RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
              RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            );
            imu.initialize(new IMU.Parameters(params));
        }
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public void setAction(Actions action){
        switch (action) {
            case WaitToBeFedUp:
                trap.setWantedColor(Constants.Color.Blue);
                mixer.emptySlot(timeTolleranceServo);
                break;
            case ColorGreen:
                trap.setWantedColor(Constants.Color.Green);
                shooter.shoot();
                pusher.setState(Pusher.States.Retracted , timeTolleranceServo);
                if(mixer.getColorFromSlot(1).equals(Constants.Color.Green)){
                    mixer.setState(Mixer.States.Slot1 , timeTolleranceServo);
                    readyToPushGreen=true;
                }else if (mixer.getColorFromSlot(2).equals(Constants.Color.Green)){
                    mixer.setState(Mixer.States.Slot2 , timeTolleranceServo);
                    readyToPushGreen=true;
                }else if (mixer.getColorFromSlot(3).equals(Constants.Color.Green)){
                    mixer.setState(Mixer.States.Slot3 , timeTolleranceServo);
                    readyToPushGreen=true;
                }
                break;
            case ColorPurple:
                trap.setWantedColor(Constants.Color.Purple);
                shooter.shoot();
                pusher.setState(Pusher.States.Retracted , 0);
                if(mixer.getColorFromSlot(1).equals(Constants.Color.Purple)){
                    mixer.setState(Mixer.States.Slot1 , timeTolleranceServo);
                }else if (mixer.getColorFromSlot(2).equals(Constants.Color.Purple)){
                    mixer.setState(Mixer.States.Slot2 , timeTolleranceServo);
                }else if (mixer.getColorFromSlot(3).equals(Constants.Color.Purple)){
                    mixer.setState(Mixer.States.Slot3 , timeTolleranceServo);
                }
                break;
            case None:
                break;
        }
    }
    @Override
    public void update(){
        if(isAuto){
            drive.update();
            camera.update();
        }else{
            teleOpDrive.update();
        }
        trap.update();
        shooter.update();
        mixer.update();
        pusher.update();
        for(LynxModule hub : allHubs){
            hub.clearBulkCache();
        }

        if(readyToPushGreen && mixer.atTarget()){
            readyToPushGreen = false;
            readyForShooting = true;
        }
        if(readyToPushPurple && shooter.getState() == Shooter.States.ReadyToShoot){
            readyToPushPurple = false;
            readyForShooting = true;
        }
        if(readyForShooting && shooter.getState() == Shooter.States.ReadyToShoot){
            pusher.setState(Pusher.States.Extended , timeTolleranceServo);
            readyForShooting = false;
            readyToStop = true;
        }
        if(readyToStop && pusher.atTarget()){
            trap.setState(Trap.States.Closed, timeTolleranceServo);
            trap.setWantedColor(Constants.Color.Blue);
            pusher.setState(Pusher.States.Retracted,timeTolleranceServo);
            shooter.stop();
            readyToStop = false;
        }

    }
    public boolean isDone(){
        return !drive.isBusy();
    }
}
