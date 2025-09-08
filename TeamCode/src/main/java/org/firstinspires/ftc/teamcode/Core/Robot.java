package org.firstinspires.ftc.teamcode.Core;


import static org.firstinspires.ftc.teamcode.Constants.mixerColors;
import static org.firstinspires.ftc.teamcode.Core.Module.Outtake.Mixer.Slot2Pose;
import static org.firstinspires.ftc.teamcode.Core.Module.Outtake.Pusher.States.Extended;
import static org.firstinspires.ftc.teamcode.Core.Module.Outtake.Pusher.States.Retracted;
import static org.firstinspires.ftc.teamcode.Core.Module.Outtake.Pusher.retractedPose;
import static org.firstinspires.ftc.teamcode.Core.Module.Outtake.Trap.closedPose;

import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.teamcode.Core.Module.Camera.Camera;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Mixer;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Pusher;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Shooter;
import org.firstinspires.ftc.teamcode.Core.Module.Outtake.Trap;

import java.util.List;

public class Robot implements HighModuleSimple {

    Telemetry telemetry;
    public Actions lastAction = Actions.None;
    public Follower drive;
    public Drive teleOpDrive;
    public List<LynxModule> allHubs;
    protected HardwareMap hardwareMap;

    public IMU imu;

    public Camera camera;
    public Shooter shooter;
    public Trap trap;
    public Mixer mixer;
    public Pusher pusher;

    boolean isAuto;

    ElapsedTime failSafeTimer = new ElapsedTime();

    boolean prepareToShootGreen = false, prepareToShootPurple = false, prepareToPush = false, prepareToRetract = false;

    public enum Actions {
        WaitToBeFedUp,
        ColorGreen,
        ColorPurple,
        CollectSampleFromPerimeter,
        None
    }

    public Robot(HardwareMap hardwareMap, Pose startPose, boolean isAuto, Constants.Color allianceColor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.isAuto = isAuto;
        if (isAuto) {
            drive = Constants.createFollower(hardwareMap);
            drive.setStartingPose(startPose);
            camera = new Camera(hardwareMap);
        } else {
            teleOpDrive = new Drive(hardwareMap);
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(LogoFacingDirection.LEFT,
                     RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
            imu.initialize(new IMU.Parameters(RevOrientation));
        }
        shooter = new Shooter(hardwareMap);
        trap = new Trap(hardwareMap, closedPose, isAuto);
        mixer = new Mixer(hardwareMap, Slot2Pose, isAuto);
        pusher = new Pusher(hardwareMap, retractedPose, isAuto);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    public void setAction(Actions action) {
        switch (action) {
            case WaitToBeFedUp:
                trap.setWantedColor(Constants.Color.Blue);
                if (mixerColors[0] == Constants.Color.None) {
                    mixer.setState(Mixer.States.Slot1);
                } else if (mixerColors[1] == Constants.Color.None) {
                    mixer.setState(Mixer.States.Slot2);
                } else if (mixerColors[2] == Constants.Color.None) {
                    mixer.setState(Mixer.States.Slot3);
                }
                break;
            case ColorGreen:
                if (mixerColors[0] == Constants.Color.Green) {
                    mixer.setState(Mixer.States.Slot1, 700);
                    shooter.shoot();
                    mixerColors[0] = Constants.Color.None;
                } else if (mixerColors[1] == Constants.Color.Green) {
                    mixer.setState(Mixer.States.Slot2, 700);
                    shooter.shoot();
                    mixerColors[1] = Constants.Color.None;
                } else if (mixerColors[2] == Constants.Color.Green) {
                    mixer.setState(Mixer.States.Slot3, 700);
                    shooter.shoot();
                    mixerColors[2] = Constants.Color.None;
                }
                prepareToShootGreen = true;
                break;
            case ColorPurple:
                if (mixerColors[0] == Constants.Color.Purple) {
                    mixer.setState(Mixer.States.Slot1, 700);
                    shooter.shoot();
                    mixerColors[0] = Constants.Color.None;
                } else if (mixerColors[1] == Constants.Color.Purple) {
                    mixer.setState(Mixer.States.Slot2, 700);
                    shooter.shoot();
                    mixerColors[1] = Constants.Color.None;
                } else if (mixerColors[2] == Constants.Color.Purple) {
                    mixer.setState(Mixer.States.Slot3, 700);
                    shooter.shoot();
                    mixerColors[2] = Constants.Color.None;
                }
                prepareToShootPurple = true;
                break;
        }
    }

    @Override
    public void update() {
        Constants.Globals.voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        if (prepareToShootGreen && mixer.atTarget()) {
            prepareToShootGreen = false;
            trap.setWantedColor(Constants.Color.Green);
            prepareToPush = true;
        }
        if (prepareToShootPurple && mixer.atTarget()) {
            prepareToShootPurple = false;
            trap.setWantedColor(Constants.Color.Purple);
            prepareToPush = true;
        }
        if (prepareToPush && shooter.getState() == Shooter.States.ReadyToShoot) {
            prepareToPush = false;
            pusher.setState(Extended,700);
            prepareToRetract = true;
        }
        if (prepareToRetract && pusher.atTarget()){
            pusher.setState(Retracted);
            trap.setState(Trap.States.Closed);
            trap.setWantedColor(Constants.Color.Blue);
            prepareToRetract = false;
        }
        if (isAuto) {
            drive.update();
            camera.update();
        } else {
            teleOpDrive.update();
        }
        trap.update();
        shooter.update();
        mixer.update();
        pusher.update();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public boolean isDone() {
        return !drive.isBusy();
    }
}
