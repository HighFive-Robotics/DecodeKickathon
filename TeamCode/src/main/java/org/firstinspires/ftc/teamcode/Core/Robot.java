package org.firstinspires.ftc.teamcode.Core;



import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.teamcode.Core.Module.Camera.Camera;

import java.util.List;

public class Robot implements HighModuleSimple {

    Telemetry telemetry;
    public Actions lastAction = Actions.None;
    public Follower drive;
    public Drive teleOpDrive;
    public List<LynxModule> allHubs;
    protected HardwareMap hardwareMap;

    public Camera camera;

    boolean isAuto;

    ElapsedTime failSafeTimer = new ElapsedTime();

    boolean goToCollectSpecimen = false, finishCollectSpecimen = false, goToScoreSpecimen = false, finishScoreSpecimen = false;
    boolean setOuttakeForTransfer = false, liftGoToCollectSample = false, intakeGoToTransferSample = false, intakeGoToTransferSpecimen = false;
    boolean setIntakeForCollecting1 = false, setIntakeForCollecting2 = false, extendSlidesForCollecting2 = false, setIntakeForCollecting3 = false, setIntakeForCollecting4 = false;
    boolean shouldMakeTransferSample = false, getOuttakeToScoreSampleTransfer = false, failSafeTransfer = false, finishOuttakeScoreSampleTransfer = false, extendOuttakeTransferSample = false;

    public enum Actions {
        GoToCollectSpecimen,
        GoToScoreSpecimen,
        SpecimenWithIntakeUp,
        SpecimenWithIntakeDown,
        IntakeGoToTransfer,
        IntakeGoToTransferSpecimen,
        StartCollecting,
        StartCollectingWithWait,
        StartCollectingWithExtension,
        StartCollectingSpecific,
        OuttakeGoToTransferSample,
        TransferSample,
        CollectSampleFromPerimeter,
        None
    }

    public Robot(HardwareMap hardwareMap, Pose startPose , boolean isAuto, Constants.Color allianceColor, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.isAuto = isAuto;
        if (isAuto) {
            drive = Constants.createFollower(hardwareMap);
            drive.setStartingPose(startPose);
            camera = new Camera(hardwareMap);
        } else {

        }
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void setAction(Actions action) {
        switch (action) {

        }
    }

    @Override
    public void update() {
        Constants.Globals.voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        if(isAuto){
            drive.update();
            camera.update();
        }
        else {
            teleOpDrive.update();
        }
        for(LynxModule hub:allHubs){
            hub.clearBulkCache();
        }
    }

    public boolean isDone(){
        return !drive.isBusy();
    }
}
