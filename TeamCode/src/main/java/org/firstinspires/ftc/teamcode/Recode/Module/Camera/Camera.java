package org.firstinspires.ftc.teamcode.Recode.Module.Camera;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.webcamName;
import static org.firstinspires.ftc.teamcode.Constants.randomizedCase;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
public class Camera implements HighModuleSimple {

    public AprilTagProcessor aprilTagProcessor;
    public ArrayList<AprilTagDetection> detections = new ArrayList<>();

    public Camera(HardwareMap hardwareMap)
    {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1385.92f,1385.92f,951.982f,534.084f)
                .setCameraPose(new Position(DistanceUnit.CM, 12.5, 15, 26,0), new YawPitchRollAngles(AngleUnit.DEGREES, 0,0,40,0))
                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();
        VisionPortal camera = new VisionPortal.Builder()
                .setCameraResolution(new Size(1920,1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTagProcessor)
                .build();
        FtcDashboard.getInstance().startCameraStream(camera,60);
    }

    @Override
    public void update() {

    }
}
