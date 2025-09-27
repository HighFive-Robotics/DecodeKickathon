package org.firstinspires.ftc.teamcode.Recode.Module.Camera;

import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.webcamName;
import static org.firstinspires.ftc.teamcode.Constants.randomizedCase;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
public class Camera implements HighModuleSimple {

    public AprilTagProcessor aprilTagProcessor;
    public ArrayList<AprilTagDetection> detections = new ArrayList<>();

    ElapsedTime timer = new ElapsedTime();

    public static double xOffset = 0, yOffset = 0;

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

    public int getAprilTagId(int index){
        if(!detections.isEmpty()){
            return detections.get(index).id;
        }
        return -1;
    }

    public Constants.Case getCase(){
        if(!detections.isEmpty()){
            for(int i = 0; i < detections.size(); i++) {
                if (getAprilTagId(i) == 21) {
                    randomizedCase = Constants.Case.Left;
                    break;
                } else if (getAprilTagId(i) == 22) {
                    randomizedCase = Constants.Case.Middle;
                    break;
                } else if (getAprilTagId(i) == 23) {
                    randomizedCase = Constants.Case.Right;
                    break;
                } else {
                    randomizedCase = Constants.Case.None;
                }
            }
        }
        return randomizedCase;
    }

    public Pose distanceToAprilTag(){
        Pose pose = new Pose();
        AprilTagDetection detection = null;
        int index = -1;
        Vector vector = new Vector();
        if(!detections.isEmpty()){
            for(int i = 0; i < detections.size(); i++) {
                if (getAprilTagId(i) == 20 || getAprilTagId(i) == 24) {
                    index = i;
                    detection = detections.get(i);
                    break;
                }
            }
        }

        if( detection != null && index != -1){
            vector = new Vector(new Pose(-detection.ftcPose.x,detection.ftcPose.y));
            vector.rotateVector(-detection.ftcPose.yaw);
            pose = new Pose(vector.getXComponent(), vector.getYComponent(), detection.ftcPose.yaw);
        }

        return pose;
    }

    public Pose targetPose(Pose currentPose){
        Pose temp = distanceToAprilTag();
        temp = temp.rotate(-currentPose.getHeading(), false);
        return new Pose(currentPose.getX() + temp.getX() + xOffset,currentPose.getY() + temp.getY() + yOffset, currentPose.getHeading() + temp.getHeading());
    }

    @Override
    public void update() {
        if(timer.milliseconds()>=50) {
            detections = aprilTagProcessor.getDetections();
            timer.reset();
        }
    }
}
