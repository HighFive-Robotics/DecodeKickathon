package org.firstinspires.ftc.teamcode.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.mixerServoName;
import static org.firstinspires.ftc.teamcode.Constants.currentColor;
import static org.firstinspires.ftc.teamcode.Constants.mixerColors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

public class Mixer implements HighModule {

    public static double Slot1Pose=0, Slot2Pose=0.5, Slot3Pose=1;
    HighServo mixerServo;
    public States state = States.None;
    private double target;

    public enum States{
        Slot1,
        Slot2,
        Slot3,
        None
    }
    public Mixer(HardwareMap hardwareMap, double initPosition, boolean isAuto){
        mixerServo = new HighServo(hardwareMap.get(Servo.class, mixerServoName) ,HighServo.RunMode.Standard, initPosition, isAuto);
        target = initPosition;
    }
    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Slot1:
                setTarget(Slot1Pose);
                break;
            case Slot2:
                setTarget(Slot2Pose);
                break;
            case Slot3:
                setTarget(Slot3Pose);
                break;
            case None:
                break;
        }
    }
    public void setState(States state, double time) {
        this.state = state;
        switch (state) {
            case Slot1:
                setTarget(Slot1Pose, time);
                break;
            case Slot2:
                setTarget(Slot2Pose, time);
                break;
            case Slot3:
                setTarget(Slot3Pose, time);
                break;
            case None:
                break;
        }
    }


    @Override
    public void setTarget(double target) {
        this.target = target;
        mixerServo.setPosition(target);
    }

    @Override
    public void setTarget(double target, double time) {
        this.target = target;
        mixerServo.setPosition(target, time);
    }

    @Override
    public boolean atTarget() {
        return mixerServo.atTarget();
    }
    public States getState() {
        return state;
    }
    @Override
    public double getTarget() {
        return target;
    }

    @Override
    public void update() {
        mixerServo.update();
        if(state== States.Slot1 && currentColor!=None){
            mixerColors[0]=currentColor;
        } else if (state==States.Slot2 && currentColor!=None) {
            mixerColors[1]=currentColor;
        }
        else if(state==States.Slot3 && currentColor!=None){
            mixerColors[2]=currentColor;
        }
    }
}
