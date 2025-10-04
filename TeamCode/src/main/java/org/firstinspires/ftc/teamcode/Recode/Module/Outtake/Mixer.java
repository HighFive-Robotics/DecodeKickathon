package org.firstinspires.ftc.teamcode.Recode.Module.Outtake;

import static org.firstinspires.ftc.teamcode.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.Constants.DeviceNames.mixerServoName;
import static org.firstinspires.ftc.teamcode.Constants.currentColor;
import static org.firstinspires.ftc.teamcode.Constants.mixerColors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighServo;

@SuppressWarnings("All")
public class Mixer implements HighModule {
    public enum States{
        Slot1(0),
        Slot2(0.5),
        Slot3(1),
        None(0);
        public double position;
        States(double position){
            this.position = position;
        }
    }
    HighServo mixerServo;
    private double target;
    public States state = States.None;

    /**
     *
     * @param hardwareMap
     * @param initPosition
     * @param isAuto
     */
    public Mixer(@NonNull HardwareMap hardwareMap, double initPosition, boolean isAuto){
        mixerServo = new HighServo(hardwareMap.get(Servo.class, mixerServoName) ,HighServo.RunMode.Standard, initPosition, isAuto);
        target = initPosition;
    }

    /**
     *
     * @param state : Slot1, Slot2, Slot3
     * @param time : Un vector, daca nu pui , nu se va folosi timp
     */
    public void setState(@NonNull States state, double ... time){
        this.state = state;
        switch (state){
            case Slot1:
                if(time.length == 0) {
                    setTarget(state.position);
                }else setTarget(state.position,time[0]);
                break;
            case Slot2:
                if(time.length == 0) {
                    setTarget(state.position);
                }else setTarget(state.position,time[0]);
                break;
            case Slot3:
                if(time.length == 0) {
                    setTarget(state.position);
                }else setTarget(state.position,time[0]);
                break;
            case None:
                break;
        }
    }
    public void emptySlot(){
        if (mixerColors[0] == Constants.Color.None) {
            setState(States.Slot1);
        } else if (mixerColors[1] == Constants.Color.None) {
            setState(States.Slot2);
        } else if (mixerColors[2] == Constants.Color.None) {
            setState(States.Slot3);
        }
    }
    public void emptySlot(double time){
        if (mixerColors[0] == Constants.Color.None) {
            setState(States.Slot1 , time);
        } else if (mixerColors[1] == Constants.Color.None) {
            setState(States.Slot2 , time);
        } else if (mixerColors[2] == Constants.Color.None) {
            setState(States.Slot3 , time);
        }
    }
    public Constants.Color getColorFromSlot(int slotNumber){
        switch (slotNumber){
            case 1:
                return mixerColors[0];
            case 2:
                return mixerColors[1];
            case 3:
                return mixerColors[2];
            default:
                return null;
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
        mixerServo.setPosition(target , time);
    }
    @Override
    public boolean atTarget() {
        return mixerServo.atTarget();
    }
    @Override
    public double getTarget() {
        return target;
    }
    public States getState(){
        return state;
    }

    @Override
    public void update() {
        mixerServo.update();
        if(currentColor != None){
            switch (state){
                case Slot1:
                    mixerColors[0] = currentColor;
                    break;
                case Slot2:
                    mixerColors[1] = currentColor;
                    break;
                case Slot3:
                    mixerColors[2] = currentColor;
                    break;
                case None:
                    break;
            }
        }
    }
}
