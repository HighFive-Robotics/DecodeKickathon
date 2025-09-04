package org.firstinspires.ftc.teamcode.Core.Module.Outtake;



import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.teamcode.Core.Hardware.HighMotor;


public class Lift implements HighModuleSimple {

    public static PIDCoefficients liftPIDCoefficients_goingUp = new PIDCoefficients(0.012, 0, 0.00001);
    public static PIDCoefficients liftPIDCoefficients_goingDown = new PIDCoefficients(0.0012, 0, 0.00001);
    public static double gravityGain = 0.00042;
    public static double maxCurrentDrawn = 5;

    public HighMotor leftLiftMotor, rightLiftMotor;
    boolean shouldResetEnc = false;
    private double power = 0;
    private boolean newTarget = false;
    ElapsedTime timerToResetEnc = new ElapsedTime();
    public States state = States.Collect;
    public double currentPosition = 0;
    public enum States {
        Collect,
        HighBasket,
        HighBasketSpecial,
        LowBasket,
        HighChamber,
        LowChamber,
        Reset
    }

    public Lift(HardwareMap hardwareMap){
        leftLiftMotor = new HighMotor(hardwareMap.get(DcMotorEx.class, "ZAZA"), HighMotor.RunMode.Standard, false, true);
        rightLiftMotor = new HighMotor(hardwareMap.get(DcMotorEx.class, "zaza"), HighMotor.RunMode.PID, false, true, true, false);
        rightLiftMotor.setPIDCoefficients(liftPIDCoefficients_goingUp.p, liftPIDCoefficients_goingUp.i, liftPIDCoefficients_goingUp.d, gravityGain, HighMotor.FeedForwardType.Lift);
        rightLiftMotor.setTarget(0);
        rightLiftMotor.setTolerance(20);
    }

    public void setState(States state) {
        this.state = state;
        newTarget = true;
        switch (state) {
            case Collect:
                power = -0.9;
                motorsSetPower(power);
                rightLiftMotor.setRunMode(HighMotor.RunMode.Standard);
                rightLiftMotor.setTarget(0);
                break;
            case Reset:
                power = -1;
                rightLiftMotor.setRunMode(HighMotor.RunMode.Standard);
                motorsSetPower(power);
                break;
        }
    }

    public void resetEnc() {
        motorsSetPower(0);
        rightLiftMotor.setTarget(0);
        leftLiftMotor.resetMotor();
        rightLiftMotor.resetMotor();
        rightLiftMotor.resetPID();
        state = States.Collect;
    }

    public void setTarget(double target) {
        rightLiftMotor.setTarget(target);
    }

    public boolean atTarget() {
        return rightLiftMotor.atTarget();
    }

    public double getTarget() {
        return rightLiftMotor.getTarget();
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public double getPower(double currentPosition) {
        return rightLiftMotor.getPower();
    }

    public States getState() {
        return state;
    }

    @Override
    public void update() {
        currentPosition = rightLiftMotor.getCurrentPosition();
        if (state != States.Reset) {
            if (rightLiftMotor.getCurrentDrawn() * (12.0 / Constants.Globals.voltage) >= (maxCurrentDrawn) && currentPosition <= 20 && rightLiftMotor.getRunMode() == HighMotor.RunMode.Standard) {
                power = 0;
                motorsSetPower(power);
                shouldResetEnc = true;
                timerToResetEnc.reset();
            }
            if (timerToResetEnc.milliseconds() >= 100 && shouldResetEnc && rightLiftMotor.getCurrentDrawn() <= 2) {
                shouldResetEnc = false;
                rightLiftMotor.setPIDCoefficients(liftPIDCoefficients_goingUp.p, liftPIDCoefficients_goingUp.i, liftPIDCoefficients_goingUp.d, gravityGain, HighMotor.FeedForwardType.Lift,1);
                rightLiftMotor.setRunMode(HighMotor.RunMode.PID);
                resetEnc();
            }
            if (newTarget) {
                if (currentPosition > getTarget())
                    rightLiftMotor.setPIDCoefficients(liftPIDCoefficients_goingDown.p, liftPIDCoefficients_goingDown.i, liftPIDCoefficients_goingDown.d, gravityGain, HighMotor.FeedForwardType.Lift,1);
                else
                    rightLiftMotor.setPIDCoefficients(liftPIDCoefficients_goingUp.p, liftPIDCoefficients_goingUp.i, liftPIDCoefficients_goingUp.d, gravityGain, HighMotor.FeedForwardType.Lift,1);
            }
            if (rightLiftMotor.getRunMode() == HighMotor.RunMode.PID) {
                power = rightLiftMotor.getPower();
                motorsSetPower(power);
            }
        }
        leftLiftMotor.update();
        rightLiftMotor.update();
    }

    public void motorsSetPower(double power){
        leftLiftMotor.setPower(power);
        rightLiftMotor.setPower(power);
    }
}
