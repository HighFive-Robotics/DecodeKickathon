package org.firstinspires.ftc.teamcode.Core.Algorithms; // Adjust package name as needed

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Implements a highly optimized PIDF controller for velocity control using the velocity form algorithm.
 * Based on the Rockwell Automation white paper: https://literature.rockwellautomation.com/idc/groups/literature/documents/wp/logix-wp008_-en-p.pdf
 * This controller includes integral anti-windup, derivative filtering via Exponential Moving Average (EMA),
 * and a feedforward term based on the change in the setpoint.
 *
 * Velocity Form Algorithm:
 * CV_n = CV_{n-1} + Kp * ΔE + Ki * E_n * Δt + Kd * (E_n - 2*E_{n-1} + E_{n-2}) / Δt
 * Where:
 * CV_n = Current Output
 * CV_{n-1} = Previous Output (accumulates integral effect)
 * E_n = Current Error (setPoint - measuredValue)
 * E_{n-1} = Previous Error
 * E_{n-2} = Error from two steps ago
 * ΔE = E_n - E_{n-1}
 * Δt = Time difference between updates (period)
 *
 * Feedforward (based on setpoint change):
 * F_change = kF * (setPoint_n - setPoint_{n-1})
 *
 * Final Output Change (ΔCV):
 * ΔCV = Kp*ΔE + Ki*E_n*Δt + Kd*Derivative + kF*ΔSetpoint
 * Output (CV_n) = CV_{n-1} + ΔCV
 */
public class VelocityPID {

    // --- Tunable Coefficients ---
    private double kP, kI, kD, kF;

    // --- Controller State ---
    private double setPoint;         // Target velocity (SP_n)
    private double measuredValue;    // Current velocity (PV_n)
    private double minOutput, maxOutput;

    private double errorVal_p;       // E_n (current error)
    private double prevErrorVal;     // E_{n-1} (previous error)
    private double prevPrevErrorVal; // E_{n-2} (error from two steps ago)
    private double prevOutput;       // CV_{n-1} (previous control variable output, accumulates integral)
    private double prevSetPoint;     // SetPoint_{n-1}

    // --- Tolerances for atSetPoint() ---
    private double errorTolerance_p = 0.05; // Positional error tolerance (actually velocity error here)
    private double errorTolerance_v = Double.POSITIVE_INFINITY; // Velocity error tolerance (derivative of velocity error)

    // --- Timing ---
    private final ElapsedTime timer;
    private double lastTimeStampSeconds = -1.0; // Initialize to indicate first run
    private double periodSeconds = 0.0;         // Δt

    // --- Derivative Filtering (EMA) ---
    private double filterGain = 0.8; // EMA gain (0 <= filterGain < 1). Lower values = more filtering.
    private double lastFilterEstimate = 0.0;
    private double currentFilterEstimate = 0.0;

    // --- Integral Anti-Windup ---
    // Note: In velocity form, the integral is implicitly stored in prevOutput.
    // Anti-windup is applied by preventing prevOutput from changing further
    // if the output is already saturated in the direction the integral term wants to push it.

    /**
     * Base constructor. Initializes with gains, zero setpoint, and zero measured value.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param kf Feedforward gain (applied to setpoint changes).
     */
    public VelocityPID(double kp, double ki, double kd, double kf, ElapsedTime timer) {
        this(kp, ki, kd, kf, 0, 0, timer);
    }

    /**
     * Full constructor.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param kf Feedforward gain (applied to setpoint changes).
     * @param sp Initial setpoint (target velocity).
     * @param pv Initial measured value (current velocity).
     * @param timer ElapsedTime object for calculating time differences.
     */
    public VelocityPID(double kp, double ki, double kd, double kf, double sp, double pv, ElapsedTime timer) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        this.timer = timer;

        setPoint = sp;
        measuredValue = pv;

        minOutput = -1.0;
        maxOutput = 1.0;

        reset(); // Initialize state variables
        errorVal_p = setPoint - measuredValue; // Calculate initial error after reset
    }

    /**
     * Resets the internal state of the controller.
     * Call this when disabling the controller, changing gains significantly, or needing a fresh start.
     */
    public void reset() {
        prevErrorVal = 0;
        prevPrevErrorVal = 0;
        prevOutput = 0; // Resets the integral accumulator in velocity form
        lastTimeStampSeconds = -1.0; // Signal that the next update is the first
        periodSeconds = 0;
        lastFilterEstimate = 0.0;
        currentFilterEstimate = 0.0;
        // Keep current setPoint and measuredValue, but reset history
        prevSetPoint = setPoint;
        // errorVal_p will be recalculated in the next calculate() call or constructor
    }

    /**
     * Sets the velocity error tolerance for use with {@link #atSetPoint()}.
     * @param positionTolerance Velocity error which is tolerable (units must match setpoint/measuredValue).
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the velocity and acceleration error tolerances for use with {@link #atSetPoint()}.
     * @param positionTolerance Velocity error which is tolerable (units must match setpoint/measuredValue).
     * @param velocityTolerance Acceleration error which is tolerable (units/sec).
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        errorTolerance_p = Math.abs(positionTolerance);
        errorTolerance_v = Math.abs(velocityTolerance);
    }

    /**
     * Returns the current setpoint (target velocity).
     * @return The current setpoint.
     */
    public double getSetPoint() {
        return setPoint;
    }

    /**
     * Sets the setpoint (target velocity) for the controller.
     * @param sp The desired setpoint.
     */
    public void setSetPoint(double sp) {
        prevSetPoint = setPoint; // Store previous setpoint for feedforward calculation
        setPoint = sp;
    }

    /**
     * Returns true if the absolute error and absolute velocity error are within the tolerances
     * set by {@link #setTolerance}.
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetPoint() {
        return Math.abs(getPositionError()) <= errorTolerance_p &&
                Math.abs(getVelocityError()) <= errorTolerance_v;
    }

    /**
     * Returns the PIDF coefficients.
     * @return Array containing {kP, kI, kD, kF}.
     */
    public double[] getCoefficients() {
        return new double[]{kP, kI, kD, kF};
    }

    /**
     * Returns the current velocity error (E_n = setPoint - measuredValue).
     * @return The current velocity error.
     */
    public double getPositionError() {
        // Renamed from getPositionError in original VelocityPID for clarity, but keeping signature
        return errorVal_p;
    }

    /**
     * Returns the tolerances {position (velocity), velocity (acceleration)} of the controller.
     * @return Array containing {errorTolerance_p, errorTolerance_v}.
     */
    public double[] getTolerance() {
        return new double[]{errorTolerance_p, errorTolerance_v};
    }

    /**
     * Returns the rate of change of the velocity error (approximated acceleration error).
     * e'(t) ≈ (E_n - E_{n-1}) / period.
     * @return The current velocity error rate (acceleration error). Returns 0 if period is too small.
     */
    public double getVelocityError() {
        if (Math.abs(periodSeconds) > 1E-9) { // Avoid division by zero
            return (errorVal_p - prevErrorVal) / periodSeconds;
        }
        return 0;
    }

    /**
     * Calculates the next output of the PIDF controller using the last measured value.
     * Call {@link #calculate(double)} or {@link #calculate(double, double)} to update the measured value first.
     * @return The calculated control output (e.g., motor power).
     */
    public double calculate() {
        return calculate(measuredValue);
    }

    /**
     * Calculates the next output of the PIDF controller. Updates the setpoint first.
     * @param pv The current measured velocity.
     * @param sp The new target velocity (setpoint).
     * @return The calculated control output.
     */
    public double calculate(double pv, double sp) {
        setSetPoint(sp); // Update the setpoint and store the previous one
        return calculate(pv);
    }

    /**
     * Calculates the control value using the velocity PIDF algorithm with EMA filtering and anti-windup.
     * @param pv The current measurement of the process variable (current velocity).
     * @return The calculated control output, clamped between output bounds.
     */
    public double calculate(double pv) {
        // --- Timing ---
        double currentTimeStamp = timer.seconds();
        if (lastTimeStampSeconds < 0) { // First run detection
            lastTimeStampSeconds = currentTimeStamp;
            measuredValue = pv;
            errorVal_p = setPoint - measuredValue; // Set E_n
            prevErrorVal = errorVal_p;             // Set E_{n-1} for next loop
            prevPrevErrorVal = errorVal_p;         // Set E_{n-2} for next loop
            prevSetPoint = setPoint;
            prevOutput = Range.clip(kF * setPoint, minOutput, maxOutput); // Initial guess using only Feedforward (clamped)
            return prevOutput;
        }
        periodSeconds = currentTimeStamp - lastTimeStampSeconds;
        lastTimeStampSeconds = currentTimeStamp;

        // --- Update Error History ---
        prevPrevErrorVal = prevErrorVal; // E_{n-2} = old E_{n-1}
        prevErrorVal = errorVal_p;     // E_{n-1} = old E_n
        measuredValue = pv;
        errorVal_p = setPoint - measuredValue; // Calculate E_n

        // Handle edge case of zero time delta
        if (Math.abs(periodSeconds) < 1E-9) {
            return prevOutput; // No change if no time passed
        }

        // --- Proportional Term ---
        // P_term = Kp * ΔE = Kp * (E_n - E_{n-1})
        double deltaError = errorVal_p - prevErrorVal;
        double proportionalTerm = kP * deltaError;

        // --- Integral Term ---
        // I_term = Ki * E_n * Δt
        double integralTerm = kI * errorVal_p * periodSeconds;

        // --- Derivative Term (with EMA Filter) ---
        // Raw derivative based on second-order difference: (E_n - 2*E_{n-1} + E_{n-2}) / Δt
        double deltaErrorDerivative = (errorVal_p - 2 * prevErrorVal + prevPrevErrorVal);
        double rawDerivative = deltaErrorDerivative / periodSeconds;

        // Apply EMA filter
        currentFilterEstimate = (filterGain * lastFilterEstimate) + (1 - filterGain) * rawDerivative;
        lastFilterEstimate = currentFilterEstimate; // Store for next iteration
        double derivativeTerm = kD * currentFilterEstimate;

        // --- Feedforward Term ---
        // F_term = kF * ΔSetpoint = kF * (SP_n - SP_{n-1})
        double deltaSetPoint = setPoint - prevSetPoint;
        double feedForwardChange = kF * deltaSetPoint;

        // --- Calculate Output Change (ΔCV) ---
        double deltaOutput = proportionalTerm + integralTerm + derivativeTerm + feedForwardChange;

        // --- Calculate New Output (CV_n) before clamping ---
        double currentOutput = prevOutput + deltaOutput;

        // --- Integral Anti-Windup (Clamping) ---
        // If the output is saturated and the integral term is trying to push it further,
        // limit the change contributed by the integral term.
        if (currentOutput >= maxOutput && integralTerm > 0) {
            // Re-calculate deltaOutput without the full integral contribution,
            // effectively setting integralTerm to saturate exactly at maxOutput
            integralTerm = Math.max(0, maxOutput - (prevOutput + proportionalTerm + derivativeTerm + feedForwardChange));
            deltaOutput = proportionalTerm + integralTerm + derivativeTerm + feedForwardChange;
            currentOutput = prevOutput + deltaOutput; // This should now be <= maxOutput

        } else if (currentOutput <= minOutput && integralTerm < 0) {
            // Re-calculate deltaOutput without the full integral contribution,
            // effectively setting integralTerm to saturate exactly at minOutput
            integralTerm = Math.min(0, minOutput - (prevOutput + proportionalTerm + derivativeTerm + feedForwardChange));
            deltaOutput = proportionalTerm + integralTerm + derivativeTerm + feedForwardChange;
            currentOutput = prevOutput + deltaOutput; // This should now be >= minOutput
        }
        // If not saturated, the original integralTerm is used.

        // --- Apply Output Bounds ---
        currentOutput = Range.clip(currentOutput, minOutput, maxOutput);

        // --- Store State for Next Iteration ---
        prevOutput = currentOutput; // CV_{n-1} for next loop is the clamped current output
        // prevSetPoint was updated in setSetPoint() or calculate(pv, sp)

        return currentOutput;
    }

    /** Sets the PIDF gains. */
    public void setPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        // Consider resetting integral/state if gains change significantly
        // reset(); // Uncomment if a full reset is desired upon gain change
    }

    /**
     * Sets the minimum and maximum output bounds for the controller.
     * @param min The minimum allowable output.
     * @param max The maximum allowable output.
     */
    public void setOutputBounds(double min, double max) {
        if (min >= max) throw new IllegalArgumentException("Minimum output limit cannot be greater than or equal to maximum limit");
        minOutput = min;
        maxOutput = max;
        // Clamp existing output immediately
        prevOutput = Range.clip(prevOutput, minOutput, maxOutput);
    }

    /**
     * Renamed from setIntegrationBounds in the provided VelocityPID for clarity.
     * @deprecated Use {@link #setOutputBounds(double, double)} instead.
     */
    @Deprecated
    public void setIntegrationBounds(double min, double max) {
        setOutputBounds(min, max);
    }


    /**
     * Clears the accumulated output (integral state) of the velocity PID controller.
     * Sets the internal previous output (which holds the integral) to zero.
     */
    public void clearTotalError() {
        prevOutput = 0; // In velocity PID, prevOutput holds the accumulated state
        // Optional: Also reset error history?
        // errorVal_p = setPoint - measuredValue;
        // prevErrorVal = errorVal_p;
        // prevPrevErrorVal = errorVal_p;
    }

    /** Sets the gain for the derivative term's EMA filter (0 <= gain < 1). Lower values filter more. */
    public void setDerivativeFilterGain(double gain) {
        if (gain < 0 || gain >= 1) throw new IllegalArgumentException("Filter gain must be between 0 (inclusive) and 1 (exclusive)");
        filterGain = gain;
    }

    // --- Standard Setters and Getters ---
    public void setP(double kp) { kP = kp; }
    public void setI(double ki) { kI = ki; }
    public void setD(double kd) { kD = kd; }
    public void setF(double kf) { kF = kf; }

    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }
    public double getF() { return kF; }

    /** Returns the time period (in seconds) between the last two updates. */
    public double getPeriod() { return periodSeconds; }
}

