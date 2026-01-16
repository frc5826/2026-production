package frc.robot.math;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;

import java.util.function.DoubleSupplier;

@Logged
public class PID implements NTSendable {
    private double P,I,D;
    double previous_error, integral = 0;
    double setpoint = 0;
    double error;
    private double output;

    private final DoubleSupplier actualSupplier;

    private double min_output = 0;
    private double max_output = 1;
    private double deadband = -1;

    public PID(double P, double I, double D, double max, double min, double deadband, DoubleSupplier actualSupplier){
        this.P = P;
        this.I = I;
        this.D = D;
        this.max_output = max;
        this.min_output = min;
        this.deadband = deadband;
        this.actualSupplier = actualSupplier;
    }

    public PID(PIDConstants pid, double max, double min, double deadband, DoubleSupplier actualSupplier){
        this(pid.kP, pid.kI, pid.kD, max, min, deadband, actualSupplier);
    }

    public double calculate() {
        error = setpoint - actualSupplier.getAsDouble(); // Error = Target - Actual
        this.integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal
        final double derivative = (error - this.previous_error) / .02;
        this.output = P * error + I * this.integral + D * derivative;
        this.previous_error = error;

        return getOutput();
    }

    public void setGoal(double setpoint){
        this.integral = 0;
        this.setpoint = setpoint;
    }

    public double getOutput(){
        if(deadband != -1 && Math.abs(error) < deadband){
            return 0;
        }
        else if(Math.abs(output) > max_output){
            return output > 0 ? max_output : -max_output;
        }
        else if(Math.abs(output) < min_output){
            return output > 0 ? min_output : -min_output;
        }
        else {
            return output;
        }
    }

    public double getGoal() {return setpoint;}

    public double getError() {
        return error;
    }

    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }

    private void setP(double P) {
        this.P = P;
    }

    private void setI(double I) {
        this.I = I;
    }

    private void setD(double D) {
        this.D = D;
    }

    public double getDeadband() { return deadband; }

    @Override
    public String toString() {
        return "PID{" +
                "P=" + P +
                ", I=" + I +
                ", D=" + D +
                ", error=" + error +
                ", output=" + output +
                ", min_output=" + min_output +
                ", max_output=" + max_output +
                ", deadband=" + deadband +
                '}';
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("5826-PID");
        builder.addDoubleProperty("P",this::getP,this::setP);
        builder.addDoubleProperty("I",this::getI,this::setI);
        builder.addDoubleProperty("D",this::getD,this::setD);
        builder.addDoubleProperty("error", this::getError,null);
        builder.addDoubleProperty("goal", this::getGoal, this::setGoal);
        builder.addDoubleProperty("output", this::getOutput, null);
    }
}
