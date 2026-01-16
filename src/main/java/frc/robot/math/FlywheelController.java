package frc.robot.math;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;

public class FlywheelController implements NTSendable {
    private double setpoint;
    private double v,s;
    private PID pid;


    public FlywheelController(double v, PID pid, double s){
     this.v = v;
     this.pid = pid;
     this.s = s;
    }
    public void setSetpoint(double setpoint){
        this.setpoint=setpoint;
        pid.setGoal(setpoint);
    }
    public double calculate() {
        return v*setpoint+s*Math.signum(setpoint)+pid.calculate();
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.addDoubleProperty("s", this::getS, this::setS);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
        builder.addDoubleProperty("v", this::getv, this::setv);
    }
    private void setv (double v) {
        this.v = v;

    }

    private double getv (){
        return v;

    }

    private double getSetpoint (){
        return setpoint;

    }
    private double getS(){
        return s;
    }
    private void setS(double s){
        this.s = s;
    }
}

