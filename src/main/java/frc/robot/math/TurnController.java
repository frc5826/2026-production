package frc.robot.math;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;

import java.util.function.DoubleSupplier;

public class TurnController implements NTSendable {
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State goal;
    private double v, s;
    private PID pid;
    private double output;
    private DoubleSupplier currentAngle;
    private double angle;

    public TurnController(double v, double s, double maxVelocity, double maxAcceleration, double p, double i, double d, DoubleSupplier currentAngle) {
        this.pid = new PID(p, i, d, 1, -1, 0.01, () -> angleDifference(currentAngle.getAsDouble(), angle)-setpoint.position);
        this.v = v;
        this.s = s;
        this.currentAngle = currentAngle;
        pid.setGoal(0);

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        setpoint = new TrapezoidProfile.State();
        goal = new TrapezoidProfile.State();
    }

    public void setGoal(double angleGoal, double startVelocity) {
        double startPoint = angleDifference(currentAngle.getAsDouble(), angleGoal);
        angle = angleGoal;
        goal = new TrapezoidProfile.State(0, 0);
        setpoint = new TrapezoidProfile.State(startPoint, startVelocity);
    }

    public double calculate(double deltaTime) {

        setpoint = profile.calculate(deltaTime, setpoint, goal);
        pid.setGoal(setpoint.position);
        double pidOutput = pid.calculate();
        output = setpoint.velocity * v + Math.signum(pidOutput) * s + pidOutput;
        return output;
    }

    private double angleDifference(double A, double B) {
        double difference = B - A;
        if (difference > Math.PI) {
            difference = difference - Math.PI * 2;
        } else if (difference < -Math.PI) {
            difference = difference + Math.PI * 2;
        }
        return difference;
    }

    public boolean isFinished() {
        return Math.abs(angleDifference(currentAngle.getAsDouble(), setpoint.position)) < Math.toRadians(5);
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.addDoubleProperty("V", this::getV, this::setV);
        builder.addDoubleProperty("S", this::getS, this::setS);
        builder.addDoubleProperty("setPoint", this::getSetpoint, null);
        builder.addDoubleProperty("Output", this::getOutput, null);
        builder.addDoubleProperty("Goal", this::getGoal, (x) -> setGoal(x, 0));
        builder.addDoubleProperty("Angle", this::getCurrent, null);
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        pid.initSendable(builder);
    }

    private double getV() {
        return v;
    }

    private void setV(double v) {
        this.v = v;
    }

    private double getS() {
        return s;
    }

    private void setS(double s) {
        this.s = s;
    }

    private double getSetpoint() {
        return setpoint.position;
    }

    private double getOutput() {
        return output;
    }

    private double getGoal() {
        return goal.position;
    }

    private double getCurrent() {
        return currentAngle.getAsDouble();
    }

    private double getVelocity() {
        return setpoint.velocity;
    }
}
