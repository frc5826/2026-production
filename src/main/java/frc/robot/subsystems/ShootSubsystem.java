package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.FlywheelController;
import frc.robot.math.PID;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Shooter.*;

public class ShootSubsystem extends LoggedSubsystem {
    private FlywheelController controller;
    private PID pid;
    private double goalSpeed;
    private SparkFlex motor1;
    private SparkFlex motor2;
    private DigitalInput beamBreak;
    private boolean stop = true;
    private Timer debouncer;

    public ShootSubsystem() {
        pid = new PID(cFlywheelPID, 10, -0.2, 0, () -> getCurrentVelocity());
        controller = new FlywheelController(cV, pid, cS);
        beamBreak = new DigitalInput(0);
        debouncer = new Timer();

        SmartDashboard.putData("5826/shoot/pid", pid);
        SmartDashboard.putData("5826/shoot/controller", controller);
        SmartDashboard.putData("5826/shoot/beamBreak", beamBreak);

        motor1 = new SparkFlex(cMotorIDShooter1, SparkLowLevel.MotorType.kBrushless);
        motor2 = new SparkFlex(cMotorIDShooter2, SparkLowLevel.MotorType.kBrushless);
        SparkFlexConfig config = (SparkFlexConfig) new SparkFlexConfig().closedLoopRampRate(0.2).smartCurrentLimit(80);
        motor1.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(config.follow(motor1, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        double output = controller.calculate();
        SmartDashboard.putNumber("5826/shoot/speed🏃‍♂️", getCurrentVelocity());
        if (stop) {
            motor1.setVoltage(cS);
        } else motor1.setVoltage(output);
        if(beamBreak.get()){
            debouncer.restart();
        }
    }

    public void setGoalSpeed(double goalSpeed) {
        this.goalSpeed = goalSpeed;
        controller.setSetpoint(goalSpeed);
    }

    public double getCurrentVelocity() {
        return (motor1.getEncoder().getVelocity() + motor2.getEncoder().getVelocity()) / 2;
    }

    public boolean isAtGoalSpeed() {
        return Math.abs(goalSpeed - getCurrentVelocity()) < cFlywheelTolerance;
    }


    public boolean isDoneShooting() {
        return debouncer.hasElapsed(1);

    }

    public Command getShootCommand(double speed) {
        Command c = new InstantCommand(() -> {
            setGoalSpeed(speed);
            stop = false;
        }, this).until(this::isAtGoalSpeed);
        return LoggedCommand.logCommand(c, "Shoot Speed Command");
    }

    public Command getShootCommand(DoubleSupplier distanceSupplier, boolean repeat) {
        Command c = new RunCommand(() -> {
            stop = false;
            setGoalSpeed(getRPMFromDistance(distanceSupplier.getAsDouble()));
        }, this).until(() -> !repeat && isAtGoalSpeed());
        return LoggedCommand.logCommand(c, " Shoot Command");
    }

    private double getRPMFromDistance(double distance) {
        double x = distance + 0.60;
        return -102.62602 * Math.pow(x, 4) + 1659.2906 * Math.pow(x, 3) - 9494.24078 * Math.pow(x, 2) + 23465.129 * x - 18598.1388;//todo
    }

    public void setGoalDistance(double distance) {
        double speed = getRPMFromDistance(distance);
        setGoalSpeed(speed);
    }

    public Command getStopCommand() {
        Command c = new InstantCommand(() -> {
            setGoalSpeed(0);
            stop = true;
        });
        return LoggedCommand.logCommand(c, "Stop Command");
    }

    public void stopShoot() {
        setGoalSpeed(1000);
        stop = true;
    }

}

