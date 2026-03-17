package frc.robot.subsystems;


import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
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
    private Timer debouncer, shootBoost;
    private CANrange canRange;


    public ShootSubsystem() {
        pid = new PID(cFlywheelPID, 10, -0.2, 0, () -> getCurrentVelocity());
        controller = new FlywheelController(cV, pid, cS);
        beamBreak = new DigitalInput(0);
        debouncer = new Timer();
        shootBoost = new Timer();
        canRange = new CANrange(cCANRangeID, cCANBusName);
        var canRangeConfig = new ProximityParamsConfigs().withProximityThreshold(0.5)
                .withMinSignalStrengthForValidMeasurement(1);
        canRange.getConfigurator().apply(canRangeConfig);

        SmartDashboard.putData("5826/shoot/pid", pid);
        SmartDashboard.putData("5826/shoot/controller", controller);
        SmartDashboard.putData("5826/shoot/beamBreak", beamBreak);
        Preferences.initDouble("distanceOffset", 0);

        motor1 = new SparkFlex(cMotorIDShooter1, SparkLowLevel.MotorType.kBrushless);
        motor2 = new SparkFlex(cMotorIDShooter2, SparkLowLevel.MotorType.kBrushless);
        SparkFlexConfig config = (SparkFlexConfig) new SparkFlexConfig().closedLoopRampRate(0.2).smartCurrentLimit(40);
        motor1.configure(config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(config.follow(motor1, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public boolean getCANRange() {
        return canRange.getIsDetected().getValue();
    }

    @Override
    public void periodic() {
        double output = controller.calculate();
        SmartDashboard.putNumber("5826/shoot/speed🏃‍♂️", getCurrentVelocity());
        SmartDashboard.putBoolean("5826/shoot/isAtGoalSpeed", isAtGoalSpeed());
        SmartDashboard.putBoolean("5826/shoot/canRange", getCANRange());
        if (stop) {
            motor1.setVoltage(cS + cV * 2000);
        } else motor1.setVoltage(output);
        if (getCANRange()) {
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
        Command c = new RunCommand(() -> {
            setGoalSpeed(speed);
            stop = false;
        }, this).until(this::isAtGoalSpeed);
        return LoggedCommand.logCommand(c, "Shoot Speed Command");
    }

    public Command getShootCommand(DoubleSupplier distanceSupplier, boolean repeat) {
        Command c = new InstantCommand(() -> shootBoost.restart()).andThen(
                new RunCommand(() -> {
                    stop = false;
                    if (shootBoost.get() < 1) {
                        setGoalSpeed(getRPMFromDistance(distanceSupplier.getAsDouble()) * 1.05);
                    } else {
                        setGoalSpeed(getRPMFromDistance(distanceSupplier.getAsDouble()));
                    }
                }, this)).until(() -> !repeat && isAtGoalSpeed());
        return LoggedCommand.logCommand(c, " Shoot Command");
    }

    private static double getRPMFromDistance(double distance) {
        double x = distance + Preferences.getDouble("distanceOffset", 0);
        return (-448.93378 * Math.pow(distance, 4)) + (5802.46914 * Math.pow(distance, 3)) - (27717.1717 * Math.pow(distance, 2)) + (58471.7172 * distance) - 43400;
        //y=-448.93378x^{4}+5802.46914x^{3}-27717.1717x^{2}+58471.7172x-43400
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
        shootBoost.stop();
        shootBoost.reset();
        stop = true;
    }


}
