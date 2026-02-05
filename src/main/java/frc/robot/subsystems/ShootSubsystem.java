package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private boolean stop = true;

    public ShootSubsystem() {
        pid = new PID(cFlywheelPID, 10, -0.2, 0, () -> getCurrentVelocity());
        controller = new FlywheelController(cV, pid, cS);
        SmartDashboard.putData("pid", pid);
        SmartDashboard.putData("controller", controller);
        motor1 = new SparkFlex(cMotorID1, SparkLowLevel.MotorType.kBrushless);
        motor2 = new SparkFlex(cMotorID2, SparkLowLevel.MotorType.kBrushless);
        motor1.configure(new SparkFlexConfig().inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(new SparkFlexConfig().follow(motor1, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        double output = controller.calculate();
        SmartDashboard.putNumber("speed", getCurrentVelocity());
        if (stop) {
            motor1.setVoltage(cS);
        } else motor1.setVoltage(output);
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
        //todo
        return true;

    }

    public Command getShootCommand(double speed) {
        Command c = new InstantCommand(() -> {
            setGoalSpeed(speed);
            stop = false;
        }, this).until(this::isAtGoalSpeed);
        return LoggedCommand.logCommand(c);
    }

    /*    public Command getShootCommand(DoubleSupplier distanceSupplier){
            Command c = new InstantCommand(()->{
                stop = false;
                setGoalSpeed(getRPMFromDistance(distanceSupplier.getAsDouble()));
            },this).until(this::isAtGoalSpeed);
            return LoggedCommand.logCommand(c);
        }
    */
    private double getRPMFromDistance(double distance) {
        double x = distance + 0.60;
        return -102.62602 * Math.pow(x, 4) + 1659.2906 * Math.pow(x, 3) - 9494.24078 * Math.pow(x, 2) + 23465.129 * x - 18598.1388;//todo
    }

    public void setGoalDistance(double distance) {
        double speed = getRPMFromDistance(distance);
        setGoalSpeed(speed);
    }

    //    public Command stopShoot(){
//        Command c = new InstantCommand(()->{setGoalSpeed(0);
//        stop = true;});
//        return LoggedCommand.logCommand(c);
//    }
    public void stopShoot() {
        setGoalSpeed(1000);
        stop = true;
    }

}

