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

    public ShootSubsystem () {
        pid = new PID(0.0007,0,0, 10, -0.2,0,()-> getCurrentVelocity());
        controller = new FlywheelController(0.0018,pid,0.14);
        SmartDashboard.putData("pid", pid);
        SmartDashboard.putData("controller", controller);
        motor1 = new SparkFlex(1, SparkLowLevel.MotorType.kBrushless);
        motor2 = new SparkFlex(22, SparkLowLevel.MotorType.kBrushless);
        motor1.configure(new SparkFlexConfig().inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(new SparkFlexConfig().follow(motor1,true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        double output = controller.calculate();
        SmartDashboard.putNumber("speed",getCurrentVelocity());
        motor1.setVoltage(output);
    }

    public void setGoalSpeed(double goalSpeed) {
        this.goalSpeed = goalSpeed;
        controller.setSetpoint(goalSpeed);
    }

    public double getCurrentVelocity(){
        return (motor1.getEncoder().getVelocity()+motor2.getEncoder().getVelocity())/2;
    }

    private boolean isAtGoalSpeed(){
        return Math.abs(goalSpeed-getCurrentVelocity())<cFlywheelTolerance;
    }



    public Command getShootCommand(double speed){
        Command c = new InstantCommand(()->setGoalSpeed(speed),this).until(this::isAtGoalSpeed);
        return LoggedCommand.logCommand(c);
    }
    public Command getShootCommand(DoubleSupplier distanceSupplier){
        Command c = new InstantCommand(()->{
            setGoalSpeed(getRPMFromDistance(distanceSupplier.getAsDouble()));
        },this).until(this::isAtGoalSpeed);
        return LoggedCommand.logCommand(c);
    }

    private  double getRPMFromDistance(double distance) {
        return 0;//todo
    }

    public Command stopShoot(){
        Command c = new InstantCommand(()->setGoalSpeed(0));
        return LoggedCommand.logCommand(c);
    }
}

