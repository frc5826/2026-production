package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.FlywheelController;
import frc.robot.math.PID;

import static frc.robot.Constants.Shooter.*;

public class ShootSubsystem extends LoggedSubsystem {
    private FlywheelController controller;
    private PID pid;
    private double goalSpeed;

    public ShootSubsystem () {
        pid = new PID(0.005,0,0, 1, -1,0,()-> getCurrentVelocity());
        controller = new FlywheelController(0.00205,pid,0.12);
        SmartDashboard.putData("pid", pid);
        SmartDashboard.putData("controller", controller);
    }

    @Override
    public void periodic() {
    double output = controller.calculate();
    SmartDashboard.putNumber("speed",getCurrentVelocity());
    }

    public void setGoalSpeed(double goalSpeed) {
        this.goalSpeed = goalSpeed;
        controller.setSetpoint(goalSpeed);

    }
    public double getCurrentVelocity(){
     return 0;//todo
    }
    public Command getShootCommand(double speed){
        Command c = new InstantCommand(()->setGoalSpeed(speed),this).until(this::isAtGoalSpeed);
        return LoggedCommand.logCommand(c);
    }
    private boolean isAtGoalSpeed(){
        return Math.abs(goalSpeed-getCurrentVelocity())<cFlywheelTolerance;
    }
    public Command stopShoot(){
        Command c = new InstantCommand(()->setGoalSpeed(0));
        return LoggedCommand.logCommand(c);
    }
}
