package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.FlywheelController;
import frc.robot.math.PID;

public class ShootSubsystem extends SubsystemBase {
    private SparkMax motorOne;
    private SparkMax motorTwo;
    private FlywheelController controller;
    private PID pid;

    public ShootSubsystem () {
        motorOne = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
        motorTwo = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
        motorOne.configure(new SparkMaxConfig().inverted(false),ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorTwo.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pid = new PID(0.005,0,0, 1, -1,0,()-> motorOne.getEncoder().getVelocity());
        controller = new FlywheelController(0.00205,pid,0.12);
        SmartDashboard.putData("pid", pid);
        SmartDashboard.putData("controller", controller);
    }

    @Override
    public void periodic() {
    double output = controller.calculate();
    motorOne.setVoltage(output);
    motorTwo.setVoltage(output);
    SmartDashboard.putNumber("speed",motorOne.getEncoder().getVelocity());
    }
}
