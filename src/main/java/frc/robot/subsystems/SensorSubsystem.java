package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

import java.util.function.BooleanSupplier;

public class SensorSubsystem extends LoggedSubsystem {

    private DigitalInput beamBreak;

    public SensorSubsystem() {
        beamBreak = new DigitalInput(1); //TODO Replace Placeholder
    }
    public BooleanSupplier getBeamBreak() {
        return () -> beamBreak.get();
    }
}