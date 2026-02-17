package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoggedSubsystem extends SubsystemBase {
   private static final StringPublisher log = NetworkTableInstance.getDefault().getStringTopic("Robot/subsytems/log").publish();
    public void log(String message ) {
    log.set(this.getName()+":"+message);

    }
}
