package frc.robot.math;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.cHubBuffer;

public class HubWidget implements Sendable {

    double counter;

    public HubState getHubState() {
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        DriverStation.Alliance alliance = DriverStation.getAlliance().get();
        DriverStation.Alliance firstShift;

        if (DriverStation.isAutonomous() || matchTime > 130 + cHubBuffer || matchTime < 30) {
            return HubState.ACTIVE;
        } else if (gameData.length() > 0) {
            if (gameData.charAt(0) == 'R') {
                firstShift = DriverStation.Alliance.Red;
            } else {
                firstShift = DriverStation.Alliance.Blue;
            }
            if (firstShift.equals(alliance)) {
                if (matchTime < 130 + cHubBuffer && matchTime > 105 + cHubBuffer || matchTime < 80 && matchTime > 55 + cHubBuffer) {
                    return HubState.ACTIVE;
                } else if (matchTime < 105 && matchTime > 80 + cHubBuffer || matchTime < 55 && matchTime > 30 + cHubBuffer) {
                    return HubState.INACTIVE;
                } else if (matchTime < 105 + cHubBuffer && matchTime > 105 || matchTime < 55 + cHubBuffer && matchTime > 55) {
                    return HubState.CHANGING_TO_INACTIVE;
                } else if (matchTime < 80 + cHubBuffer && matchTime > 80 || matchTime < 30 + cHubBuffer && matchTime > 30) {
                    return HubState.CHANGING_TO_ACTIVE;
                }
            } else {
                if (matchTime < 130 && matchTime > 105 + cHubBuffer || matchTime < 80 && matchTime > 55 + cHubBuffer) {
                    return HubState.INACTIVE;
                } else if (matchTime < 105 && matchTime > 80 + cHubBuffer || matchTime < 55 && matchTime > 30 + cHubBuffer) {
                    return HubState.ACTIVE;
                } else if (matchTime < 105 + cHubBuffer && matchTime > 105 || matchTime < 55 + cHubBuffer && matchTime > 55) {
                    return HubState.CHANGING_TO_ACTIVE;
                } else if (matchTime < 130 + cHubBuffer&& matchTime> 130||matchTime < 80 + cHubBuffer && matchTime > 80) {
                    return HubState.CHANGING_TO_INACTIVE;
                }
            }
        }
        return HubState.UNKNOWN;
    }

    public Color getColor(HubState state) {
        return switch (state) {
            case ACTIVE -> Color.kGreen;
            case INACTIVE -> Color.kRed;
            case UNKNOWN -> Color.kOrange;
            case CHANGING_TO_ACTIVE -> {
                if (counter++ > 10) {
                    yield Color.kGreen;
                } else if (counter > 20) {
                    counter = 0;
                }
                yield Color.kYellow;
            }
            case CHANGING_TO_INACTIVE -> {
                if (counter++ > 10) {
                    yield Color.kRed;
                } else if (counter > 20) {
                    counter = 0;
                }
                yield Color.kYellow;
            }
        };
    }

    public enum HubState {
        UNKNOWN,
        ACTIVE,
        INACTIVE,
        CHANGING_TO_ACTIVE,
        CHANGING_TO_INACTIVE
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Color");
        builder.addStringProperty("Color", ()-> getColor(getHubState()).toHexString(),null);

    }


}
