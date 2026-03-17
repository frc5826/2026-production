package frc.robot.math;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.cHubBuffer;

public class HubWidget implements Sendable {

    String[] ColorCodes = new String[2];
    String AUTONOMOUS = "AUTONOMOUS";
    String TRANSITION = "TRANSITION";
    String FIRST_SHIFT = "1st SHIFT";
    String SECOND_SHIFT = "2nd SHIFT";
    String THIRD_SHIFT = "3rd SHIFT";
    String FOURTH_SHIFT = "4th SHIFT";
    String END_GAME = "END GAME";



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

    public String getShift() {
        if(DriverStation.isAutonomous())
            return AUTONOMOUS;
        double matchTime = DriverStation.getMatchTime();
        if(matchTime <= 30) {
            return END_GAME;
        } else if (matchTime <= 55) {
            return FOURTH_SHIFT;
        } else if (matchTime <= 80) {
            return THIRD_SHIFT;
        } else if (matchTime <= 105) {
            return SECOND_SHIFT;
        } else if (matchTime <= 120) {
            return FIRST_SHIFT;
        } else
            return TRANSITION;
    }

    public String[] getColor(HubState state) {
        switch (state) {
            case ACTIVE -> {
                ColorCodes[0] = Color.kGreen.toHexString();
                ColorCodes[1] = Color.kGreen.toHexString();
            }
            case INACTIVE -> {
                ColorCodes[0] = Color.kRed.toHexString();
                ColorCodes[1] = Color.kRed.toHexString();
            }
            case UNKNOWN -> {
                ColorCodes[0] = Color.kOrange.toHexString();
                ColorCodes[1] = Color.kOrange.toHexString();
            }
            case CHANGING_TO_ACTIVE -> {
                ColorCodes[0] = Color.kRed.toHexString();
                ColorCodes[1] = Color.kGreen.toHexString();
            }
            case CHANGING_TO_INACTIVE -> {
                ColorCodes[0] = Color.kGreen.toHexString();
                ColorCodes[1] = Color.kRed.toHexString();
            }
        }
        return ColorCodes;
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
        builder.addStringArrayProperty("Color", ()-> getColor(getHubState()),null);
        builder.addStringProperty("Shift", ()-> getShift(), null);
        builder.addDoubleProperty("Match Time", ()-> DriverStation.getMatchTime(), null);

    }


}
