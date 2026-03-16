package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Constants {


    public static class Swerve {
        //TODO
        public static final PathConstraints cSlowPath = new PathConstraints(
                1,
                2,
                3,
                6
        );
        public static final PathConstraints cMidPath = new PathConstraints(
                2,
                3.5,
                4.5,
                8
        );
        public static final PathConstraints cFastPath = new PathConstraints(
                4,
                6,
                6,
                12
        );
        public static final PIDConstants cPathDrivePID = new PIDConstants(1,0,0);
        public static final PIDConstants cPathTurnPID = new PIDConstants(4,0,0);
        public static final ChassisSpeeds cStopped = new ChassisSpeeds();
    }

    public static class Shooter {
        public static final double cFlywheelTolerance = 50;//todo
        public static final PIDConstants cFlywheelPID = new PIDConstants(0.0007, 0, 0);
        public static final double cV = 0.0018;
        public static final double cS = 0.14;
        public static final int cMotorIDShooter1 = 9;
        public static final int cMotorIDShooter2 = 10;
        public static final int cCANRangeID = 20;
        public static final CANBus cCANBusName = CANBus.roboRIO();


    }

    public static class Hood {
        public static final int cMotorID = 16;
        public static final int cAbsoluteEncodeID = 0;
        public static final int cEncoderIDA = 0;
        public static final int cEncoderIDB = 0;
        public static final double cAngleOffset = 0;
        public static final double cConversionFactor = 0;
    }

    public static class Intake {
        public static final int cMotorIDIntake1 = 5;
        public static final double cArmMotorSpeed = 0.4;
        public static final double cShakeSpeed = 1.5;
        public static final int cArmMotor = 7;
        public static final int cArmMotorFollower = 6;
        public static final double cSpeed = 1;
        public static final double intakeP = 0.05;
        public static final double intakeI = 0;
        public static final double intakeD = 0;
        public static final double intakeFF = 0;
        public static final double cMaxIntakeShake = -20;
        public static final double cMinIntakeShake = -1;


    }

    public static class Index {
        public static final int cInnerIndex = 14;
        public static final int cOuterIndex = 8;
        public static final double cIndexerSpeed = -1;
        public static final double cConveyorSpeed = 1;

    }

    public static class Climb {
        public static final int cClimber = 15;
        public static final double cDownPos = 0.3;
        public static final double cStowPos = 0;
        public static final double cUpPos = 1.3;
        public static final double cConfigMultiplier = ((1.0 / 45.0) * (1.124 + 0.25) * Math.PI) / 10.875;
    }

    public static class Vision {
        public static final double cDistanceCutoff = 4; //Meters
    }

    public static final int cHubBuffer = 5;

    /*
    1-4 swerve drive driver
    5 intake roller
    6-7 intake arm
    8 conveyor
    9-10 shooter
    11-13 & 17 swerve drive angle
    14 inner indexer
    16 hood
    15 climb
     */
}
