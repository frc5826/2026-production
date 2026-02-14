package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

public class Constants {


    public static class Shooter {
        public static final double cFlywheelTolerance = 50;//todo
        public static final PIDConstants cFlywheelPID = new PIDConstants(0.0007, 0, 0);
        public static final double cV = 0.0018;
        public static final double cS = 0.14;
        public static final int cMotorIDShooter1 = 9;
        public static final int cMotorIDShooter2 = 10;

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
        public static final double cArmMotorSpeed = 0.2;
        public static final int cArmMotor = 7;
        public static final int cArmMotorFollower = 6;
        public static final double cSpeed = 0.5;
    }
    public static class Index {
        public static final int cInnerIndex = 15;
        public static final int cOuterIndex = 8;
        public static final double cIndexerSpeed = 1;
        public static final double cConveyorSpeed = 1;

    }
    public static class Climb{
        public static final int cClimber = 17;
    }
    /*
    1-4 swerve drive driver
    5 intake roller
    6-7 intake arm
    8 conveyor
    9-10 shooter
    11-14 swerve drive angle
    15 inner indexer
    16 hood
    17 climb
     */
}
