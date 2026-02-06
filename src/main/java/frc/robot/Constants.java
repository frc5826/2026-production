package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

public class Constants {


    public static class Shooter {
        public static final double cFlywheelTolerance = 50;//todo
        public static final PIDConstants cFlywheelPID = new PIDConstants(0.0007, 0, 0);
        public static final double cV = 0.0018;
        public static final double cS = 0.14;
        public static final int cMotorIDShooter1 = 1;
        public static final int cMotorIDShooter2 = 22;
    }

    public static class Hood {
        public static final int cMotorID = 0;
        public static final int cAbsoluteEncodeID = 0;
        public static final int cEncoderIDA = 0;
        public static final int cEncoderIDB = 0;
        public static final double cAngleOffset = 0;
        public static final double cConversionFactor = 0;
    }

    public static class Intake {
        public static final int cMotorIDIntake1 = 5;
        public static final double cArmMotorSpeed = 0.5;
    }
    //TODO Find SparkMax ID's
    public static class Index {
        public static final int cMotorIDIndex1 = 1;
        public static final int cMotorIDIndex2 = 2;
    }
}
