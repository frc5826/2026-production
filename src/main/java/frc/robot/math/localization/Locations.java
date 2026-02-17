package frc.robot.math.localization;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Locations {
    private static double length = 16.54;
    private static double width = 8.07;
    private static boolean isBlue = false;


    private Locations () {

    }
    public static Pose2d getDepotPose () {
        //0.758, 5.948
        Pose2d pose = new Pose2d(0.758, 5.948, Rotation2d.kZero);
        return flip(pose);
    }
    public static Pose2d getHubPose () {
        //4.625x, 4.035y
        Pose2d pose = new Pose2d(4.625,4.035, Rotation2d.kZero );
        return flip(pose);
    }
    public static Pose2d getLeftSideTarget(){

        Pose2d pose = new Pose2d(4.625,6.075, Rotation2d.kZero );
        return flip(pose);

    }
    public static Pose2d getRightSideTarget(){

        Pose2d pose = new Pose2d(4.625,2.025, Rotation2d.kZero );
        return flip(pose);

    }
    public static Rectangle2d getLeftAllianceZonePose(){

        Translation2d cornerA = new Translation2d(0, 4);
        Translation2d cornerB = new Translation2d(3.975, 8);

        return new Rectangle2d(flip(cornerA), flip(cornerB));
    }
    public static Rectangle2d getRightAllianceZonePose(){

        Translation2d cornerA = new Translation2d(0, 0);
        Translation2d cornerB = new Translation2d(3.975, 4);

        return new Rectangle2d(flip(cornerA), flip(cornerB));
    }
    public static Rectangle2d getAllianceZonePose(){
        //Center: , X: 4.616, Y: 8.1
        Translation2d cornerA = new Translation2d(0, 0);
        Translation2d cornerB = new Translation2d(4.616, 8.1);

        return new Rectangle2d(flip(cornerA) , flip(cornerB));

    }
    public static Rectangle2d getLeftSideMidPose(){

        Translation2d cornerA = new Translation2d(4.616, 4.050);
        Translation2d cornerB = new Translation2d(11.915, 8.1);

        return new Rectangle2d(flip(cornerA), flip(cornerB));

    }
    public static Rectangle2d getRightSideMidPose(){

        Translation2d cornerA = new Translation2d(4.616, 0);
        Translation2d cornerB = new Translation2d(11.915, 4.05);


        return new Rectangle2d(flip(cornerA), flip(cornerB));

    }
    private static Translation2d flip (Translation2d pose) {
        if (isBlue) {
            return pose;
        }
        double x2 = -pose.getX() + length;
        double y2 = -pose.getY() + width;

        return new Translation2d(x2,y2);
    }

    private static Pose2d flip (Pose2d pose) {
        if (isBlue) {
            return pose;
        }
        double x2 = -pose.getX() + length;
        double y2 = -pose.getY() + width;

        return new Pose2d(x2,y2,pose.getRotation().plus(Rotation2d.k180deg));
    }

    public static void setAlliance() {
        isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

}