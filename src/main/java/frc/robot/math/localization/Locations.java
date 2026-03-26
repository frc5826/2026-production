package frc.robot.math.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Locations {
    private static double length = 16.54;
    private static double width = 8.07;
    private static boolean isBlue = false;


    private Locations() {}

    public static Pose2d getDepotPose() {
        //0.758, 5.948
        Pose2d pose = new Pose2d(0.758, 5.948, Rotation2d.kZero);
        return flip(pose);
    }

    public static Pose2d getAutoShootPose() {
        Pose2d pose = new Pose2d(2.316, 5.282, Rotation2d.kZero);
        return flip(pose);
    }

    public static Pose2d getLeftAutoStartPos() {
        Pose2d pose = new Pose2d(4.294, 7.409, Rotation2d.k180deg);
        return flip(pose);
    }

    public static Pose2d getRightAutoStartPos() {
        Pose2d pose = new Pose2d(4.391, 0.628, Rotation2d.k180deg);
        return flip(pose);
    }

    public static Pose2d getRightFromMidPose() {
        Pose2d pose = new Pose2d(3.7, 0.7, Rotation2d.k180deg);
        return flip(pose);
    }
    public static Pose2d getMidFromRightPose() {
        Pose2d pose = new Pose2d(6, 0.7, Rotation2d.kZero);
        return flip(pose);
    }
    public static Pose2d getLeftFromMidPose() {
        Pose2d pose = new Pose2d(3.7, 7.2, Rotation2d.k180deg);
        return flip(pose);
    }
    public static Pose2d getMidFromLeftPose() {
        Pose2d pose = new Pose2d(6, 7.2, Rotation2d.kZero);
        return flip(pose);
    }

    public static Pose2d getHubPose() {
        //4.625x, 4.035y
        Pose2d pose = new Pose2d(4.625, 4.035, Rotation2d.kZero);
        return flip(pose);
    }

    public static Pose2d getLeftSideTarget() {

        Pose2d pose = new Pose2d(1.376, 7.219, Rotation2d.kZero);
        return flip(pose);

    }

    public static Pose2d getRightSideTarget() {

        Pose2d pose = new Pose2d(1.376, 0.714, Rotation2d.kZero);
        return flip(pose);

    }

    public static Rectangle2d getFieldZone() {

        Translation2d cornerA = new Translation2d(0, 0);
        Translation2d cornerB = new Translation2d(length, width);

        return new Rectangle2d(cornerA, cornerB);

    }

    public static Rectangle2d getLeftAllianceZone() {

        Translation2d cornerA = new Translation2d(0, 4);
        Translation2d cornerB = new Translation2d(3.975, 8);

        return new Rectangle2d(flip(cornerA), flip(cornerB));
    }

    public static Rectangle2d getRightAllianceZone() {

        Translation2d cornerA = new Translation2d(0, 0);
        Translation2d cornerB = new Translation2d(3.975, 4);

        return new Rectangle2d(flip(cornerA), flip(cornerB));
    }

    public static Rectangle2d getAllianceZone() {
        //Center: , X: 4.616, Y: 8.1
        Translation2d cornerA = new Translation2d(0, 0);
        Translation2d cornerB = new Translation2d(4.616, 8.1);

        return new Rectangle2d(flip(cornerA), flip(cornerB));

    }

    public static Rectangle2d getLeftSideMidZone() {

        Translation2d cornerA = new Translation2d(4.616, 4.050);
        Translation2d cornerB = new Translation2d(11.915, 8.1);

        return new Rectangle2d(flip(cornerA), flip(cornerB));

    }

    public static Rectangle2d getRightSideMidZone() {

        Translation2d cornerA = new Translation2d(4.616, 0);
        Translation2d cornerB = new Translation2d(11.915, 4.05);


        return new Rectangle2d(flip(cornerA), flip(cornerB));

    }

    public static Rectangle2d getLeftAutoZone() {

        Translation2d cornerA = new Translation2d(0, 8);
        Translation2d cornerB = new Translation2d(8.249, 4);

        return new Rectangle2d(flip(cornerA), flip(cornerB));
    }

    public static Rectangle2d getRightAutoZone() {

        Translation2d cornerA = new Translation2d(0, 4);
        Translation2d cornerB = new Translation2d(8.249, 0);

        return new Rectangle2d(flip(cornerA), flip(cornerB));
    }

    public static Pose2d move(Pose2d pose, double x, double y){
        pose = flip(pose);
        pose = new Pose2d(pose.getX()+x, pose.getY()+y, pose.getRotation());
        return flip(pose);
    }

    public static Rotation2d angleTo(Pose2d a, Pose2d b){
        return a.getTranslation().minus(b.getTranslation()).getAngle();
    }

    private static Translation2d flip(Translation2d pose) {
        if (isBlue) {
            return pose;
        }
        double x2 = -pose.getX() + length;
        double y2 = -pose.getY() + width;

        return new Translation2d(x2, y2);
    }

    private static Pose2d flip(Pose2d pose) {
        if (isBlue) {
            return pose;
        }
        double x2 = -pose.getX() + length;
        double y2 = -pose.getY() + width;

        return new Pose2d(x2, y2, pose.getRotation().plus(Rotation2d.k180deg));
    }

    public static void setAlliance() {
        isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    public static boolean getIsBlue() {
        return isBlue;
    }
}