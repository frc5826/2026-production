package frc.robot.math.localization;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Locations {
    private static double length = 16.54;
    private static double width = 8.07;
    private static boolean isBlue = false; //todo


    private Locations () {

    }
    public static Pose2d getHubPose () {
        //4.625x, 4.035y
        Pose2d pose = new Pose2d(4.625,4.035, Rotation2d.kZero );
        return flip(pose);
    }
    private static Pose2d flip (Pose2d pose) {
        if (isBlue) {
            return pose;
        }
        double x2 = -pose.getX() + length;
        double y2 = -pose.getY() + width;

        return new Pose2d(x2,y2,pose.getRotation().plus(Rotation2d.k180deg));
    }

}
