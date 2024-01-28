package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class Rect2d {
    private final Pose2d bottomLeftCorner;
    private final Pose2d topRightCorner;

    public Rect2d(Pose2d bottomLeftCorner, Pose2d topRightCorner) {
        this.bottomLeftCorner = bottomLeftCorner;
        this.topRightCorner = topRightCorner;
    }

    public boolean contains(Pose2d p) {
        return (
                p.getX() > bottomLeftCorner.getX() &&
                        p.getY() > bottomLeftCorner.getY() &&
                        p.getX() < topRightCorner.getX() &&
                        p.getY() < topRightCorner.getY()
        );
    }
}
