package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Util;

public class Rect {
    private double x, y, width, height;

    public Rect(double x, double y, double width, double height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }

    public Rect() {
        this.x = 0;
        this.y = 0;
        this.width = 0;
        this.height = 0;
    }

    /**
     * Constructs a rectangle from two points on the blue side and flips the points if needed.
     */
    public Rect(Pose2d p1, Pose2d p2) {
        p1 = Util.flipIfNeeded(p1);
        p2 = Util.flipIfNeeded(p2);
        x = (double) (p1.getX() < p2.getX() ? p1.getX() : p2.getX());
        y = (double) (p1.getY() < p2.getY() ? p1.getY() : p2.getY());
        width = (double) (p1.getX() > p2.getX() ? p1.getX() : p2.getX()) - x;
        height = (double) (p1.getY() > p2.getY() ? p1.getY() : p2.getY()) - y;
    }

    public boolean contains(Pose2d p) {
        return x <= p.getX() && p.getX() < x + width && y <= p.getY() && p.getY() < y + height;
    }
}
