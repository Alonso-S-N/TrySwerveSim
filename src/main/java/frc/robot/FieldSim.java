package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.Obstacle;

public class FieldSim {

    public static final double FIELD_X = 16.54;
    public static final double FIELD_Y = 8.21;

    public static final double ROBOT_RADIUS = 0.45;
    public static final double ROBOT_MASS = 50.0;

    private static final double FLOOR_FRICTION = 1.8;
    private static final double WALL_RESTITUTION = 0.2;

    public final List<GamePiece> pieces = new ArrayList<>();

    public void update(
        Pose2d robotPose,
        ChassisSpeeds robotSpeeds,
        double dt
    ) {
        for (GamePiece p : pieces) {
            handleRobotCollision(robotPose, robotSpeeds, p);
            handleWallCollision(p);
            integratePiece(p, dt);
        }
    }

    /* ---------------- ROBÔ ↔ OBJETO ---------------- */

    private void handleRobotCollision(
        Pose2d robotPose,
        ChassisSpeeds robotSpeeds,
        GamePiece piece
    ) {
        Translation2d robotPos = robotPose.getTranslation();

        Translation2d delta = piece.position.minus(robotPos);
        double dist = delta.getNorm();
        double minDist = ROBOT_RADIUS + piece.radius;

        if (dist >= minDist || dist < 1e-6) return;

        Translation2d normal = unit(delta);

        Translation2d robotVel = new Translation2d(
            robotSpeeds.vxMetersPerSecond,
            robotSpeeds.vyMetersPerSecond
        );

        double relativeSpeed = dot(robotVel, normal);
        if (relativeSpeed <= 0.0) return;

        double impulse =
            (2.0 * ROBOT_MASS * relativeSpeed)
            / (ROBOT_MASS + piece.mass);

        piece.velocity = piece.velocity.plus(
            normal.times(impulse / piece.mass)
        );
    }

    /* ---------------- OBJETO ↔ PAREDE ---------------- */

    private void handleWallCollision(GamePiece p) {
        double x = p.position.getX();
        double y = p.position.getY();

        double vx = p.velocity.getX();
        double vy = p.velocity.getY();

        if (x - p.radius < 0.0) {
            x = p.radius;
            vx = -vx * WALL_RESTITUTION;
        } else if (x + p.radius > FIELD_X) {
            x = FIELD_X - p.radius;
            vx = -vx * WALL_RESTITUTION;
        }

        if (y - p.radius < 0.0) {
            y = p.radius;
            vy = -vy * WALL_RESTITUTION;
        } else if (y + p.radius > FIELD_Y) {
            y = FIELD_Y - p.radius;
            vy = -vy * WALL_RESTITUTION;
        }

        p.position = new Translation2d(x, y);
        p.velocity = new Translation2d(vx, vy);
    }

    /* ---------------- INTEGRAÇÃO ---------------- */

    private void integratePiece(GamePiece p, double dt) {
        if (dt <= 0.0) return;

        p.position = p.position.plus(p.velocity.times(dt));

        p.velocity = p.velocity.times(
            Math.exp(-FLOOR_FRICTION * dt)
        );
    }

    /* ---------------- HELPERS MATEMÁTICOS ---------------- */

    private static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    private static Translation2d unit(Translation2d v) {
        double norm = Math.hypot(v.getX(), v.getY());
        if (norm < 1e-6) {
            return new Translation2d();
        }
        return new Translation2d(
            v.getX() / norm,
            v.getY() / norm
        );
    }
    
}

