package frc.robot;

import java.util.List;

import org.opencv.dnn.Model;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.Obstacle;
import frc.SwerveBase;
import frc.robot.SwerveDriveWrapper;

public class FakeSwerve implements SwerveBase{

    private static final double MAX_SPEED = 4.0; // m/s
    private static final double MAX_ACCEL = 3.0; // m/s²
    private static final double MAX_ALPHA = 6.0; // rad/s²
    private static final double TRACK_WIDTH = 0.6;
    private static final double WHEEL_BASE  = 0.6;

    private ChassisSpeeds commanded = new ChassisSpeeds();
    private ChassisSpeeds simulated = new ChassisSpeeds();

    private double dtEnc = 0.0;
    private double dtGyro = 0.0;
    private static final double FIELD_X = 17.8;
    private static final double FIELD_Y = 8.21;

    private static final double ROBOT_RADIUS = 0.55;


    private double lastSimTime = Timer.getFPGATimestamp();


    private static final double TAU_V = 0.3;
    private static final double TAU_W = 0.25;
    private double lastEncoderTime = -1;
    private double lastGyroTime = -1;
    private double lastTime = -1;
    

    private final SwerveDriveOdometry odometry;


    private double headingRad = 0.0;
    
    public Pose2d pose = new Pose2d();

    ChassisSpeeds speeds;

    private final SwerveDriveKinematics kinematics;

    private SwerveModuleState[] states =
        new SwerveModuleState[] {
            new SwerveModuleState(0.0, new Rotation2d()),
            new SwerveModuleState(0.0, new Rotation2d()),
            new SwerveModuleState(0.0, new Rotation2d()),
            new SwerveModuleState(0.0, new Rotation2d())
        };

        private SwerveModulePosition[] modulePositions =
        new SwerveModulePosition[] {
            new SwerveModulePosition(0.0,new Rotation2d()),
            new SwerveModulePosition(0.0, new Rotation2d()),
            new SwerveModulePosition(0.0, new Rotation2d()),
            new SwerveModulePosition(0.0, new Rotation2d())
        };

        
    public FakeSwerve() {
        kinematics = new SwerveDriveKinematics(
            new Translation2d( WHEEL_BASE / 2,  TRACK_WIDTH / 2),
            new Translation2d( WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2,  TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        );
        this.odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), getModulePositions());
    }

     public static List<Obstacle> obstacles() {
        return List.of(

            new Obstacle(
                3.80,   // x
                3.5,   // y
                1.60,   // largura
                1.0    // altura
            ),

            new Obstacle(
                12.5,
                3.6,
                1.6,
                1.0
            ),
            new Obstacle(
                8.95,
                4.0,
                0.1,
                0.1
            )
        );
    }

    @Override
public void drive(Translation2d translation, double rot, boolean fieldCentric, boolean openLoop) {

    updateGyro(rot);

    ChassisSpeeds speeds = fieldCentric
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX() * MAX_SPEED,
            translation.getY() * MAX_SPEED,
            rot,
            getRotation2d()
          )
        : new ChassisSpeeds(
            translation.getX() * MAX_SPEED,
            translation.getY() * MAX_SPEED,
            rot
          );
    
          commanded = speeds;
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);

    double now = Timer.getFPGATimestamp();

    if (lastEncoderTime < 0) {
        lastEncoderTime = now;
        lastGyroTime = now;
        lastTime = now;
        return;
    }

double dt = now - lastTime;
lastTime = now;

if (dt <= 0 || dt > 0.1) {
    return;
}

    
    // encoders simulados
    dtEnc = now - lastEncoderTime;
    lastEncoderTime = now;
    
    for (int i = 0; i < 4; i++) {
        modulePositions[i] = new SwerveModulePosition(
            modulePositions[i].distanceMeters
                + states[i].speedMetersPerSecond * dtEnc,
            states[i].angle
        );
    }

    
    simulated = new ChassisSpeeds(
        stepWithAccelLimit(
            simulated.vxMetersPerSecond,
            commanded.vxMetersPerSecond,
            dtEnc,
            MAX_ACCEL
        ),
        stepWithAccelLimit(
            simulated.vyMetersPerSecond,
            commanded.vyMetersPerSecond,
            dtEnc,
            MAX_ACCEL
        ),
        stepWithAccelLimit(
            simulated.omegaRadiansPerSecond,
            commanded.omegaRadiansPerSecond,
            dtEnc,
            MAX_SPEED
        )
    );
    states = kinematics.toSwerveModuleStates(simulated);
    // odometria
    odometry.update(getRotation2d(), modulePositions);
    handleWallCollision();
    handleObstacleCollisions();

}

     @Override
    public SwerveModuleState[] getModuleStates() {
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return simulated;
    }

    public SwerveModulePosition[] getModulePositions() {
       return modulePositions;
    }

    public void update(double omegaRadPerSec) {
        double now = Timer.getFPGATimestamp();
        double dtGyro = now - lastTime;
        lastTime = now;
  
        headingRad += omegaRadPerSec * dtGyro;
      }
      public Rotation2d getRotation2d() {
        return new Rotation2d(headingRad);
    }

    private double stepWithAccelLimit(
    double current,
    double target,
    double dtEnc,
    double maxAccel
) {
    double accel = (target - current) / dtEnc;
    accel = MathUtil.clamp(accel, -maxAccel, maxAccel);
    return current + accel * dtEnc;
}

    public void reset() {
        headingRad = 0.0;
        lastTime = Timer.getFPGATimestamp();
    }
    public Rotation2d getHeading() {
        return getRotation2d();
    }

    public Pose2d getPose() {
        return pose = odometry.getPoseMeters();
    }

    public void updateGyro(double omegaRadPerSec) {
        double now = Timer.getFPGATimestamp();
        dtGyro = now - lastGyroTime;
        lastGyroTime = now;
    
        headingRad += omegaRadPerSec * dtGyro;
    }

    private double firstOrder(double current, double target, double dt, double tau) {
        return current + (target - current) * (dt / tau);
    }
    
    public double getDt() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastSimTime;
        lastSimTime = now;
        return dt;
    }
    private void handleWallCollision() {
        Pose2d pose = odometry.getPoseMeters();
    
        double x = pose.getX();
        double y = pose.getY();
    
        boolean collided = false;
    
        if (x < ROBOT_RADIUS) {
            x = ROBOT_RADIUS;
            simulated = new ChassisSpeeds(
                simulated.vyMetersPerSecond *0.1,
                simulated.vyMetersPerSecond,
                simulated.omegaRadiansPerSecond
            );
            collided = true;
        } else if (x > FIELD_X - ROBOT_RADIUS) {
            x = FIELD_X - ROBOT_RADIUS;
            simulated = new ChassisSpeeds(
                simulated.vyMetersPerSecond *0.1,
                simulated.vyMetersPerSecond,
                simulated.omegaRadiansPerSecond
            );
            collided = true;
        }
    
        if (y < ROBOT_RADIUS) {
            y = ROBOT_RADIUS;
            simulated = new ChassisSpeeds(
                simulated.vxMetersPerSecond,
            simulated.vxMetersPerSecond * 0.1,
                simulated.omegaRadiansPerSecond
            );
            collided = true;
        } else if (y > FIELD_Y - ROBOT_RADIUS) {
            y = FIELD_Y - ROBOT_RADIUS;
            simulated = new ChassisSpeeds(
                simulated.vxMetersPerSecond,
                simulated.vxMetersPerSecond * 0.1,
                simulated.omegaRadiansPerSecond
            );
            collided = true;
        }
    
        if (collided) {
            Pose2d corrected =
                new Pose2d(x, y, pose.getRotation());
    
            odometry.resetPosition(
                getRotation2d(),
                modulePositions,
                corrected
            );
        }
    }

    private void handleObstacleCollisions() {
    Pose2d pose = odometry.getPoseMeters();
    double rx = pose.getX();
    double ry = pose.getY();

    boolean collided = false;

    for (Obstacle o : obstacles()) {

        double closestX = MathUtil.clamp(rx, o.minX, o.maxX);
        double closestY = MathUtil.clamp(ry, o.minY, o.maxY);

        double dx = rx - closestX;
        double dy = ry - closestY;

        double distSq = dx * dx + dy * dy;
        if (distSq >= ROBOT_RADIUS * ROBOT_RADIUS) continue;

        double dist = Math.sqrt(distSq);
        if (dist < 1e-6) continue;

        double nx = dx / dist;
        double ny = dy / dist;

        double penetration = ROBOT_RADIUS - dist;

        rx += nx * penetration;
        ry += ny * penetration;

        // remove velocidade na normal
        simulated = new ChassisSpeeds(
            simulated.vxMetersPerSecond * (1.0 - Math.abs(nx)),
            simulated.vyMetersPerSecond * (1.0 - Math.abs(ny)),
            simulated.omegaRadiansPerSecond
        );

        collided = true;
    }

    if (collided) {
        Pose2d corrected = new Pose2d(rx, ry, pose.getRotation());
        odometry.resetPosition(
            getRotation2d(),
            modulePositions,
            corrected
        );
    }
}

    
    
}
