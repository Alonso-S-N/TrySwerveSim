package frc.robot;

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
import frc.SwerveBase;
import frc.robot.SwerveDriveWrapper;

public class FakeSwerve implements SwerveBase{

    private static final double MAX_SPEED = 4.0; // m/s
    private static final double MAX_ACCEL = 3.0; // m/s²
    private static final double MAX_ALPHA = 6.0; // rad/s²
    private static final double TRACK_WIDTH = 0.6;
    private static final double WHEEL_BASE  = 0.6;

    private ChassisSpeeds commanded = new ChassisSpeeds();

    private static final double TAU_V = 0.3;
    private static final double TAU_W = 0.25;
 

    private double mapleHeadingRad = 0.0;

    private Pose2d maplePose = new Pose2d();
    private boolean useMapleSim = true;

    

    

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

    @Override
public void drive(Translation2d translation, double rot, boolean fieldCentric, boolean openLoop) {

    ChassisSpeeds speeds = fieldCentric
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX() * MAX_SPEED,
            translation.getY() * MAX_SPEED,
            rot,
            getRotation2d() // ← AGORA 100% MapleSim
        )
        : new ChassisSpeeds(
            translation.getX() * MAX_SPEED,
            translation.getY() * MAX_SPEED,
            rot
        );

    commanded = speeds;

    // apenas visualização
    states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);
}



     @Override
    public SwerveModuleState[] getModuleStates() {
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return commanded;
    }

    public SwerveModulePosition[] getModulePositions() {
       return modulePositions;
    }

      public Rotation2d getRotation2d() {
        return maplePose.getRotation();
    }



   // public Rotation2d getHeading() {
     //   return getRotation2d();
   // }


    public Pose2d getPose() {
        return useMapleSim ? maplePose : odometry.getPoseMeters();
    }

    
    public void updateFromMapleSim(double x, double y, double thetaRad) {
        maplePose = new Pose2d(
            x,
            y,
            new Rotation2d(thetaRad)
        );
    
        headingRad = thetaRad; // sincroniza gyro
    }

    public ChassisSpeeds getCommandedSpeeds() {
        return commanded;
    }
    

    
    
    
}
