package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {

    private SwerveDriveWrapper realSwerve;
    private FakeSwerve fakeSwerve;
    private PS5Controller ps5;

    Field2d field = new Field2d();
FieldSim fieldSim = new FieldSim();

    @Override
    public void robotInit() {

          SmartDashboard.putData("Field", field);

    fieldSim.pieces.add(
        new GamePiece(
            new Translation2d(8.0, 4.0),
            0.18,
            3.0
        )
    );
        
    Logger.recordMetadata("ProjectName", "SwerveSim");
    Logger.recordMetadata("BuildType", "Sim");

    Logger.addDataReceiver(new NT4Publisher()); // AdvantageScope LIVE
    Logger.start(); // ←←← SEM ISSO NADA FUNCIONA

    ps5 = new PS5Controller(0);

    if (RobotBase.isSimulation()) {
        fakeSwerve = new FakeSwerve();
        System.out.println(" FakeSwerve ativo (simulação)");
    } else {
        realSwerve = new SwerveDriveWrapper();
        System.out.println(" YAGSL ativo (robo real)");
    }
}

    @Override
    public void teleopPeriodic() {

        double x = -applyDeadband(ps5.getLeftY()); 
        double y = -applyDeadband(ps5.getLeftX()); 
        double rot = -applyDeadband(ps5.getRightX());


        SwerveModuleState[] states;

        if (fakeSwerve != null) {
            fakeSwerve.drive(new Translation2d(x, y), rot, true, true);
            states = fakeSwerve.getModuleStates();
        } else {
            realSwerve.drive(new Translation2d(x, y), rot, true, true);
            states = realSwerve.getModuleStates();
        }

        if (states == null || states.length < 4) {
            return; 
        }

        double[] log = new double[8];
        for (int i = 0; i < 4; i++) {
            log[i * 2]     = states[i].speedMetersPerSecond;
            log[i * 2 + 1] = states[i].angle.getRadians();
        }
        ChassisSpeeds cs = fakeSwerve.getChassisSpeeds();

       Logger.recordOutput("MapleSim/Commanded/Vx", cs.vxMetersPerSecond);
       Logger.recordOutput("MapleSim/Commanded/Vy", cs.vyMetersPerSecond);
       Logger.recordOutput("MapleSim/Commanded/Omega", cs.omegaRadiansPerSecond);

        Logger.recordOutput("Swerve/ModuleStates", log);
        Logger.recordOutput(
    "Swerve/HeadingRad",
    fakeSwerve.getHeading().getRadians());
        Logger.recordOutput("Swerve/Pose2d", fakeSwerve.getPose());

        double dt = fakeSwerve.getDt();

fieldSim.update(
    fakeSwerve.getPose(),
    fakeSwerve.getChassisSpeeds(),
    dt
);

// robô
field.setRobotPose(fakeSwerve.getPose());

// game piece
int i = 0;
for (GamePiece p : fieldSim.pieces) {
    field.getObject("Piece" + i++)
         .setPose(new Pose2d(p.position, new Rotation2d()));
}

    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : value;
    }
}
