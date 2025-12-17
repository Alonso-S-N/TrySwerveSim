package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {

    private SwerveDriveWrapper realSwerve;
    private FakeSwerve fakeSwerve;
    private PS5Controller ps5;

    @Override
    public void robotInit() {

        // 🔥 ISSO É OBRIGATÓRIO 🔥
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
        
            // ===============================
            // 1️⃣ LEITURA DO CONTROLE
            // ===============================
            double xCmd  = -applyDeadband(ps5.getLeftY());
            double yCmd  = -applyDeadband(ps5.getLeftX());
            double rotCmd = -applyDeadband(ps5.getRightX());
        
            // ===============================
            // 2️⃣ DRIVE (COMANDO)
            // ===============================
            if (fakeSwerve != null) {
                fakeSwerve.drive(
                    new Translation2d(xCmd, yCmd),
                    rotCmd,
                    false,
                    true
                );
            } else {
                realSwerve.drive(
                    new Translation2d(xCmd, yCmd),
                    rotCmd,
                    true,
                    true
                );
            }
        
            // ===============================
            // 3️⃣ NETWORKTABLE (MAPLESIM)
            // ===============================
            var table = edu.wpi.first.networktables.NetworkTableInstance
                .getDefault()
                .getTable("MapleSim");
        
            // --- leitura da pose vinda do MapleSim ---
            double x     = table.getEntry("Pose/X").getDouble(0.0);
            double y     = table.getEntry("Pose/Y").getDouble(0.0);
            double theta = table.getEntry("Pose/Theta").getDouble(0.0);
        
            fakeSwerve.updateFromMapleSim(x, y, theta);
        
            // --- envio do comando PARA o MapleSim ---
            ChassisSpeeds cmd = fakeSwerve.getCommandedSpeeds();
        
            table.getEntry("Cmd/Vx").setDouble(cmd.vxMetersPerSecond);
            table.getEntry("Cmd/Vy").setDouble(cmd.vyMetersPerSecond);
            table.getEntry("Cmd/Omega").setDouble(cmd.omegaRadiansPerSecond);
        
            // ===============================
            // 4️⃣ LOG
            // ===============================
            Logger.recordOutput("MapleSim/Cmd/Vx", cmd.vxMetersPerSecond);
            Logger.recordOutput("MapleSim/Cmd/Vy", cmd.vyMetersPerSecond);
            Logger.recordOutput("MapleSim/Cmd/Omega", cmd.omegaRadiansPerSecond);
            Logger.recordOutput("MapleSim/Pose", fakeSwerve.getPose());
        }
        

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : value;
    }
}
