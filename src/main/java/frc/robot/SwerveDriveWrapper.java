package frc.robot;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import java.io.File;

import com.ctre.phoenix.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.SwerveBase;

public class SwerveDriveWrapper implements SwerveBase {

    private final SwerveDrive swerve;

    public SwerveDriveWrapper() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            swerve = null;
            return;
        }

        try {
            File json = new File("/home/lvuser/deploy/yagsl/");
            swerve = new SwerveParser(json).createSwerveDrive(3);
        } catch (Exception e) {
            throw new RuntimeException("Erro carregando configuração YAGSL!", e);
        }
    }
    @Override
    public void drive(Translation2d translation, double rot, boolean fieldCentric, boolean isOpenLoop) {
        if (swerve == null) return;
        swerve.drive(translation, rot, fieldCentric, isOpenLoop);
    }
    
    @Override
    public SwerveModuleState[] getModuleStates() {
        if (swerve == null) return new SwerveModuleState[0];
        return swerve.getStates();
    }
}

