
package frc;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveBase {
    void drive(Translation2d translation, double rot, boolean fieldCentric, boolean openLoop);
    SwerveModuleState[] getModuleStates();
}

