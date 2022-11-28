/* Copyright 2022 Texas Torque.
**
* This file is part of Paddlefoot-2022, which is not licensed for distribution.
* For more details, see ./license.txt or write <jus@gtsbr.org>.
*/

package org.texastorque.subsystems;

import org.texastorque.Input;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueTimeout;
import org.texastorque.torquelib.modules.TorqueSwerveModule2022;
import org.texastorque.torquelib.modules.TorqueSwerveModule2022.TorqueSwerveModuleConfiguration;
import org.texastorque.torquelib.modules.base.TorqueSwerveModule;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.motors.legacy.TorqueSparkMax;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Testbench extends TorqueSubsystem implements Subsystems {
    private static volatile Testbench instance;

    final TorqueSwerveModule2022 module;

    private Testbench() {
        final var config = TorqueSwerveModuleConfiguration.defaultConfig;
        module = new TorqueSwerveModule2022(8, 15, 0, config);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
    }

    private TorqueClick tear = new TorqueClick();

    @Override
    public final void update(final TorqueMode mode) {
        final var ctrl = Input.getInstance().getDriver();
        final double lx = ctrl.getLeftXAxis();
        final double ly = ctrl.getLeftYAxis();
        final double rx = ctrl.getRightXAxis();
        final double ry = ctrl.getRightYAxis();

        if (tear.calculate(ctrl.isAButtonDown()))
            module.tear();

        final double rot = rx == 0 && ry == 0 ? 0 : Math.atan2(rx, -ry); // rotation of the left stick radians
        SmartDashboard.putNumber("Raw rot", rot);

        final var state = new SwerveModuleState(3 * lx, new Rotation2d(rot));

        SmartDashboard.putNumber("Req Velo", state.speedMetersPerSecond);
        SmartDashboard.putNumber("Req Turn", state.angle.getRadians());

        module.setDesiredState(state);

        SmartDashboard.putNumber("Rotation", rot);

        final var feedback = module.getState();
        SmartDashboard.putNumber("Real Velo", feedback.speedMetersPerSecond);
        SmartDashboard.putNumber("Real Turn", feedback.angle.getRadians());






    }

    public static final synchronized Testbench getInstance() {
        return instance == null ? instance = new Testbench() : instance;
    }
}
