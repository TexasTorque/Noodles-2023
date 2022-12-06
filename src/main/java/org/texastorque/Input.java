/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueSlewLimiter;
import org.texastorque.torquelib.control.TorqueTraversableSelection;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.util.TorqueUtil;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private Input() {
        driver = new TorqueController(0, .1);
        operator = new TorqueController(1);
    }

    @Override
    public final void update() {
        updateDrivebase();
    }


    private final TorqueTraversableSelection<Double> speedSettings 
            = new TorqueTraversableSelection<Double>(0, 1.0, 0.75, 0.5, 0.25);

    private final TorqueClick toggleRotationLock = new TorqueClick();

    private void updateDrivebase() { 

        final double speedSetting = speedSettings.calculate(driver.isRightBumperDown(), driver.isLeftBumperDown());

        final double xVelocity = -driver.getLeftYAxis() * Drivebase.MAX_VELOCITY * speedSetting;
        final double yVelocity = driver.getLeftXAxis() * Drivebase.MAX_VELOCITY * speedSetting;
        final double rotationVelocity = driver.getRightXAxis() * Drivebase.MAX_ANGULAR_VELOCITY * speedSetting;

        drivebase.inputSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);

        if (toggleRotationLock.calculate(driver.isAButtonDown()))
            drivebase.isRotationLocked = !drivebase.isRotationLocked;

    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}
