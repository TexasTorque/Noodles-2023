/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * DO NOT modify this class.
 */
public final class Main {
    private Main() {}

    /**
     * Do not modify this function.
     *
     * @param args Command line arguments.
     */
    public final static void main(final String... args) { RobotBase.startRobot(Robot::new); }
}
