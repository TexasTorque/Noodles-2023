/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.auto.commands.Path;
import org.texastorque.torquelib.auto.TorqueBlock;
import org.texastorque.torquelib.auto.TorqueSequence;

public final class Drive extends TorqueSequence {
    public Drive() { 
        addBlock(new TorqueBlock(new Path("Test", true, 2, 1))); 
    }
}