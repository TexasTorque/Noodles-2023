/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.auto.commands.EventPath;
import org.texastorque.auto.commands.Path;
import org.texastorque.auto.commands.Wait;
import org.texastorque.auto.commands.Zero;
import org.texastorque.torquelib.auto.TorqueBlock;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;

public final class Drive extends TorqueSequence {
    public Drive() { 
        addBlock(new TorqueBlock(new Zero()));
        // final EventPath testPath = new EventPath("Test", true, 2, 1);
        // testPath.addEvent("A", new TorqueExecute(() -> {
        //     for (int i = 0; i < 10; i++)
        //         System.out.println("AAAAAAAAAA");
        // }));
        // addBlock(new TorqueBlock(testPath));
        // addBlock(new Path("move-tests", true, 1, 1));
        addBlock(new Path("cool-path-a", true, 2, 1));
        addBlock(new Wait(1));
        addBlock(new Path("cool-path-b", false, 1.5, 1));
        addBlock(new Wait(1));
        addBlock(new Path("cool-path-c", false, 2, 1));

    }
}