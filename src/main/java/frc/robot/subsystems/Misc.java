package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.logging.LogGroup;
import frc.robot.logging.LogProfileBuilder;
import frc.robot.logging.Logger;

public class Misc extends SubsystemBase {
    private PowerDistribution pdh = new PowerDistribution();
    private PneumaticHub ph = new PneumaticHub();

    private LogGroup logger = new LogGroup("Miscellaneous", new Logger<?>[] {
            new Logger<PowerDistribution>(pdh, "Power Distribution Hub",
                    LogProfileBuilder.buildPDHLogItems(pdh)),
            new Logger<PneumaticHub>(ph, "Pneumatic Hub",
                    LogProfileBuilder.buildPneumaticHubLogItems(ph)),
    });

    public Misc() {

    }

    /**
     * Runs every 20ms. In this method, all we do is run SmartDashboard/logging
     * related functions (do NOT run any code that should belong in a command here!)
     */
    @Override
    public void periodic() {
        logger.run();
    }

}