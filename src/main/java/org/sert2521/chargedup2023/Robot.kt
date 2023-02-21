package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.sert2521.chargedup2023.subsystems.LEDSides
import org.sert2521.chargedup2023.subsystems.LEDs

object Robot : TimedRobot() {
    private val commandScheduler = CommandScheduler.getInstance()
    init{
        LEDs.setAllLEDHSV(60, 100, 80)



    }
    override fun robotPeriodic() {
        commandScheduler.run()



        Output.update()
    }
}
