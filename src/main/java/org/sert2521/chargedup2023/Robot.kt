package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler

object Robot : TimedRobot() {
    private val commandScheduler = CommandScheduler.getInstance()

    override fun robotPeriodic() {
        commandScheduler.run()

        Output.update()
    }
}
