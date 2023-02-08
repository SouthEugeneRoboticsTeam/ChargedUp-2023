package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.sert2521.chargedup2023.commands.ClawIntake

object Robot : TimedRobot() {
    private val commandScheduler = CommandScheduler.getInstance()

    override fun robotPeriodic() {
        commandScheduler.run()

        Output.update()
    }

    override fun teleopInit() {
        ClawIntake(true).schedule()
    }

    override fun disabledInit() {
        commandScheduler.cancelAll()
    }
}
