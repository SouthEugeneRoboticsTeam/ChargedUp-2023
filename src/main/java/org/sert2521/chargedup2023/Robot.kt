package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler

object Robot : TimedRobot() {
    private val commandScheduler = CommandScheduler.getInstance()

    init {
        Input
    }
    override fun robotPeriodic() {
        commandScheduler.run()

        Output.update()
    }

    //override fun teleopInit() {
    //}

    override fun disabledInit() {
        commandScheduler.cancelAll()
    }
}
