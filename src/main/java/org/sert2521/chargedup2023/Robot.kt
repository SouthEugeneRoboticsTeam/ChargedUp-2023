package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.sert2521.chargedup2023.commands.SetElevator

object Robot : TimedRobot() {
    private val commandScheduler = CommandScheduler.getInstance()

    override fun robotPeriodic() {
        commandScheduler.run()

        Output.update()
    }

    override fun teleopPeriodic() {
        //Elevator.setExtend(0.1)
        SetElevator(0.13, 0.5).schedule()//.withTimeout(1.0).andThen(SetElevator(0.5, 0.7)).schedule()
    }
}
