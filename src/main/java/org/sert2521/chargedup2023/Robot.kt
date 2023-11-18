package org.sert2521.chargedup2023

import com.pathplanner.lib.server.PathPlannerServer
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.sert2521.chargedup2023.subsystems.Drivetrain

object Robot : TimedRobot() {
    private val commandScheduler = CommandScheduler.getInstance()

    init {
        Input

        PathPlannerServer.startServer(5811)
    }

    override fun robotPeriodic() {
        commandScheduler.run()
    }

    override fun disabledExit() {
    }

    override fun disabledPeriodic() {
    }

    override fun disabledInit() {
        commandScheduler.cancelAll()
    }
}
