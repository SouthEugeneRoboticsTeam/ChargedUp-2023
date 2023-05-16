package org.sert2521.chargedup2023

import com.pathplanner.lib.server.PathPlannerServer
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.sert2521.chargedup2023.commands.InitElevator
import org.sert2521.chargedup2023.subsystems.Drivetrain
import org.sert2521.chargedup2023.subsystems.Elevator
import org.sert2521.chargedup2023.subsystems.LEDs

object Robot : TimedRobot() {
    private val commandScheduler = CommandScheduler.getInstance()

    init {
        Input
        // Just so braking mode engages (maybe?)
        Elevator

        LEDs

        PathPlannerServer.startServer(5811)
    }

    override fun robotPeriodic() {
        commandScheduler.run()
        Output.update()
    }

    override fun disabledExit() {
        if (!Elevator.extensionInited || !Elevator.angleInited) {
            InitElevator().withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).schedule()
        }
    }

    override fun disabledPeriodic() {
        if (Drivetrain.camerasConnected()) {
            LEDs.setAllLEDRGB(0, 50, 0)
        } else {
            LEDs.setAllLEDRGB(50, 0, 0)
        }
    }

    override fun disabledInit() {
        commandScheduler.cancelAll()
    }
}
