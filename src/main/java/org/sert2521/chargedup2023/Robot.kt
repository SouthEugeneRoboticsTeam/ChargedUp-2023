package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.sert2521.chargedup2023.commands.InitElevator
import org.sert2521.chargedup2023.commands.JoystickDrive
import org.sert2521.chargedup2023.subsystems.Elevator

object Robot : TimedRobot() {
    private val commandScheduler = CommandScheduler.getInstance()

    init {
        Input
    }

    override fun robotPeriodic() {
        commandScheduler.run()

        Output.update()
    }

    override fun teleopInit() {
        JoystickDrive(true).schedule()
    }

    override fun autonomousInit() {
        Input.getAuto()?.schedule()
    }

    override fun disabledExit() {
        if (!Elevator.extensionInited) {
            InitElevator().withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).schedule()
        }
    }

    override fun disabledInit() {
        commandScheduler.cancelAll()
    }
}
