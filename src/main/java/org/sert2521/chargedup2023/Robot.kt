package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.sert2521.chargedup2023.commands.InitElevator
import org.sert2521.chargedup2023.commands.JoystickDrive
import org.sert2521.chargedup2023.commands.VisionAlignCone
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

        //PathPlannerServer.startServer(5811)
    }

    override fun robotPeriodic() {
        commandScheduler.run()
        Output.update()
    }

    override fun teleopInit() {
        Drivetrain.defaultCommand = JoystickDrive(true)
    }

    override fun teleopExit() {
        // Turn off joystick drive (maybe?)
    }

    override fun disabledExit() {
        // Make it maybe drive while initing
        if (!Elevator.extensionInited || !Elevator.angleInited) {
            val initElevator = InitElevator().withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
            val auto = Input.getAuto()
            if (auto != null && isAutonomous) {
                initElevator.andThen(auto.andThen(InstantCommand({ Drivetrain.stop() }))).schedule()
            } else {
                initElevator.schedule()
            }
        } else {
            if (isAutonomous) {
                Input.getAuto()?.andThen(InstantCommand({ Drivetrain.stop() }))?.schedule()
            }
        }
    }

    override fun disabledPeriodic() {
        if (Drivetrain.camerasConnected()) {
            LEDs.setAllLEDHSV(0, 50, 50)
        } else {
            LEDs.setAllLEDHSV(90, 50, 50)
        }
    }

    override fun disabledInit() {
        commandScheduler.cancelAll()
    }
}
