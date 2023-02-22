package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Elevator

class InitElevator : CommandBase() {
    private val anglePID = ProfiledPIDController(TunedConstants.elevatorAngleP, TunedConstants.elevatorAngleI, TunedConstants.elevatorAngleD, TrapezoidProfile.Constraints(TunedConstants.elevatorAngleMaxV, TunedConstants.elevatorAngleMaxA))
    private var angleTarget = 0.0

    init {
        addRequirements(Elevator)
    }

    override fun initialize() {
        val angle = Elevator.angleMeasure()
        anglePID.reset(angle)

        angleTarget = if (angle >= TunedConstants.elevatorExtensionMinAngleTarget) {
            angle
        } else {
            TunedConstants.elevatorExtensionMinAngleTarget
        }
    }

    override fun execute() {
        Elevator.setAngle(anglePID.calculate(Elevator.angleMeasure(), angleTarget))
    }

    override fun isFinished(): Boolean {
        return Elevator.extensionInited
    }

    override fun end(interrupted: Boolean) {
        Elevator.stop()
    }
}