package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Elevator

class SetElevator(private val extension: Double, private val angle: Double) : CommandBase() {
    private val anglePID = PIDController(TunedConstants.elevatorAngleP, TunedConstants.elevatorAngleI, TunedConstants.elevatorAngleD)
    private val angleFeedforward = ArmFeedforward(TunedConstants.elevatorAngleS, TunedConstants.elevatorAngleG, TunedConstants.elevatorAngleV)

    private val extensionPID = PIDController(TunedConstants.elevatorExtensionP, TunedConstants.elevatorExtensionI, TunedConstants.elevatorExtensionD)

    init {
        addRequirements(Elevator)

        extensionPID.setTolerance(TunedConstants.elevatorExtensionTolerance)
    }

    override fun initialize() {
        anglePID.reset()
        extensionPID.reset()
    }

    // Make it stay above the extension safe limit until extension is in bounds
    override fun execute() {
        val angleTarget: Double = if (angle >= TunedConstants.elevatorExtensionMinAngleTarget || (extensionPID.atSetpoint() && Elevator.extensionInited)) {
            angle
        } else {
            TunedConstants.elevatorExtensionMinAngleTarget
        }

        Elevator.setAngle(angleFeedforward.calculate(angleTarget, 0.0) + anglePID.calculate(Elevator.angleMeasure(), angleTarget))
        Elevator.setExtend(extensionPID.calculate(Elevator.extensionMeasure(), extension))
    }

    override fun end(interrupted: Boolean) {
        Elevator.stop()
    }
}