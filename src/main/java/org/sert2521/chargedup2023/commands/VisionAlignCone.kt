package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import org.sert2521.chargedup2023.Input
import org.sert2521.chargedup2023.Output
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import java.lang.Math.PI
import kotlin.math.abs

class VisionAlignCone : JoystickCommand() {
    private val positionPID = ProfiledPIDController(TunedConstants.swerveAlignDistanceP, TunedConstants.swerveAlignDistanceI, TunedConstants.swerveAlignDistanceD, TrapezoidProfile.Constraints(TunedConstants.swerveAlignV, TunedConstants.swerveAlignA))
    private val anglePID = PIDController(TunedConstants.swerveAlignAngleP, TunedConstants.swerveAlignAngleI, TunedConstants.swerveAlignAngleD)

    init {
        addRequirements(Drivetrain)

        anglePID.enableContinuousInput(0.0, 2 * PI)

        positionPID.setTolerance(TunedConstants.visionPositionTolerance)
        anglePID.setTolerance(TunedConstants.visionAngleTolerance)
    }

    override fun initialize() {
        super.initialize()

        Drivetrain.setVisionStandardDeviations()
        positionPID.reset(Drivetrain.getPose().y, Drivetrain.deltaPose.y)
        anglePID.reset()
    }

    override fun execute() {
        val pose = Drivetrain.getVisionPose()

        val yTarget = PhysicalConstants.conePoints.minBy { abs(it - pose.y) }
        val angleTarget = PhysicalConstants.colorToConeAngle[Input.getColor()]
        if (angleTarget != null) {
            // Moving the x on the stick will affect the rate of change of the y

            if (!positionPID.atSetpoint() || !anglePID.atSetpoint()) {
                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(readJoystick().x, positionPID.calculate(pose.y, yTarget), anglePID.calculate(pose.rotation.radians, angleTarget), pose.rotation))
                Output.visionHappy = true
            } else {
                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(readJoystick().x, 0.0, 0.0, pose.rotation))
                Output.visionHappy = true
            }
        } else {
            positionPID.reset(pose.y, Drivetrain.deltaPose.y)
            anglePID.reset()

            Drivetrain.stop()
            Output.visionHappy = false
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.setVisionAlignDeviations()
        Output.visionHappy = false
        Drivetrain.stop()
    }
}