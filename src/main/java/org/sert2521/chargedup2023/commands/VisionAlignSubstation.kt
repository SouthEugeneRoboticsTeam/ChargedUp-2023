package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.sert2521.chargedup2023.Output
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import java.lang.Math.PI
import kotlin.math.abs

class VisionAlignSubstation : JoystickCommand() {
    private val positionPID = PIDController(TunedConstants.swerveAlignDistanceP, TunedConstants.swerveAlignDistanceI, TunedConstants.swerveAlignDistanceD)
    private val anglePID = PIDController(TunedConstants.swerveSubstationAlignAngleP, TunedConstants.swerveSubstationAlignAngleI, TunedConstants.swerveSubstationAlignAngleD)

    init {
        addRequirements(Drivetrain)

        anglePID.enableContinuousInput(0.0, 2 * PI)

        positionPID.setTolerance(TunedConstants.visionSubstationPositionTolerance)
        anglePID.setTolerance(TunedConstants.visionSubstationAngleTolerance)
    }

    override fun initialize() {
        super.initialize()

        Drivetrain.setVisionStandardDeviations()
        positionPID.reset()
        anglePID.reset()
    }

    override fun execute() {
        val pose = Drivetrain.getVisionPose()

        // Moving the x on the stick will affect the rate of change of the y
        if (!positionPID.atSetpoint() || !anglePID.atSetpoint()) {
            val t = clamp((abs(pose.x - PhysicalConstants.substationX) - PhysicalConstants.substationCloseAngleAtDistance.second) / (PhysicalConstants.substationFarAngleAtDistance.second - PhysicalConstants.substationCloseAngleAtDistance.second), 0.0, 1.0)
            val angleTarget = t * (PhysicalConstants.substationFarAngleAtDistance.first - PhysicalConstants.substationCloseAngleAtDistance.first) + PhysicalConstants.substationCloseAngleAtDistance.first

            Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(positionPID.calculate(pose.x, PhysicalConstants.substationX), readJoystick().y, anglePID.calculate(pose.rotation.radians, angleTarget), pose.rotation))
            Output.visionHappy = false
        } else {
            // To update stuff
            positionPID.calculate(pose.y, PhysicalConstants.substationX)
            anglePID.calculate(pose.rotation.radians, PhysicalConstants.coneAngle)

            Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, readJoystick().y, 0.0, pose.rotation))
            Output.visionHappy = true
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.setVisionAlignDeviations()
        Output.visionHappy = false
        Drivetrain.stop()
    }
}