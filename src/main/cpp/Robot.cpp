// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and motor controller inputs as
 * range from -1 to 1 making it easy to work together.
 *
 * In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent to the Dashboard.
 */
class Robot : public frc::TimedRobot {
 public:
  void TeleopPeriodic() override { 
    if(controller.GetRightTriggerAxis() > 0)
      left_motor.Set(-controller.GetRightY());

    if(controller.GetLeftTriggerAxis() > 0)
      right_motor.Set(controller.GetLeftY());
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  void RobotPeriodic() override {
    frc::SmartDashboard::PutNumber("Encoder", m_encoder.GetDistance());
  }

  void RobotInit() override {
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    m_encoder.SetDistancePerPulse((std::numbers::pi * 6) / 360.0);
  }

 private:
  frc::XboxController controller{0};
  frc::PWMSparkMax left_motor{0};
  frc::PWMSparkMax right_motor{1};
  frc::Encoder m_encoder{0, 1};
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
