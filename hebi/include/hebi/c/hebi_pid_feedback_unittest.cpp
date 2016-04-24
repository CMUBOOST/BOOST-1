// Include unit testing code
#include <gtest/gtest.h>

#include "hebi_feedback.h"
#include "hebi_command.h"
#include "hebi_pid_feedback.h"
#include "hebi_settings.h"
#include "hebi_pid_settings.h"
#include "hebi_module.h"
#include "hebi_parrot_module.h"

// TODO: somehow combine with pid_feedback_unittest?

TEST(PidFeedbackCreateTest, Create)
{
  HebiFeedback* fbk = hebiFeedbackCreate();
  ASSERT_TRUE(NULL != fbk);
  HebiActuatorFeedback* af = hebiFeedbackGetActuatorFeedback(fbk);
  EXPECT_TRUE(NULL != af);
  HebiPidFeedback* ppf = hebiActuatorFeedbackGetPositionPidFeedback(af);
  EXPECT_TRUE(NULL != ppf);
  HebiPidFeedback* pvf = hebiActuatorFeedbackGetVelocityPidFeedback(af);
  EXPECT_TRUE(NULL != pvf);
  HebiPidFeedback* ptf = hebiActuatorFeedbackGetTorquePidFeedback(af);
  EXPECT_TRUE(NULL != ptf);
  hebiFeedbackDestroy(fbk);
}

class PidFeedbackParrotModuleTests : public testing::Test
{
  protected:
    HebiCommand* command_send;
    HebiPidSettings* pos_pid_send;
    HebiPidSettings* vel_pid_send;
    HebiPidSettings* trq_pid_send;
    HebiCommand* command_recv;
    HebiFeedback* feedback_recv;
    HebiPidFeedback* pos_pid_recv;
    HebiPidFeedback* vel_pid_recv;
    HebiPidFeedback* trq_pid_recv;
    HebiParrotModule* pm;

    virtual void SetUp()
    {
      command_send = hebiCommandCreate();
      ASSERT_TRUE(NULL != command_send);
      HebiSettings* settings_send = hebiCommandGetSettings(command_send);
      ASSERT_TRUE(NULL != settings_send);
      HebiActuatorSettings* as_send = hebiSettingsGetActuatorSettings(settings_send);
      ASSERT_TRUE(NULL != settings_send);
      pos_pid_send = hebiActuatorSettingsGetPositionPidSettings(as_send);
      vel_pid_send = hebiActuatorSettingsGetVelocityPidSettings(as_send);
      trq_pid_send = hebiActuatorSettingsGetTorquePidSettings(as_send);

      command_recv = hebiCommandCreate();
      ASSERT_TRUE(NULL != command_recv);

      feedback_recv = hebiFeedbackCreate();
      ASSERT_TRUE(NULL != feedback_recv);
      HebiActuatorFeedback* af_recv = hebiFeedbackGetActuatorFeedback(feedback_recv);
      ASSERT_TRUE(NULL != af_recv);
      pos_pid_recv = hebiActuatorFeedbackGetPositionPidFeedback(af_recv);
      vel_pid_recv = hebiActuatorFeedbackGetVelocityPidFeedback(af_recv);
      trq_pid_recv = hebiActuatorFeedbackGetTorquePidFeedback(af_recv);

      pm = hebiParrotModuleCreate();
      ASSERT_TRUE(NULL != pm);
    }
  
    virtual void TearDown()
    {
      hebiCommandDestroy(command_send);
      hebiCommandDestroy(command_recv);
      hebiFeedbackDestroy(feedback_recv);
      hebiParrotModuleDestroy(pm);
    }
};

TEST_F(PidFeedbackParrotModuleTests, Pid)
{
  float setPoint1 = 0.15;
  float setPoint2 = -0.15;
  float setPoint3 = 0;
  float setPoint4 = 0.45;
  float setPoint5 = -0.35;
  float setPoint6 = .74;
  float setPoint7 = 0.333;
  float setPoint8 = 0.634;
  float setPoint9 = -.124;

  // Add feedbackrmation, transfer through the serialization process, and check:
  hebiPidSettingsSetKp(pos_pid_send, setPoint1);
  hebiPidSettingsSetKp(vel_pid_send, setPoint2);
  hebiPidSettingsSetKp(trq_pid_send, setPoint3);
  hebiPidSettingsSetKi(pos_pid_send, setPoint4);
  hebiPidSettingsSetKi(vel_pid_send, setPoint5);
  hebiPidSettingsSetKi(trq_pid_send, setPoint6);
  hebiPidSettingsSetKd(pos_pid_send, setPoint7);
  hebiPidSettingsSetKd(vel_pid_send, setPoint8);
  hebiPidSettingsSetKd(trq_pid_send, setPoint9);
  hebiParrotModuleParrotCommand(pm, command_send, command_recv);
  hebiParrotModuleRequestFeedback(pm, feedback_recv);
  EXPECT_FLOAT_EQ(setPoint1, hebiPidFeedbackGetKp(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint2, hebiPidFeedbackGetKp(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint3, hebiPidFeedbackGetKp(trq_pid_recv));
  EXPECT_FLOAT_EQ(setPoint4, hebiPidFeedbackGetKi(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint5, hebiPidFeedbackGetKi(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint6, hebiPidFeedbackGetKi(trq_pid_recv));
  EXPECT_FLOAT_EQ(setPoint7, hebiPidFeedbackGetKd(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint8, hebiPidFeedbackGetKd(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint9, hebiPidFeedbackGetKd(trq_pid_recv));

  hebiPidSettingsSetKp(pos_pid_send, setPoint6);
  hebiPidSettingsSetKp(vel_pid_send, setPoint5);
  hebiPidSettingsSetKp(trq_pid_send, setPoint4);
  hebiPidSettingsSetKi(pos_pid_send, setPoint9);
  hebiPidSettingsSetKi(vel_pid_send, setPoint8);
  hebiPidSettingsSetKi(trq_pid_send, setPoint7);
  hebiPidSettingsSetKd(pos_pid_send, setPoint3);
  hebiPidSettingsSetKd(vel_pid_send, setPoint2);
  hebiPidSettingsSetKd(trq_pid_send, setPoint1);
  hebiParrotModuleParrotCommand(pm, command_send, command_recv);
  hebiParrotModuleRequestFeedback(pm, feedback_recv);
  EXPECT_FLOAT_EQ(setPoint6, hebiPidFeedbackGetKp(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint5, hebiPidFeedbackGetKp(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint4, hebiPidFeedbackGetKp(trq_pid_recv));
  EXPECT_FLOAT_EQ(setPoint9, hebiPidFeedbackGetKi(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint8, hebiPidFeedbackGetKi(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint7, hebiPidFeedbackGetKi(trq_pid_recv));
  EXPECT_FLOAT_EQ(setPoint3, hebiPidFeedbackGetKd(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint2, hebiPidFeedbackGetKd(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint1, hebiPidFeedbackGetKd(trq_pid_recv));
}

TEST_F(PidFeedbackParrotModuleTests, Other)
{
  float setPoint1 = 0.15;
  float setPoint2 = 0.30;
  float setPoint3 = 0.6623;
  float setPoint4 = 0.45;
  float setPoint5 = 0.35;
  float setPoint6 = 0.74;
  float setPoint7 = 0.333;
  float setPoint8 = 0.634;
  float setPoint9 = 0.124;
  float setPoint10 = 0.224;

  HebiPidSettings* pid_send[3] = {pos_pid_send, vel_pid_send, trq_pid_send};
  HebiPidFeedback* pid_recv[3] = {pos_pid_recv, vel_pid_recv, trq_pid_recv};

  for (int i = 0; i < 3; i++)
  {
    // Add feedback information, transfer through the serialization process, and check:
    hebiPidSettingsSetFeedForward(pid_send[i], setPoint1);
    hebiPidSettingsSetDeadZone(pid_send[i], setPoint2);
    hebiPidSettingsSetIClamp(pid_send[i], setPoint3);
    hebiPidSettingsSetPunch(pid_send[i], setPoint4);
    hebiPidSettingsSetMinTarget(pid_send[i], setPoint5);
    hebiPidSettingsSetMaxTarget(pid_send[i], setPoint6);
    hebiPidSettingsSetTargetLowpassGain(pid_send[i], setPoint7);
    hebiPidSettingsSetMinOutput(pid_send[i], setPoint8);
    hebiPidSettingsSetMaxOutput(pid_send[i], setPoint9);
    hebiPidSettingsSetOutputLowpassGain(pid_send[i], setPoint10);
    hebiPidSettingsSetDOnError(pid_send[i], false);
    hebiParrotModuleParrotCommand(pm, command_send, command_recv);
    hebiParrotModuleRequestFeedback(pm, feedback_recv);
    EXPECT_FLOAT_EQ(setPoint1, hebiPidFeedbackGetFeedForward(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint2, hebiPidFeedbackGetDeadZone(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint3, hebiPidFeedbackGetIClamp(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint4, hebiPidFeedbackGetPunch(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint5, hebiPidFeedbackGetMinTarget(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint6, hebiPidFeedbackGetMaxTarget(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint7, hebiPidFeedbackGetTargetLowpassGain(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint8, hebiPidFeedbackGetMinOutput(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint9, hebiPidFeedbackGetMaxOutput(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint10, hebiPidFeedbackGetOutputLowpassGain(pid_recv[i]));
    EXPECT_FLOAT_EQ(false, hebiPidFeedbackGetDOnError(pid_recv[i]));
  }

  // Switch the order and try again!
  for (int i = 0; i < 3; i++)
  {
    // Add feedbackrmation, transfer through the serialization process, and check:
    hebiPidSettingsSetFeedForward(pid_send[i], setPoint10);
    hebiPidSettingsSetDeadZone(pid_send[i], setPoint9);
    hebiPidSettingsSetIClamp(pid_send[i], setPoint8);
    hebiPidSettingsSetPunch(pid_send[i], setPoint7);
    hebiPidSettingsSetMinTarget(pid_send[i], setPoint6);
    hebiPidSettingsSetMaxTarget(pid_send[i], setPoint5);
    hebiPidSettingsSetTargetLowpassGain(pid_send[i], setPoint4);
    hebiPidSettingsSetMinOutput(pid_send[i], setPoint3);
    hebiPidSettingsSetMaxOutput(pid_send[i], setPoint2);
    hebiPidSettingsSetOutputLowpassGain(pid_send[i], setPoint1);
    hebiPidSettingsSetDOnError(pid_send[i], true);
    hebiParrotModuleParrotCommand(pm, command_send, command_recv);
    hebiParrotModuleRequestFeedback(pm, feedback_recv);
    EXPECT_FLOAT_EQ(setPoint10, hebiPidFeedbackGetFeedForward(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint9, hebiPidFeedbackGetDeadZone(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint8, hebiPidFeedbackGetIClamp(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint7, hebiPidFeedbackGetPunch(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint6, hebiPidFeedbackGetMinTarget(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint5, hebiPidFeedbackGetMaxTarget(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint4, hebiPidFeedbackGetTargetLowpassGain(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint3, hebiPidFeedbackGetMinOutput(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint2, hebiPidFeedbackGetMaxOutput(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint1, hebiPidFeedbackGetOutputLowpassGain(pid_recv[i]));
    EXPECT_FLOAT_EQ(true, hebiPidFeedbackGetDOnError(pid_recv[i]));
  }
}

// TODO: finish basic tests below! Autogenerate?
/*
class ActuatorCommandTests : public testing::Test
{
  protected:
    HebiCommand* cmd;
    HebiActuatorCommand* ac;

    virtual void SetUp()
    {
      cmd = hebiCommandCreate();
      ASSERT_TRUE(NULL != cmd);
      ac = hebiCommandGetActuatorCommand(cmd);
      ASSERT_TRUE(NULL != ac);
    }
  
    virtual void TearDown()
    {
      hebiCommandDestroy(cmd);
    }
};

TEST_F(ActuatorCommandTests, Position)
{
  float setPoint1 = 0.15;
  float setPoint2 = -0.15;
  float setPoint3 = 0;

  // Starts blank:
  ASSERT_EQ(false, hebiActuatorCommandHasPosition(ac));

  // Add feedbackrmation and check:
  hebiActuatorCommandSetPosition(ac, setPoint1);
  ASSERT_EQ(true, hebiActuatorCommandHasPosition(ac));
  EXPECT_FLOAT_EQ(setPoint1, hebiActuatorCommandGetPosition(ac));

  hebiActuatorCommandSetPosition(ac, setPoint2);
  ASSERT_EQ(true, hebiActuatorCommandHasPosition(ac));
  EXPECT_FLOAT_EQ(setPoint2, hebiActuatorCommandGetPosition(ac));

  hebiActuatorCommandSetPosition(ac, setPoint3);
  ASSERT_EQ(true, hebiActuatorCommandHasPosition(ac));
  EXPECT_FLOAT_EQ(setPoint3, hebiActuatorCommandGetPosition(ac));
  
  // Try removing:
  hebiActuatorCommandClearPosition(ac);
  ASSERT_EQ(false, hebiActuatorCommandHasPosition(ac));
}

TEST_F(ActuatorCommandTests, Velocity)
{
  float setPoint1 = 0.15;
  float setPoint2 = -0.15;
  float setPoint3 = 0;

  // Starts blank:
  ASSERT_EQ(false, hebiActuatorCommandHasVelocity(ac));

  // Add feedbackrmation and check:
  hebiActuatorCommandSetVelocity(ac, setPoint1);
  ASSERT_EQ(true, hebiActuatorCommandHasVelocity(ac));
  EXPECT_FLOAT_EQ(setPoint1, hebiActuatorCommandGetVelocity(ac));

  hebiActuatorCommandSetVelocity(ac, setPoint2);
  ASSERT_EQ(true, hebiActuatorCommandHasVelocity(ac));
  EXPECT_FLOAT_EQ(setPoint2, hebiActuatorCommandGetVelocity(ac));

  hebiActuatorCommandSetVelocity(ac, setPoint3);
  ASSERT_EQ(true, hebiActuatorCommandHasVelocity(ac));
  EXPECT_FLOAT_EQ(setPoint3, hebiActuatorCommandGetVelocity(ac));
  
  // Try removing:
  hebiActuatorCommandClearVelocity(ac);
  ASSERT_EQ(false, hebiActuatorCommandHasVelocity(ac));
}

TEST_F(ActuatorCommandTests, Torque)
{
  float setPoint1 = 0.15;
  float setPoint2 = -0.15;
  float setPoint3 = 0;

  // Starts blank:
  ASSERT_EQ(false, hebiActuatorCommandHasTorque(ac));

  // Add feedbackrmation and check:
  hebiActuatorCommandSetTorque(ac, setPoint1);
  ASSERT_EQ(true, hebiActuatorCommandHasTorque(ac));
  EXPECT_FLOAT_EQ(setPoint1, hebiActuatorCommandGetTorque(ac));

  hebiActuatorCommandSetTorque(ac, setPoint2);
  ASSERT_EQ(true, hebiActuatorCommandHasTorque(ac));
  EXPECT_FLOAT_EQ(setPoint2, hebiActuatorCommandGetTorque(ac));

  hebiActuatorCommandSetTorque(ac, setPoint3);
  ASSERT_EQ(true, hebiActuatorCommandHasTorque(ac));
  EXPECT_FLOAT_EQ(setPoint3, hebiActuatorCommandGetTorque(ac));
  
  // Try removing:
  hebiActuatorCommandClearTorque(ac);
  ASSERT_EQ(false, hebiActuatorCommandHasTorque(ac));
}
*/
