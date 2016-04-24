// Include unit testing code
#include <gtest/gtest.h>

#include "hebi_command.h"
#include "hebi_settings.h"
#include "hebi_pid_settings.h"
#include "hebi_module.h"
#include "hebi_parrot_module.h"

// TODO: somehow combine with pid_command_unittest?

TEST(PidSettingsCreateTest, Create)
{
  HebiCommand* command = hebiCommandCreate();
  ASSERT_TRUE(NULL != command);
  HebiSettings* settings = hebiCommandGetSettings(command);
  ASSERT_TRUE(NULL != settings);
  HebiActuatorSettings* as = hebiSettingsGetActuatorSettings(settings);
  EXPECT_TRUE(NULL != as);
  HebiPidSettings* pps = hebiActuatorSettingsGetPositionPidSettings(as);
  EXPECT_TRUE(NULL != pps);
  HebiPidSettings* pvs = hebiActuatorSettingsGetVelocityPidSettings(as);
  EXPECT_TRUE(NULL != pvs);
  HebiPidSettings* pts = hebiActuatorSettingsGetTorquePidSettings(as);
  EXPECT_TRUE(NULL != pts);
  hebiCommandDestroy(command);
}

class PidSettingsParrotModuleTests : public testing::Test
{
  protected:
    HebiCommand* command_send;
    HebiPidSettings* pos_pid_send;
    HebiPidSettings* vel_pid_send;
    HebiPidSettings* trq_pid_send;
    HebiCommand* command_recv;
    HebiPidSettings* pos_pid_recv;
    HebiPidSettings* vel_pid_recv;
    HebiPidSettings* trq_pid_recv;
    HebiParrotModule* pm;

    virtual void SetUp()
    {
      command_send = hebiCommandCreate();
      ASSERT_TRUE(NULL != command_send);
      HebiSettings* settings_send = hebiCommandGetSettings(command_send);
      ASSERT_TRUE(NULL != settings_send);
      HebiActuatorSettings* as_send = hebiSettingsGetActuatorSettings(settings_send);
      ASSERT_TRUE(NULL != as_send);
      pos_pid_send = hebiActuatorSettingsGetPositionPidSettings(as_send);
      vel_pid_send = hebiActuatorSettingsGetVelocityPidSettings(as_send);
      trq_pid_send = hebiActuatorSettingsGetTorquePidSettings(as_send);

      command_recv = hebiCommandCreate();
      ASSERT_TRUE(NULL != command_recv);
      HebiSettings* settings_recv = hebiCommandGetSettings(command_recv);
      ASSERT_TRUE(NULL != settings_recv);
      HebiActuatorSettings* as_recv = hebiSettingsGetActuatorSettings(settings_recv);
      ASSERT_TRUE(NULL != as_recv);
      pos_pid_recv = hebiActuatorSettingsGetPositionPidSettings(as_recv);
      vel_pid_recv = hebiActuatorSettingsGetVelocityPidSettings(as_recv);
      trq_pid_recv = hebiActuatorSettingsGetTorquePidSettings(as_recv);

      pm = hebiParrotModuleCreate();
      ASSERT_TRUE(NULL != pm);
    }
  
    virtual void TearDown()
    {
      hebiCommandDestroy(command_send);
      hebiCommandDestroy(command_recv);
      hebiParrotModuleDestroy(pm);
    }
};

TEST_F(PidSettingsParrotModuleTests, Pid)
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

  // Add information, transfer through the serialization process, and check:
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
  EXPECT_FLOAT_EQ(setPoint1, hebiPidSettingsGetKp(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint2, hebiPidSettingsGetKp(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint3, hebiPidSettingsGetKp(trq_pid_recv));
  EXPECT_FLOAT_EQ(setPoint4, hebiPidSettingsGetKi(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint5, hebiPidSettingsGetKi(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint6, hebiPidSettingsGetKi(trq_pid_recv));
  EXPECT_FLOAT_EQ(setPoint7, hebiPidSettingsGetKd(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint8, hebiPidSettingsGetKd(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint9, hebiPidSettingsGetKd(trq_pid_recv));

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
  EXPECT_FLOAT_EQ(setPoint6, hebiPidSettingsGetKp(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint5, hebiPidSettingsGetKp(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint4, hebiPidSettingsGetKp(trq_pid_recv));
  EXPECT_FLOAT_EQ(setPoint9, hebiPidSettingsGetKi(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint8, hebiPidSettingsGetKi(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint7, hebiPidSettingsGetKi(trq_pid_recv));
  EXPECT_FLOAT_EQ(setPoint3, hebiPidSettingsGetKd(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint2, hebiPidSettingsGetKd(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint1, hebiPidSettingsGetKd(trq_pid_recv));
}

TEST_F(PidSettingsParrotModuleTests, Other)
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
  HebiPidSettings* pid_recv[3] = {pos_pid_recv, vel_pid_recv, trq_pid_recv};

  for (int i = 0; i < 3; i++)
  {
    // Add information, transfer through the serialization process, and check:
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
    EXPECT_FLOAT_EQ(setPoint1, hebiPidSettingsGetFeedForward(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint2, hebiPidSettingsGetDeadZone(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint3, hebiPidSettingsGetIClamp(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint4, hebiPidSettingsGetPunch(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint5, hebiPidSettingsGetMinTarget(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint6, hebiPidSettingsGetMaxTarget(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint7, hebiPidSettingsGetTargetLowpassGain(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint8, hebiPidSettingsGetMinOutput(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint9, hebiPidSettingsGetMaxOutput(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint10, hebiPidSettingsGetOutputLowpassGain(pid_recv[i]));
    EXPECT_FLOAT_EQ(false, hebiPidSettingsGetDOnError(pid_recv[i]));
  }

  // Switch the order and try again!
  for (int i = 0; i < 3; i++)
  {
    // Add information, transfer through the serialization process, and check:
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
    EXPECT_FLOAT_EQ(setPoint10, hebiPidSettingsGetFeedForward(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint9, hebiPidSettingsGetDeadZone(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint8, hebiPidSettingsGetIClamp(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint7, hebiPidSettingsGetPunch(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint6, hebiPidSettingsGetMinTarget(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint5, hebiPidSettingsGetMaxTarget(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint4, hebiPidSettingsGetTargetLowpassGain(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint3, hebiPidSettingsGetMinOutput(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint2, hebiPidSettingsGetMaxOutput(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint1, hebiPidSettingsGetOutputLowpassGain(pid_recv[i]));
    EXPECT_FLOAT_EQ(true, hebiPidSettingsGetDOnError(pid_recv[i]));
  }
}

// TODO: finish basic tests below! Autogenerate?
/*
class ActuatorSettingsTests : public testing::Test
{
  protected:
    HebiSettings* cmd;
    HebiActuatorSettings* ac;

    virtual void SetUp()
    {
      cmd = hebiSettingsCreate();
      ASSERT_TRUE(NULL != cmd);
      ac = hebiSettingsGetActuatorSettings(cmd);
      ASSERT_TRUE(NULL != ac);
    }
  
    virtual void TearDown()
    {
      hebiSettingsDestroy(cmd);
    }
};

TEST_F(ActuatorSettingsTests, Position)
{
  float setPoint1 = 0.15;
  float setPoint2 = -0.15;
  float setPoint3 = 0;

  // Starts blank:
  ASSERT_EQ(false, hebiActuatorSettingsHasPosition(ac));

  // Add information and check:
  hebiActuatorSettingsSetPosition(ac, setPoint1);
  ASSERT_EQ(true, hebiActuatorSettingsHasPosition(ac));
  EXPECT_FLOAT_EQ(setPoint1, hebiActuatorSettingsGetPosition(ac));

  hebiActuatorSettingsSetPosition(ac, setPoint2);
  ASSERT_EQ(true, hebiActuatorSettingsHasPosition(ac));
  EXPECT_FLOAT_EQ(setPoint2, hebiActuatorSettingsGetPosition(ac));

  hebiActuatorSettingsSetPosition(ac, setPoint3);
  ASSERT_EQ(true, hebiActuatorSettingsHasPosition(ac));
  EXPECT_FLOAT_EQ(setPoint3, hebiActuatorSettingsGetPosition(ac));
  
  // Try removing:
  hebiActuatorSettingsClearPosition(ac);
  ASSERT_EQ(false, hebiActuatorSettingsHasPosition(ac));
}

TEST_F(ActuatorSettingsTests, Velocity)
{
  float setPoint1 = 0.15;
  float setPoint2 = -0.15;
  float setPoint3 = 0;

  // Starts blank:
  ASSERT_EQ(false, hebiActuatorSettingsHasVelocity(ac));

  // Add information and check:
  hebiActuatorSettingsSetVelocity(ac, setPoint1);
  ASSERT_EQ(true, hebiActuatorSettingsHasVelocity(ac));
  EXPECT_FLOAT_EQ(setPoint1, hebiActuatorSettingsGetVelocity(ac));

  hebiActuatorSettingsSetVelocity(ac, setPoint2);
  ASSERT_EQ(true, hebiActuatorSettingsHasVelocity(ac));
  EXPECT_FLOAT_EQ(setPoint2, hebiActuatorSettingsGetVelocity(ac));

  hebiActuatorSettingsSetVelocity(ac, setPoint3);
  ASSERT_EQ(true, hebiActuatorSettingsHasVelocity(ac));
  EXPECT_FLOAT_EQ(setPoint3, hebiActuatorSettingsGetVelocity(ac));
  
  // Try removing:
  hebiActuatorSettingsClearVelocity(ac);
  ASSERT_EQ(false, hebiActuatorSettingsHasVelocity(ac));
}

TEST_F(ActuatorSettingsTests, Torque)
{
  float setPoint1 = 0.15;
  float setPoint2 = -0.15;
  float setPoint3 = 0;

  // Starts blank:
  ASSERT_EQ(false, hebiActuatorSettingsHasTorque(ac));

  // Add information and check:
  hebiActuatorSettingsSetTorque(ac, setPoint1);
  ASSERT_EQ(true, hebiActuatorSettingsHasTorque(ac));
  EXPECT_FLOAT_EQ(setPoint1, hebiActuatorSettingsGetTorque(ac));

  hebiActuatorSettingsSetTorque(ac, setPoint2);
  ASSERT_EQ(true, hebiActuatorSettingsHasTorque(ac));
  EXPECT_FLOAT_EQ(setPoint2, hebiActuatorSettingsGetTorque(ac));

  hebiActuatorSettingsSetTorque(ac, setPoint3);
  ASSERT_EQ(true, hebiActuatorSettingsHasTorque(ac));
  EXPECT_FLOAT_EQ(setPoint3, hebiActuatorSettingsGetTorque(ac));
  
  // Try removing:
  hebiActuatorSettingsClearTorque(ac);
  ASSERT_EQ(false, hebiActuatorSettingsHasTorque(ac));
}
*/
