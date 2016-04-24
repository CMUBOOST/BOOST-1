// Include unit testing code
#include <gtest/gtest.h>

#include "hebi_info.h"
#include "hebi_settings.h"
#include "hebi_pid_info.h"
#include "hebi_pid_settings.h"
#include "hebi_module.h"
#include "hebi_parrot_module.h"

// TODO: somehow combine with pid_feedback_unittest?

TEST(PidInfoCreateTest, Create)
{
  HebiInfo* info = hebiInfoCreate();
  ASSERT_TRUE(NULL != info);
  HebiActuatorInfo* ai = hebiInfoGetActuatorInfo(info);
  EXPECT_TRUE(NULL != ai);
  HebiPidInfo* ppi = hebiActuatorInfoGetPositionPidInfo(ai);
  EXPECT_TRUE(NULL != ppi);
  HebiPidInfo* pvi = hebiActuatorInfoGetVelocityPidInfo(ai);
  EXPECT_TRUE(NULL != pvi);
  HebiPidInfo* pti = hebiActuatorInfoGetTorquePidInfo(ai);
  EXPECT_TRUE(NULL != pti);
  hebiInfoDestroy(info);
}

class PidInfoParrotModuleTests : public testing::Test
{
  protected:
    HebiCommand* command_send;
    HebiSettings* settings_send;
    HebiPidSettings* pos_pid_send;
    HebiPidSettings* vel_pid_send;
    HebiPidSettings* trq_pid_send;
    HebiCommand* command_recv;
    HebiSettings* settings_recv;
    HebiInfo* info_recv;
    HebiPidInfo* pos_pid_recv;
    HebiPidInfo* vel_pid_recv;
    HebiPidInfo* trq_pid_recv;
    HebiParrotModule* pm;

    virtual void SetUp()
    {
      command_send = hebiCommandCreate();
      ASSERT_TRUE(NULL != command_send);
      settings_send = hebiCommandGetSettings(command_send);
      ASSERT_TRUE(NULL != settings_send);
      HebiActuatorSettings* as_send = hebiSettingsGetActuatorSettings(settings_send);
      ASSERT_TRUE(NULL != as_send);
      pos_pid_send = hebiActuatorSettingsGetPositionPidSettings(as_send);
      vel_pid_send = hebiActuatorSettingsGetVelocityPidSettings(as_send);
      trq_pid_send = hebiActuatorSettingsGetTorquePidSettings(as_send);

      command_recv = hebiCommandCreate();
      ASSERT_TRUE(NULL != command_recv);
      settings_recv = hebiCommandGetSettings(command_recv);
      ASSERT_TRUE(NULL != settings_recv);

      info_recv = hebiInfoCreate();
      ASSERT_TRUE(NULL != info_recv);
      HebiActuatorInfo* ai_recv = hebiInfoGetActuatorInfo(info_recv);
      ASSERT_TRUE(NULL != ai_recv);
      pos_pid_recv = hebiActuatorInfoGetPositionPidInfo(ai_recv);
      vel_pid_recv = hebiActuatorInfoGetVelocityPidInfo(ai_recv);
      trq_pid_recv = hebiActuatorInfoGetTorquePidInfo(ai_recv);

      pm = hebiParrotModuleCreate();
      ASSERT_TRUE(NULL != pm);
    }
  
    virtual void TearDown()
    {
      hebiCommandDestroy(command_send);
      hebiCommandDestroy(command_recv);
      hebiInfoDestroy(info_recv);
      hebiParrotModuleDestroy(pm);
    }
};

TEST_F(PidInfoParrotModuleTests, Pid)
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
  hebiParrotModuleRequestInfo(pm, info_recv);
  EXPECT_FLOAT_EQ(setPoint1, hebiPidInfoGetKp(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint2, hebiPidInfoGetKp(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint3, hebiPidInfoGetKp(trq_pid_recv));
  EXPECT_FLOAT_EQ(setPoint4, hebiPidInfoGetKi(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint5, hebiPidInfoGetKi(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint6, hebiPidInfoGetKi(trq_pid_recv));
  EXPECT_FLOAT_EQ(setPoint7, hebiPidInfoGetKd(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint8, hebiPidInfoGetKd(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint9, hebiPidInfoGetKd(trq_pid_recv));

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
  hebiParrotModuleRequestInfo(pm, info_recv);
  EXPECT_FLOAT_EQ(setPoint6, hebiPidInfoGetKp(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint5, hebiPidInfoGetKp(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint4, hebiPidInfoGetKp(trq_pid_recv));
  EXPECT_FLOAT_EQ(setPoint9, hebiPidInfoGetKi(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint8, hebiPidInfoGetKi(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint7, hebiPidInfoGetKi(trq_pid_recv));
  EXPECT_FLOAT_EQ(setPoint3, hebiPidInfoGetKd(pos_pid_recv));
  EXPECT_FLOAT_EQ(setPoint2, hebiPidInfoGetKd(vel_pid_recv));
  EXPECT_FLOAT_EQ(setPoint1, hebiPidInfoGetKd(trq_pid_recv));
}

TEST_F(PidInfoParrotModuleTests, Other)
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
  HebiPidInfo* pid_recv[3] = {pos_pid_recv, vel_pid_recv, trq_pid_recv};

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
    hebiParrotModuleRequestInfo(pm, info_recv);
    EXPECT_FLOAT_EQ(setPoint1, hebiPidInfoGetFeedForward(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint2, hebiPidInfoGetDeadZone(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint3, hebiPidInfoGetIClamp(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint4, hebiPidInfoGetPunch(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint5, hebiPidInfoGetMinTarget(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint6, hebiPidInfoGetMaxTarget(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint7, hebiPidInfoGetTargetLowpassGain(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint8, hebiPidInfoGetMinOutput(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint9, hebiPidInfoGetMaxOutput(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint10, hebiPidInfoGetOutputLowpassGain(pid_recv[i]));
    EXPECT_FLOAT_EQ(false, hebiPidInfoGetDOnError(pid_recv[i]));
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
    hebiParrotModuleRequestInfo(pm, info_recv);
    EXPECT_FLOAT_EQ(setPoint10, hebiPidInfoGetFeedForward(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint9, hebiPidInfoGetDeadZone(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint8, hebiPidInfoGetIClamp(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint7, hebiPidInfoGetPunch(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint6, hebiPidInfoGetMinTarget(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint5, hebiPidInfoGetMaxTarget(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint4, hebiPidInfoGetTargetLowpassGain(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint3, hebiPidInfoGetMinOutput(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint2, hebiPidInfoGetMaxOutput(pid_recv[i]));
    EXPECT_FLOAT_EQ(setPoint1, hebiPidInfoGetOutputLowpassGain(pid_recv[i]));
    EXPECT_FLOAT_EQ(true, hebiPidInfoGetDOnError(pid_recv[i]));
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
