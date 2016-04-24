// Include unit testing code
#include <gtest/gtest.h>

#include "hebi_command.h"
#include "hebi_module.h"
#include "hebi_parrot_module.h"

TEST(ActuatorCommandCreateTest, Create)
{
  HebiCommand* cmd = hebiCommandCreate();
  ASSERT_TRUE(NULL != cmd);
  HebiActuatorCommand* ac = hebiCommandGetActuatorCommand(cmd);
  EXPECT_TRUE(NULL != ac);
  hebiCommandDestroy(cmd);
}

class ActuatorCommandParrotModuleTests : public testing::Test
{
  protected:
    HebiCommand* cmd_send;
    HebiActuatorCommand* ac_send;
    HebiCommand* cmd_recv;
    HebiActuatorCommand* ac_recv;
    HebiParrotModule* pm;

    virtual void SetUp()
    {
      cmd_send = hebiCommandCreate();
      ASSERT_TRUE(NULL != cmd_send);
      ac_send = hebiCommandGetActuatorCommand(cmd_send);
      ASSERT_TRUE(NULL != ac_send);
      cmd_recv = hebiCommandCreate();
      ASSERT_TRUE(NULL != cmd_recv);
      ac_recv = hebiCommandGetActuatorCommand(cmd_recv);
      ASSERT_TRUE(NULL != ac_recv);
      pm = hebiParrotModuleCreate();
      ASSERT_TRUE(NULL != pm);
    }
  
    virtual void TearDown()
    {
      hebiCommandDestroy(cmd_send);
      hebiCommandDestroy(cmd_recv);
      hebiParrotModuleDestroy(pm);
    }
};

TEST_F(ActuatorCommandParrotModuleTests, Position)
{
  float setPoint1 = 0.15;
  float setPoint2 = -0.15;
  float setPoint3 = 0;

  // Add information, transfer through the serialization process, and check:
  hebiActuatorCommandSetPosition(ac_send, setPoint1);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  ASSERT_EQ(true, hebiActuatorCommandHasPosition(ac_recv));
  EXPECT_FLOAT_EQ(setPoint1, hebiActuatorCommandGetPosition(ac_recv));

  hebiActuatorCommandSetPosition(ac_send, setPoint2);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  ASSERT_EQ(true, hebiActuatorCommandHasPosition(ac_recv));
  EXPECT_FLOAT_EQ(setPoint2, hebiActuatorCommandGetPosition(ac_recv));

  hebiActuatorCommandSetPosition(ac_send, setPoint3);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  ASSERT_EQ(true, hebiActuatorCommandHasPosition(ac_recv));
  EXPECT_FLOAT_EQ(setPoint3, hebiActuatorCommandGetPosition(ac_recv));
}

TEST_F(ActuatorCommandParrotModuleTests, Velocity)
{
  float setPoint1 = 0.15;
  float setPoint2 = -0.15;
  float setPoint3 = 0;

  // Add information, transfer through the serialization process, and check:
  hebiActuatorCommandSetVelocity(ac_send, setPoint1);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  ASSERT_EQ(true, hebiActuatorCommandHasVelocity(ac_recv));
  EXPECT_FLOAT_EQ(setPoint1, hebiActuatorCommandGetVelocity(ac_recv));

  hebiActuatorCommandSetVelocity(ac_send, setPoint2);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  ASSERT_EQ(true, hebiActuatorCommandHasVelocity(ac_recv));
  EXPECT_FLOAT_EQ(setPoint2, hebiActuatorCommandGetVelocity(ac_recv));

  hebiActuatorCommandSetVelocity(ac_send, setPoint3);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  ASSERT_EQ(true, hebiActuatorCommandHasVelocity(ac_recv));
  EXPECT_FLOAT_EQ(setPoint3, hebiActuatorCommandGetVelocity(ac_recv));
}

TEST_F(ActuatorCommandParrotModuleTests, Torque)
{
  float setPoint1 = 0.15;
  float setPoint2 = -0.15;
  float setPoint3 = 0;

  // Add information, transfer through the serialization process, and check:
  hebiActuatorCommandSetTorque(ac_send, setPoint1);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  ASSERT_EQ(true, hebiActuatorCommandHasTorque(ac_recv));
  EXPECT_FLOAT_EQ(setPoint1, hebiActuatorCommandGetTorque(ac_recv));

  hebiActuatorCommandSetTorque(ac_send, setPoint2);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  ASSERT_EQ(true, hebiActuatorCommandHasTorque(ac_recv));
  EXPECT_FLOAT_EQ(setPoint2, hebiActuatorCommandGetTorque(ac_recv));

  hebiActuatorCommandSetTorque(ac_send, setPoint3);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  ASSERT_EQ(true, hebiActuatorCommandHasTorque(ac_recv));
  EXPECT_FLOAT_EQ(setPoint3, hebiActuatorCommandGetTorque(ac_recv));
}

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

  // Add information and check:
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

  // Add information and check:
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

  // Add information and check:
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
