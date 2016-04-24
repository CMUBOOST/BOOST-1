// Include unit testing code
#include <gtest/gtest.h>

#include "hebi_command.h"
#include "hebi_module.h"

TEST(CommandCreateTest, Create)
{
  HebiCommand* cmd = hebiCommandCreate();
  ASSERT_TRUE(NULL != cmd);
  hebiCommandDestroy(cmd);
}

class CommandTests : public testing::Test
{
  protected:
    HebiCommand* cmd;

    virtual void SetUp()
    {
      cmd = hebiCommandCreate();
      ASSERT_TRUE(NULL != cmd);
    }
  
    virtual void TearDown()
    {
      hebiCommandDestroy(cmd);
    }
};

TEST_F(CommandTests, ActuatorCmd)
{
  HebiActuatorCommand* ac = hebiCommandGetActuatorCommand(cmd);
  ASSERT_TRUE(NULL != ac);
}

TEST_F(CommandTests, LedCmd)
{
  HebiLedCommand* lc = hebiCommandGetLedCommand(cmd);
  ASSERT_TRUE(NULL != lc);
}
