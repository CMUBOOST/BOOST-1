// Include unit testing code
#include <gtest/gtest.h>

#include "hebi_command.h"
#include "hebi_settings.h"
#include "hebi_module.h"
#include "hebi_parrot_module.h"

TEST(SettingsCreateTest, Create)
{
  HebiCommand* cmd = hebiCommandCreate();
  ASSERT_TRUE(NULL != cmd);
  HebiSettings* settings = hebiCommandGetSettings(cmd);
  ASSERT_TRUE(NULL != settings);
  hebiCommandDestroy(cmd);
}

class SettingsParrotModuleTests : public testing::Test
{
  protected:
    HebiCommand* cmd_send;
    HebiSettings* settings_send;
    HebiCommand* cmd_recv;
    HebiSettings* settings_recv;
    HebiParrotModule* pm;

    virtual void SetUp()
    {
      cmd_send = hebiCommandCreate();
      ASSERT_TRUE(NULL != cmd_send);
      settings_send = hebiCommandGetSettings(cmd_send);
      ASSERT_TRUE(NULL != settings_send);
      cmd_recv = hebiCommandCreate();
      ASSERT_TRUE(NULL != cmd_recv);
      settings_recv = hebiCommandGetSettings(cmd_recv);
      ASSERT_TRUE(NULL != settings_recv);
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

TEST_F(SettingsParrotModuleTests, Name)
{
  const char* name1 = "TestModule";
  const char* name2 = "TheEagleHasLanded";
  
  // Add information, transfer through the serialization process, and check:
  hebiSettingsSetName(settings_send, name1);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  EXPECT_EQ(0, strcmp(hebiSettingsGetName(settings_recv), name1));

  // Add information, transfer through the serialization process, and check:
  hebiSettingsSetName(settings_send, name2);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  EXPECT_EQ(0, strcmp(hebiSettingsGetName(settings_recv), name2));
}

TEST_F(SettingsParrotModuleTests, Family)
{
  const char* fam_name1 = "FamilyTest1";
  const char* fam_name2 = "TheBradyBunch";
  
  // Add information, transfer through the serialization process, and check:
  hebiSettingsSetName(settings_send, fam_name1);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  EXPECT_EQ(0, strcmp(hebiSettingsGetName(settings_recv), fam_name1));

  // Add information, transfer through the serialization process, and check:
  hebiSettingsSetName(settings_send, fam_name2);
  hebiParrotModuleParrotCommand(pm, cmd_send, cmd_recv);
  EXPECT_EQ(0, strcmp(hebiSettingsGetName(settings_recv), fam_name2));
}

class SettingsTests : public testing::Test
{
  protected:
    HebiSettings* settings;
    HebiCommand* command;

    virtual void SetUp()
    {
      command = hebiCommandCreate();
      ASSERT_TRUE(NULL != command);
      HebiSettings* settings = hebiCommandGetSettings(command);
      ASSERT_TRUE(NULL != settings);
    }
  
    virtual void TearDown()
    {
      hebiCommandDestroy(command);
    }
};

TEST_F(SettingsTests, ActuatorCmd)
{
  HebiActuatorSettings* as = hebiSettingsGetActuatorSettings(settings);
  ASSERT_TRUE(NULL != as);
}
