// Include unit testing code
#include <gtest/gtest.h>

// Note: because feedback is read-only, to unit test we need to connect to a
// module.
#include "hebi_lookup.h"
#include "hebi_module.h"
#include "hebi_command.h"
#include "hebi_feedback.h"
#include "hebi_actuator_feedback.h"

#include "hebi_test_module.h"

// Tests which require connecting to a test module and getting "feedback"
class ActuatorFeedbackModuleTests : public testing::Test
{
  protected:
    HebiModule* module;
    virtual void SetUp()
    {
      printf("Setting up test\n");
      HebiLookup* lookup = hebiCreateLookup();
      sleep(2);
      hebiPrintLookupTable(lookup);
      ASSERT_TRUE(NULL != lookup);
      module = hebiGetModuleFromName(lookup, TEST_MODULE_NAME, TEST_MODULE_GROUP, 5000);
      ASSERT_TRUE(NULL != module);
      // Clear the lookup; we don't need this anymore.
      if (lookup != NULL)
        hebiDeleteLookup(lookup);
    }
  
    virtual void TearDown()
    {
      if (module != NULL)
        hebiReleaseModule(module);
    }
};

TEST_F(ActuatorFeedbackModuleTests, PositionVelocityTorque)
{
  float setP = 0.15;
  float setV = 0.45;
  float setT = 0.25;

  // Start by setting the position:
  HebiCommand* cmd = hebiCommandCreate();
  HebiActuatorCommand* ac = hebiCommandGetActuatorCommand(cmd);
  hebiActuatorCommandSetPosition(ac, setP);
  hebiActuatorCommandSetVelocity(ac, setV);
  hebiActuatorCommandSetTorque(ac, setT);
  ASSERT_TRUE(hebiModuleSendCommandWithAcknowledgement(module, cmd, 500));

  // Check feedback
  HebiFeedback* fbk = hebiFeedbackCreate();
  hebiModuleRequestFeedback(module, fbk, 500);
  HebiActuatorFeedback* af = hebiFeedbackGetActuatorFeedback(fbk);
  ASSERT_TRUE(hebiActuatorFeedbackHasPosition(af));
  EXPECT_FLOAT_EQ(setP, hebiActuatorFeedbackGetPosition(af));
  ASSERT_TRUE(hebiActuatorFeedbackHasVelocity(af));
  EXPECT_FLOAT_EQ(setV, hebiActuatorFeedbackGetVelocity(af));
  ASSERT_TRUE(hebiActuatorFeedbackHasTorque(af));
  EXPECT_FLOAT_EQ(setT, hebiActuatorFeedbackGetTorque(af));

  hebiFeedbackDestroy(fbk);
  hebiCommandDestroy(cmd);
};

TEST_F(ActuatorFeedbackModuleTests, Deflection)
{
  // Check feedback
  HebiFeedback* fbk = hebiFeedbackCreate();
  hebiModuleRequestFeedback(module, fbk, 500);
  HebiActuatorFeedback* af = hebiFeedbackGetActuatorFeedback(fbk);
  // TODO: how can be check deflection/defvel values?
  ASSERT_TRUE(hebiActuatorFeedbackHasDeflection(af));
  hebiActuatorFeedbackGetDeflection(af);
  ASSERT_TRUE(hebiActuatorFeedbackHasDeflectionVelocity(af));
  hebiActuatorFeedbackGetDeflectionVelocity(af);

  hebiFeedbackDestroy(fbk);
};

TEST_F(ActuatorFeedbackModuleTests, Motor)
{
  // Check feedback
  HebiFeedback* fbk = hebiFeedbackCreate();
  hebiModuleRequestFeedback(module, fbk, 500);
  HebiActuatorFeedback* af = hebiFeedbackGetActuatorFeedback(fbk);
  // TODO: how can be check these values?
  ASSERT_TRUE(hebiActuatorFeedbackHasMotorVelocity(af));
  hebiActuatorFeedbackGetMotorVelocity(af);
  ASSERT_TRUE(hebiActuatorFeedbackHasMotorCurrent(af));
  hebiActuatorFeedbackGetMotorCurrent(af);

  hebiFeedbackDestroy(fbk);
};

TEST_F(ActuatorFeedbackModuleTests, Temperature)
{
  // Check feedback
  HebiFeedback* fbk = hebiFeedbackCreate();
  hebiModuleRequestFeedback(module, fbk, 500);
  HebiActuatorFeedback* af = hebiFeedbackGetActuatorFeedback(fbk);
  // TODO: how can be check these values?
  ASSERT_TRUE(hebiActuatorFeedbackHasMotorTemperature(af));
  hebiActuatorFeedbackGetMotorTemperature(af);
  ASSERT_TRUE(hebiActuatorFeedbackHasMotorWindingTemperature(af));
  hebiActuatorFeedbackGetMotorWindingTemperature(af);
  ASSERT_TRUE(hebiActuatorFeedbackHasActuatorTemperature(af));
  hebiActuatorFeedbackGetActuatorTemperature(af);

  hebiFeedbackDestroy(fbk);
};
