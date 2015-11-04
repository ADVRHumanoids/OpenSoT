#include <example_klampt_controller.h>

int main(int argc, char** argv)
{
    ExampleKlamptController controller;
    KlamptController::JntCommand cmq = controller.computeControl(controller.getPosture());
}
