import ExampleKlamptController
from collections import Counter
#import pYTask

if __name__ == "__main__":
    controller = ExampleKlamptController.ExampleKlamptController()
    q = controller.getPosture()

    # Main loop
    dq = controller.computeControl(q)
    q = ExampleKlamptController.JntMap(Counter(q) + Counter(dq))
    controller.setPosture(q)