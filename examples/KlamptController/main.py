import ExampleKlamptController
from collections import Counter
#import pYTask

if __name__ == "__main__":
    controller = ExampleKlamptController.ExampleKlamptController()
    q = controller.getPosture()

    # Main loop
    dq = controller.computeControl(q)
    accumulator = Counter(q.asdict)
    accumulator.update(Counter(dq.asdict()))
    q = ExampleKlamptController.JntMap(accumulator)
    controller.setPosture(q)