from util.actioncommand import stand

from BehaviourTask import BehaviourTask

import numpy as np

class TestNumpy(BehaviourTask):

    def _tick(self):

        # Create a simple NumPy array
        arr = np.array([1, 2, 3, 4, 5])

        # Print the array
        print("NumPy Array:")
        print(arr)

        # Perform some basic operations on the array
        print("\nArray Operations:")
        print("Sum:", np.sum(arr))
        print("Mean:", np.mean(arr))
        print("Maximum:", np.max(arr))
        print("Minimum:", np.min(arr))


        self.world.b_request.actions.body = stand()
