# write from player.py
from competition import Competition

class BalancingTest(Competition):
    def __init__(self, button):
        super().__init__(self, button)
        # st else

    def run(self):
        # need to write
        print("in balance tester")

if __name__ == '__main__':
    default_button = 1
    balance_tester = BalancingTest(default_button)
    print("running the kondo_walk.py")
    balance_tester.run()
    print("end of running the kondo_walk.py")