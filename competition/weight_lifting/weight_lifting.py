from competition import Competition

class WeightLifting(Competition):
    def __init__(self, button):
        super().__init__(self, button)
        # st else

    def run(self):
        print("running")

if __name__ == '__main__':
    default_button = 1
    weight_lifter = WeightLifting(default_button)
    weight_lifter.run()