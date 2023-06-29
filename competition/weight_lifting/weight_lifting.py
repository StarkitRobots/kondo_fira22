from competition import Competition

class WeightLifting(Competition):
    def __init__(self):
        self.working = 1

if __name__ == '__main__':
    triple_jumper = WeightLifting()
    triple_jumper.run()