from competition import Competition

class Basketball(Competition):
    def __init__(self):
        self.working = 1

if __name__ == '__main__':
    basketballer = Basketball()
    basketballer.run()