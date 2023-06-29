from competition import Competition

class Archery(Competition):
    def __init__(self):
        self.working = 1

if __name__ == '__main__':
    archer = Archery()
    archer.run()