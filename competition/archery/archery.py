from competition import Competition

class Archery(Competition):
    def __init__(self, button):
        super().__init__(button)
        self.working = 1

if __name__ == '__main__':
    default_button = 1
    archer = Archery(default_button)
    archer.run()