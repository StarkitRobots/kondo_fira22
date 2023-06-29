from competition import Competition

class TripleJump(Competition):
    def __init__(self):
        self.working = 1

if __name__ == '__main__':
    triple_jumper = TripleJump()
    triple_jumper.run()