from competition import Competition

class TripleJump(Competition):
    def __init__(self, button):
        super().__init__(self, button)
        # st else
    
    def run(self):
        print("running")

if __name__ == '__main__':
    default_button = 1
    triple_jumper = TripleJump(default_button)
    triple_jumper.run()