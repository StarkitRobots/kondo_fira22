# write from player.py
from competition import Competition

class KondoWalk(Competition):
    def __init__(self, button, step_number):
        super().__init__(self, button)
        self.step_number = step_number
        # st else

    def run(self):
        self.step_number = 5
        self.motion.play_Soft_Motion_Slot(name = 'Walk_B_Initial')
        for i in range(self.step_number):
            self.motion.play_Soft_Motion_Slot(name = 'Walk_B_Cycle1')
        return

if __name__ == '__main__':
    default_button = 1
    step_number = 5
    kondo_walker = KondoWalk(default_button, step_number)
    print("running the kondo_walk.py")
    kondo_walker.run()
    print("end of running the kondo_walk.py")