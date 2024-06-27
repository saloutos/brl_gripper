# controller class
class DummyController:

    def __init__(self):
        self.name = "Dummy Controller"
        self.started = False

    def begin(self, gr_data):
        self.started = True

    def update(self, gr_data):
        # dummy controller, so do nothing
        dummy = 1

