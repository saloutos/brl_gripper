
class BaseState:

    def __init__(self):
        self.name = "Base"
        self.enabled = 0

    def enter(self, MP):
        self.enabled = 1

    def exit(self, MP):
        self.enabled = 0

    def execute(self, MP):
        next_state = self.name
        return next_state

    # TODO: add state helper functions here to avoid mismatches and excessive copying in helpers between similar classes?