# header


# imports?



# define FSM class
class FiniteStateMachine:

    def __init__(self, states=[]):
        # TODO: assert that states is not empty (at least one state is given)
        self.states = dict(zip([state.name for state in states], states))
        self.current_state = ""
        self.next_state = states[0].name # the first state in the list is the starting state

    def begin(self,MP):
        # TODO: add any other initialization code here?
        self.states[self.next_state].enter(MP)
        self.current_state = self.next_state

    def update(self,MP):

        # TODO: check for user inputs to change state as e-stops??
        # if there is a user input, execute -> transition -> execute ??
        # if there is not a user input, execute -> transition ??

        # execute FSM 
        # TODO: try except here to go back to gravity comp if there is an error?
        self.next_state = self.states[self.current_state].execute(MP) # check for user inputs within functions

        # if there was a valid state transition, update state
        if (self.next_state != self.current_state):
            if (self.next_state in self.states.keys()):
                # handle transition
                # TODO: add better error handling here
                self.states[self.current_state].exit(MP)
                self.states[self.next_state].enter(MP)
                # reset state tracking
                self.current_state = self.next_state
            else:
                # TODO: need to test this
                print("Attempted a bad state transition. " + self.next_state + " is not a valid state name!")