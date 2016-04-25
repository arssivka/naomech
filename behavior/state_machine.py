class State:
    def __init__(self, ctx=None, action=None):
        self.ctx = ctx
        self.action = action

    def run(self):
        raise NotImplementedError

    def update(self, action):
        raise NotImplementedError

    def prepare(self, action):
        pass

    def finalize(self):
        pass


def ActionFactory(name, args=()):
    def __init__(self, **kwargs):
        setattr(self, "action", name)
        for key, value in kwargs.items():
            if key not in args:
                raise TypeError("Argument %s not valid for %s" % (key, self.__class__.__name__))
            setattr(self, key, value)
        Action.__init__(self, name)

    def __eq__(self, other):
        return other and Action.__eq__(self, other) \
               and all(getattr(self, key, None) == getattr(other, key, None) for key in args)

    return type(name, (Action,), {"__init__": __init__, "__eq__": __eq__})


class Action(object):
    def __init__(self, name, **kwargs):
        self.name = name
        for key, value in kwargs.items():
            self.__dict__[key] = value

    def __str__(self):
        return str(self.name)

    def __eq__(self, other):
        return self.name == str(other)

    def __hash__(self):
        return hash(self.name)


class StateMachine:
    def __init__(self, initial_state):
        self.current_state = initial_state

    def update(self, action, run=True):
        state = self.current_state.update(action)
        if self.current_state is not None and self.current_state is not state:
            self.current_state.finalize()
        if self.current_state is not state:
            self.current_state = state
            self.current_state.prepare(action)
        self.current_state = state
        if run:
            self.run()

    def run(self):
        self.current_state.run()

    def stop(self):
        self.current_state.finalize()

