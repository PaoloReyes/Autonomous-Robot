import fuzzylite as fl

class FuzzyController:
    def __init__(self) -> None:
        self.engine = fl.Engine(name="VelocityControllers", 
                                description="Fuzzy logic controller for controlling the velocities of the puzzlebot")
        
        self.engine.input_variables = [
            fl.InputVariable(name="x", 
                             description="Position in the x axis", 
                             minimum=-160.0, 
                             maximum=160.0,
                             enabled=True,
                             lock_range=False,
                             terms=[
                                 fl.Gaussian('left', 160.0, 40.0),
                                 fl.Gaussian('center', 0.0, 40.0),
                                 fl.Gaussian('right', -160.0, 40.0)
                             ]),
            fl.InputVariable(name="y",
                             description="Position in the y axis",
                             minimum=0,
                             maximum=50,
                             enabled=True,
                             lock_range=False,
                             terms=[
                                fl.Gaussian('back', 25.0, 10.0),
                                fl.Gaussian('up', 50.0, 10.0)
                             ])
        ]

        self.engine.output_variables = [
            fl.OutputVariable(name="lVel",
                              description="Linear velocity",
                              minimum=0.0,
                              maximum=0.25,
                              enabled=True,
                              lock_range=False,
                              aggregation=None,
                              defuzzifier=fl.WeightedAverage("TakagiSugeno"),
                              default_value=0.0,
                              lock_previous=False,
                              terms=[
                                  fl.Gaussian('full_speed', 0.25, 0.2),
                                  fl.Gaussian('speed', 0.20, 0.2),
                                  fl.Gaussian('mid_speed', 0.15, 0.2),
                                  fl.Gaussian('low_speed', 0.8, 0.2, height=0.8),
                                  fl.Gaussian('stop', 0.0, 0.2)
                              ]),

            fl.OutputVariable(name="aVel",
                                description="Angular velocity",
                                minimum=-1.5,
                                maximum=1.5,
                                enabled=True,
                                lock_range=False,
                                aggregation=None,
                                defuzzifier=fl.WeightedAverage("TakagiSugeno"),
                                default_value=0.0,
                                lock_previous=False,
                                terms=[
                                    fl.Gaussian('lefter', 1.5, 0.2),
                                    fl.Gaussian('left', 0.5, 0.2),
                                    fl.Gaussian('mid', 0.0, 0.2),
                                    fl.Gaussian('right', -0.5, 0.2),
                                    fl.Gaussian('righter', -1.5, 0.2)
                                ])

        ]

        self.engine.rule_blocks = [
            fl.RuleBlock(
                name='',
                description='',
                conjunction=fl.AlgebraicProduct(),
                disjunction=fl.AlgebraicSum(),
                enabled=True,
                activation=fl.General(),
                implication=None,
                rules=[
                    fl.Rule.create("if x is left and y is back then lVel is low_speed and aVel is lefter", self.engine),
                    fl.Rule.create("if x is left and y is up then lVel is mid_speed and aVel is left", self.engine),
                    fl.Rule.create("if x is center and y is back then lVel is mid_speed and aVel is mid", self.engine),
                    fl.Rule.create("if x is center and y is up then lVel is full_speed and aVel is mid", self.engine),
                    fl.Rule.create("if x is right and y is back then lVel is mid_speed and aVel is right", self.engine),
                    fl.Rule.create("if x is right and y is up then lVel is low_speed and aVel is righter", self.engine)
                ],
            )
        ]

    def compute(self, x, y):
        self.engine.input_variable('x').value = x
        self.engine.input_variable('y').value = y
        self.engine.process()
        return self.engine.output_variable('lVel').value, self.engine.output_variable('aVel').value