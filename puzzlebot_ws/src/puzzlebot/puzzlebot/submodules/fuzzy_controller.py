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
                                 fl.Sigmoid('lefter', 30, 0.2),
                                 fl.Gaussian('left', 15.0, 5.0),
                                 fl.Gaussian('right', -15.0, 5.0),
                                 fl.Sigmoid('righter', -30, -0.2),
                             ]),
            fl.InputVariable(name="y",
                             description="Position in the y axis",
                             minimum=0,
                             maximum=50,
                             enabled=True,
                             lock_range=False,
                             terms=[
                                fl.Sigmoid('down', 15, -0.6),
                                fl.Gaussian('mid', 22, 10.0),
                                fl.Sigmoid('up', 30.0, 0.6)
                             ])
        ]

        self.engine.output_variables = [
            fl.OutputVariable(name="lVel",
                              description="Linear velocity",
                              minimum=0.0,
                              maximum=0.30,
                              enabled=True,
                              lock_range=False,
                              aggregation=None,
                              defuzzifier=fl.WeightedAverage("TakagiSugeno"),
                              default_value=0.0,
                              lock_previous=False,
                              terms=[
                                fl.Constant('low_speed', 0.05),
                                fl.Constant('mid_speed', 0.20),
                                fl.Constant('speed', 0.35)
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
                                    fl.Constant('left', 1.0),
                                    fl.Constant('mid_left', 0.5),
                                    fl.Constant('mid_right', -0.5),
                                    fl.Constant('right', -1.0),
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
                    fl.Rule.create("if x is lefter and y is down then lVel is low_speed and aVel is left", self.engine),
                    fl.Rule.create("if x is lefter and y is mid then lVel is mid_speed and aVel is left", self.engine),
                    fl.Rule.create("if x is lefter and y is up then lVel is speed and aVel is left", self.engine),

                    fl.Rule.create("if x is left and y is down then lVel is low_speed and aVel is mid_left", self.engine),
                    fl.Rule.create("if x is left and y is mid then lVel is mid_speed and aVel is mid_left", self.engine),
                    fl.Rule.create("if x is left and y is up then lVel is speed and aVel is mid_left", self.engine),
                    fl.Rule.create("if x is right and y is down then lVel is low_speed and aVel is mid_right", self.engine),
                    fl.Rule.create("if x is right and y is mid then lVel is mid_speed and aVel is mid_right", self.engine),
                    fl.Rule.create("if x is right and y is up then lVel is speed and aVel is mid_right", self.engine),

                    fl.Rule.create("if x is righter and y is down then lVel is low_speed and aVel is right", self.engine),
                    fl.Rule.create("if x is righter and y is mid then lVel is mid_speed and aVel is right", self.engine),
                    fl.Rule.create("if x is righter and y is up then lVel is speed and aVel is right", self.engine)
                ],
            )
        ]
        
    def compute(self, x, y):
        self.engine.input_variable('x').value = x
        self.engine.input_variable('y').value = y
        self.engine.process()
        return self.engine.output_variable('lVel').value, self.engine.output_variable('aVel').value