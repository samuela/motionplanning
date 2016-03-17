#!/usr/bin/env python
from ompl import base as ob
from ompl import geometric as og

import numpy as np

class ValidityChecker(ob.StateValidityChecker):
  def __init__(self, si):
    super(ValidityChecker, self).__init__(si)

  def isValid(self, state):
    return bool(self.clearance(state) > 0.0)

  def clearance(self, state):
    x = state[0]
    y = state[1]
    return np.sqrt(x * x + y * y) - 1

space = ob.RealVectorStateSpace(2)
bounds = ob.RealVectorBounds(2)
bounds.setLow(-2.5)
bounds.setHigh(2.5)
space.setBounds(bounds)

ss = og.SimpleSetup(space)
si = ss.getSpaceInformation()

ss.setStateValidityChecker(ValidityChecker(ss.getSpaceInformation()))

state1 = space.allocState()
si.allocStateSampler().sampleUniform(state1)
state2 = space.allocState()
si.allocStateSampler().sampleUniform(state2)

path = og.PathGeometric(si)
path.append(state1)
path.append(state2)

path.check()
