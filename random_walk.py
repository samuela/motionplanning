#!/usr/bin/env python
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og

import numpy as np
import matplotlib.pyplot as plt

class DPMPPlanner(ob.Planner):
  def __init__(self, si):
    super(DPMPPlanner, self).__init__(si, "DPMPPlanner")
    self.states_ = []
    self.sampler_ = si.allocStateSampler()

  def solve(self, ptc):
    pdef = self.getProblemDefinition()
    goal = pdef.getGoal()
    si = self.getSpaceInformation()
    pi = self.getPlannerInputStates()

    st = pi.nextStart()
    while st:
      self.states_.append(st)
      st = pi.nextStart()
    print self.states_

    solution = None
    approxsol = 0
    approxdif = 1e6
    while not ptc():
      rstate = si.allocState()
      # pick a random state in the state space
      self.sampler_.sampleUniform(rstate)
      # check motion
      if si.checkMotion(self.states_[-1], rstate):
        self.states_.append(rstate)
        sat = goal.isSatisfied(rstate)
        dist = goal.distanceGoal(rstate)
        if sat:
          approxdif = dist
          solution = len(self.states_)
          break
        if dist < approxdif:
          approxdif = dist
          approxsol = len(self.states_)
    solved = False
    approximate = False
    if not solution:
      solution = approxsol
      approximate = True
    if solution:
      path = og.PathGeometric(si)
      for s in self.states_[:solution]:
        path.append(s)
      pdef.addSolutionPath(path)
      solved = True
    return ob.PlannerStatus(solved, approximate)

  def clear(self):
    super(DPMPPlanner, self).clear()
    self.states_ = []

def isStateValid(state):
  x = state[0]
  y = state[1]
  return x * x + y * y > 1

def plan():
  # create an R^3 state space
  space = ob.RealVectorStateSpace(2)

  # set lower and upper bounds
  bounds = ob.RealVectorBounds(2)
  bounds.setLow(-2.5)
  bounds.setHigh(2.5)
  space.setBounds(bounds)

  # create a simple setup object
  ss = og.SimpleSetup(space)
  ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
  start = ob.State(space)
  start()[0] = 0.0
  start()[1] = -2.0
  goal = ob.State(space)
  goal()[0] = 0.0
  goal()[1] = 2.0
  ss.setStartAndGoalStates(start, goal, .05)

  # set the planner
  planner = DPMPPlanner(ss.getSpaceInformation())
  ss.setPlanner(planner)

  result = ss.solve(10.0)
  if result:
    if result.getStatus() == ob.PlannerStatus.APPROXIMATE_SOLUTION:
      print("Solution is approximate")
    # try to shorten the path
    ss.simplifySolution()

    path = ss.getSolutionPath()
    print(path)
    plot_solution(path)

def plot_solution(path):
  plt.figure(figsize=(8, 8))
  plt.axis([-2.5, 2.5, -2.5, 2.5])
  an = np.linspace(0, 2 * np.pi, 100)
  plt.plot(np.cos(an), np.sin(an))

  points = [(s[0], s[1]) for s in path.getStates()]
  plt.plot(*zip(*points), marker='o')

  plt.xlabel('x_1')
  plt.ylabel('x_2')
  plt.title('Random walk path')

  plt.show()

if __name__ == "__main__":
  plan()
