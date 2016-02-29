#!/usr/bin/env python
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og

import matplotlib.pyplot as plt
import numpy as np
import random
import time

from pyDPMP.messagepassing import MaxSumMP, tree_sched
from pyDPMP.mrf import Factor, MRF
from pyDPMP.particleselection import SelectDiverse, SelectLazyGreedy
from pyDPMP.proposals import mixture_proposal
from pyDPMP.util import merge_dicts
from pyDPMP import DPMP_infer


def sample_uniform(ss, sampler):
  """Sample a state uniformly at random in StateSpace `ss` with StateSampler
  `sampler`."""
  s = ss.allocState()
  sampler.sampleUniform(s)
  return s

def sample_gaussian(ss, sampler, mean, stddev):
  """Sample a state from a spherical Guassian distribution centered at `mean`
  with standard deviation `stddev`."""
  s = ss.allocState()
  sampler.sampleGaussian(s, mean, stddev)
  return s

def interpolate(ss, from_state, to_state, t):
  """Interpolate between States `from_state` and `to_state` in StateSpace
  `ss`."""
  s = ss.allocState()
  ss.interpolate(from_state, to_state, t, s)
  return s

def sample_goal(ss, goal):
  s = ss.allocState()
  goal.sampleGoal(s)
  return s

def construct_path(si, states):
  path = og.PathGeometric(si)
  for s in states:
    path.append(s)
  return path

class DPMPPlanner(ob.Planner):
  def __init__(self, si, n_waypoints, n_particles):
    super(DPMPPlanner, self).__init__(si, "DPMPPlanner")
    self.n_waypoints = n_waypoints
    self.n_particles = n_particles
    self.sampler_ = si.allocStateSampler()

  def solve(self, ptc):
    pdef = self.getProblemDefinition()
    si = self.getSpaceInformation()
    space = si.getStateSpace()
    svc = si.getStateValidityChecker()

    goal = pdef.getGoal()
    goal_state = sample_goal(space, goal)

    assert pdef.getStartStateCount() == 1
    start_state = pdef.getStartState(0)

    def huber(clearance, epsilon):
      if clearance < 0:
        return clearance - epsilon / 2.0
      elif (0 <= clearance) and (clearance <= epsilon):
        return -1.0 / (2.0 * epsilon) * ((clearance - epsilon) ** 2)
      else:
        return 0.0

    T = self.n_waypoints
    nodes = range(T)

    obstacle_pots = {
      'obs_{}'.format(t):
        Factor([t], lambda x_t: huber(svc.clearance(x_t), 0.1))
      for t in nodes
    }

    start_pot = {
      'motion_start':
        Factor([0], lambda x_0: -1 * si.distance(start_state, x_0) ** 2)
    }
    motion_pots = {
      'motion_t{}'.format(t):
        Factor([t, t + 1], lambda x_u, x_v: -1 * si.distance(x_u, x_v) ** 2)
      for t in range(T - 1)
    }
    goal_pot = {
      'motion_goal':
        Factor([T - 1], lambda x_T: -1 * goal.distanceGoal(x_T) ** 2)
    }

    # Create the MRF
    mrf = MRF(nodes, merge_dicts(
      obstacle_pots,
      start_pot,
      motion_pots,
      goal_pot))

    # Set up message passing
    maxsum = MaxSumMP(mrf, sched=tree_sched(mrf, 'motion_start'))

    x0 = {t: [interpolate(space, start_state, goal_state, float(t) / T)]
          for t in mrf.nodes}

    def uniform_proposal(mrf, nAdd, x):
      return {v: [sample_uniform(space, self.sampler_)
                  for _ in range(nAdd[v])]
              for v in mrf.nodes}

    def rw_proposal(mrf, nAdd, x):
      return {v: [sample_gaussian(space, self.sampler_, random.choice(x[v]), 0.25)
                  for _ in range(nAdd[v])]
              for v in mrf.nodes}

    # Turn on matplotlib's interactive mode
    plt.ion()

    plt.figure(figsize=(8, 8))
    def callback(info):
      print info['iter']
      x = info['x']
      xMAP = info['xMAP']

      plt.clf()
      plt.axis([-2.5, 2.5, -2.5, 2.5])
      an = np.linspace(0, 2 * np.pi, 100)
      plt.plot(np.cos(an), np.sin(an))

      particles = [(x_t[0], x_t[1]) for t in range(T) for x_t in x[t]]
      c = [float(t) / float(T) for t in range(T) for x_t in x[t]]
      plt.scatter(*zip(*particles), s=10, c=c, marker='x')

      MAPparticles = [start_state] + [xMAP[t] for t in range(T)] + [goal_state]
      plt.plot(*zip(*[(s[0], s[1]) for s in MAPparticles]), marker='o')

      plt.xlabel('x_1')
      plt.ylabel('x_2')
      plt.title('Iter. %d' % info['iter'])

      plt.draw()

      time.sleep(0.25)

    xMAP, xParticles, stats = DPMP_infer(
      mrf,
      x0,
      self.n_particles,
      mixture_proposal([uniform_proposal, rw_proposal], weights=[0.0, 1.0]),
      SelectLazyGreedy(),
      maxsum,
      conv_tol=None,
      max_iters=25,
      callback=callback,
      verbose=False)

    # Turn off matplotlib's interactive mode
    plt.ioff()

    path_states = [start_state] + [xMAP[t] for t in range(T)] + [goal_state]
    path = construct_path(si, path_states)
    path_valid = path.check()

    pdef.addSolutionPath(path)
    return ob.PlannerStatus(path_valid, False)

  def clear(self):
    super(DPMPPlanner, self).clear()

class ValidityChecker(ob.StateValidityChecker):
  def __init__(self, si):
    super(ValidityChecker, self).__init__(si)

  def isValid(self, state):
    # Convert np.bool_ to standard Python bool
    return bool(self.clearance(state) > 0.0)

  def clearance(self, state):
    x = state[0]
    y = state[1]
    return np.sqrt(x * x + y * y) - 1

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
  ss.setStateValidityChecker(ValidityChecker(ss.getSpaceInformation()))
  start = ob.State(space)
  start()[0] = 0.0
  start()[1] = -2.0
  goal = ob.State(space)
  goal()[0] = 0.0
  goal()[1] = 2.0
  ss.setStartAndGoalStates(start, goal, .05)

  # set the planner
  planner = DPMPPlanner(ss.getSpaceInformation(), 15, 50)
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
  plt.title('Solved path')

  plt.show()

if __name__ == "__main__":
  ou.RNG.setSeed(1)
  plan()
