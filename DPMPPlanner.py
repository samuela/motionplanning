from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og

from pyDPMP.messagepassing import MaxSumMP, tree_sched
from pyDPMP.mrf import Factor, MRF
from pyDPMP.particleselection import SelectDiverse, SelectLazyGreedy
from pyDPMP.proposals import mixture_proposal
from pyDPMP.util import merge_dicts
from pyDPMP import DPMP_infer

import random


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

class ParamHandler(object):
  def __init__(self, param_vals, name, typ):
    super(ParamHandler, self).__init__()
    self.param_vals = param_vals
    self.name = name
    self.typ = typ

  def setValue(self, v):
    if self.typ == ParamSet.UNKNOWN:
      self.param_vals[self.name] = v
    elif self.typ == ParamSet.BOOL:
      self.param_vals[self.name] = (v == "1")
    elif self.typ == ParamSet.ENUM:
      self.param_vals[self.name] = v
    elif self.typ == ParamSet.INT:
      self.param_vals[self.name] = int(v)
    elif self.typ == ParamSet.DOUBLE:
      self.param_vals[self.name] = float(v)
    else:
      raise Exception("WTF is this shit?")

  def getValue(self):
    return self.param_vals[self.name]

class ParamSet(object):
  UNKNOWN = 0
  BOOL = 1
  ENUM = 2
  INT = 3
  DOUBLE = 4

  def __init__(self, param_defs):
    super(ParamSet, self).__init__()
    self.param_defs = param_defs
    self._param_vals = {k: v[3] for (k, v) in self.param_defs.items()}

  def __getitem__(self, key):
    return ParamHandler(self._param_vals, key, self.param_defs[key][1])

class DPMPPlanner(ob.Planner):
  PARAM_DEFS = {
    'avoidance_weight': ('avoidance_weight', ParamSet.DOUBLE, [0.0, 0.01, 1.0e32], 200.0),
    'clearance': ('clearance', ParamSet.DOUBLE, [0.0, 0.01, 1.0e32], 10.0),
    'max_iters': ('max_iters', ParamSet.INT, [5, 1, 1000], 25),
    'n_particles': ('n_particles', ParamSet.INT, [2, 1, 1000], 10),
    'n_waypoints': ('n_waypoints', ParamSet.INT, [5, 1, 1000], 50),
    'sig_rw': ('sig_rw', ParamSet.DOUBLE, [0.0, 0.01, 1.0e32], 25.0),
    'uniform_proposal_weight': ('uniform_proposal_weight', ParamSet.DOUBLE, [0.0, 0.01, 1.0], 0.25)
  }

  def __init__(self, si):
    super(DPMPPlanner, self).__init__(si, "DPMPPlanner")
    self.param_set = ParamSet(DPMPPlanner.PARAM_DEFS)

    self.sampler_ = si.allocStateSampler()

    self._start_state = None
    self._goal_state = None
    self._xMAP = None
    self._xParticles = None
    self._stats = None

  def solve(self, ptc):
    pdef = self.getProblemDefinition()
    si = self.getSpaceInformation()
    space = si.getStateSpace()
    svc = si.getStateValidityChecker()

    goal = pdef.getGoal()
    self._goal_state = sample_goal(space, goal)

    assert pdef.getStartStateCount() == 1
    self._start_state = pdef.getStartState(0)

    print 'Start/goal distance:', si.distance(self._start_state, self._goal_state)

    def huber(clearance, epsilon):
      if clearance < 0:
        return clearance - epsilon / 2.0
      elif (0 <= clearance) and (clearance <= epsilon):
        return -1.0 / (2.0 * epsilon) * ((clearance - epsilon) ** 2)
      else:
        return 0.0

    T = self.params()['n_waypoints'].getValue()
    n_particles = self.params()['n_particles'].getValue()
    max_iters = self.params()['max_iters'].getValue()
    clearance = self.params()['clearance'].getValue()
    avoidance_weight = self.params()['avoidance_weight'].getValue()
    sig_rw = self.params()['sig_rw'].getValue()
    uniform_proposal_weight = self.params()['uniform_proposal_weight'].getValue()

    nodes = range(T)

    obstacle_pots = {
      'obs_{}'.format(t):
        Factor([t], lambda x_t: avoidance_weight * huber(svc.clearance(x_t), clearance))
      for t in nodes
    }

    def pw_pot(x_u, x_v):
      return -1 * (si.distance(x_u, x_v) ** 2)

    def pw_pot_check_motion(x_u, x_v):
      if si.checkMotion(x_u, x_v):
        return -1 * (si.distance(x_u, x_v) ** 2)
      else:
        return -1e32

    start_pot = {
      'motion_start':
        Factor([0], lambda x_0: pw_pot(self._start_state, x_0))
    }
    motion_pots = {
      'motion_t{}'.format(t): Factor([t, t + 1], pw_pot)
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

    x0 = {t: [interpolate(space,
                          self._start_state,
                          self._goal_state,
                          float(t) / T)]
          for t in mrf.nodes}

    def uniform_proposal(mrf, nAdd, x):
      return {v: [sample_uniform(space, self.sampler_)
                  for _ in range(nAdd[v])]
              for v in mrf.nodes}

    def rw_proposal(mrf, nAdd, x):
      return {v: [sample_gaussian(space,
                                  self.sampler_,
                                  random.choice(x[v]),
                                  sig_rw)
                  for _ in range(nAdd[v])]
              for v in mrf.nodes}

    # def callback(info):
    #   print info['iter']

    self._xMAP, self._xParticles, self._stats = DPMP_infer(
      mrf,
      x0,
      n_particles,
      mixture_proposal([uniform_proposal, rw_proposal],
                       weights=[uniform_proposal_weight,
                                1.0 - uniform_proposal_weight]),
      SelectLazyGreedy(),
      maxsum,
      conv_tol=None,
      max_iters=max_iters,
      # callback=callback,
      verbose=True)

    path_states = ([self._start_state]
                 + [self._xMAP[t] for t in range(T)]
                 + [self._goal_state])
    path = construct_path(si, path_states)
    path_valid = path.check()
    pdef.addSolutionPath(path)
    print 'DPMP path valid:', path_valid

    return ob.PlannerStatus(path_valid, False)

  def getPlannerData(self, data):
    # Add start and goal states
    data.addStartVertex(ob.PlannerDataVertex(self._start_state))
    data.addGoalVertex(ob.PlannerDataVertex(self._goal_state))

    T = self.params()['n_waypoints'].getValue()

    # Add all particles
    for t in range(T):
      for s in self._xParticles[t]:
        data.addVertex(ob.PlannerDataVertex(s))

    # Add MAP path
    data.addEdge(ob.PlannerDataVertex(self._start_state),
                 ob.PlannerDataVertex(self._xMAP[0]))

    for t in range(T - 1):
      data.addEdge(ob.PlannerDataVertex(self._xMAP[t]),
                   ob.PlannerDataVertex(self._xMAP[t + 1]))

    data.addEdge(ob.PlannerDataVertex(self._xMAP[T - 1]),
                 ob.PlannerDataVertex(self._goal_state))

  def clear(self):
    super(DPMPPlanner, self).clear()

  def params(self):
    return self.param_set

# if __name__ == "__main__":
#   ou.RNG.setSeed(1)
#
#   # create an R^2 state space
#   space = ob.RealVectorStateSpace(2)
#
#   # set lower and upper bounds
#   bounds = ob.RealVectorBounds(2)
#   bounds.setLow(-2.5)
#   bounds.setHigh(2.5)
#   space.setBounds(bounds)
#
#   # create a simple setup object
#   ss = og.SimpleSetup(space)
#   # ss.setStateValidityChecker(ValidityChecker(ss.getSpaceInformation()))
#   start = ob.State(space)
#   start()[0] = 0.0
#   start()[1] = -2.0
#   goal = ob.State(space)
#   goal()[0] = 0.0
#   goal()[1] = 2.0
#   ss.setStartAndGoalStates(start, goal, .05)
#
#   # set the planner
#   planner = DPMPPlanner(ss.getSpaceInformation())
