
from toposort import toposort2


def preempt_graph_to_priorities(preempt_graph, start):
   set_graph = dict(map(lambda x: (x[0], set(x[1])), preempt_graph.iteritems()))
   priomap = {}
   prio = start
   for equals in toposort2(set_graph):
      for equal in equals:
         priomap[equal] = prio
      prio -= 1
   return priomap


def preempt_graph_to_graphviz(preempt_graph, start):
   code = 'graph g {\n'
   code += 'rankdir = "BT";\n'
   for key, prio in toposort_preempt(preempt_graph, start).iteritems():
      code += '%s [label="%s, prio = %d"];\n' % (key, key, prio)
   for key, val in preempt_graph.iteritems():
      for v in val:
         code += '%s -- %s;\n' % (key, v)
   code += '}'
   return code

