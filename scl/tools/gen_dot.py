import yaml
import sys
import zmq


data = yaml.load(sys.stdin)

print 'graph System\n{\n   ranksep=1.0;\n   nodesep=1.0\n'

for component in data:
   comp_name = component['name']
   for gate in component['sockets']:
      gate_name, gate_type = gate.items()[0]
      gate_name = 'g_' + gate_name
      print '%s -- %s\n' % (comp_name, gate_name)

print '}\n'

