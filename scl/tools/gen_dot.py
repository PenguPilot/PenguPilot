import yaml
import sys
import zmq


def generate_dot_code():

   data = yaml.load(sys.stdin)

   components = data['components']
   connections = data['connections']

   dot_code = 'graph System\n{\n   ranksep=1.0;\n   nodesep=1.0\n'

   for component in components:
      compname =  component['name']
      color = 'cornflowerblue'
      dot_code += '   %s [shape=record, style=filled, fontname="Helvetica-Outline", label=\"%s | {' % (compname, compname.upper())
      for gate in component['gates']:
         gate_name, gate_type = gate.items()[0]
         dot_code += '<%s> %s | ' % (gate_name, gate_name + '[' + gate_type + ']')
      dot_code = dot_code[0:-3]
      dot_code += '}\"];\n'
   
   dot_code += '\n'

   for connection in connections:
      a = connection[0].split('.')
      b = connection[1].split('.')
      dot_code += '   %s:%s -- %s:%s;\n' % (a[0], a[1], b[0], b[1])

   return dot_code + '}\n'


print generate_dot_code()
