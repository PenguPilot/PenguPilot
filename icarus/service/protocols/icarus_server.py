

# ICARUS server
###############
#
# purpose: responsible for receiving, delegating and IcarusReping incoming commands
#
# Author: Tobias Simon, Ilmenau University of Technology


from threading import Thread
from icarus_pb2 import IcarusReq, IcarusRep, OK, E_SYNTAX, E_SEMANTIC


class ICARUS_Exception(Exception):

   def __init__(self, msg):
      self.msg = msg


class ICARUS_Server(Thread):

   '''
   ICARUS server
   responsible for receiving, delegating and IcarusReping incoming commands
   '''

   def __init__(self, socket, delegate):
      '''
      socket: a zmq socket
      delegate: object providing handle(IcarusReq) routine, raising ICARUS_Exception
      '''
      Thread.__init__(self)
      self._socket = socket
      self._delegate = delegate
      self.daemon = True


   def run(self):
      '''
      receives, parses and executes commands using the submited delegate in a loop
      '''
      while True:
         # receive message via SCL:
         try:
            data = self._socket.recv()
         except:
            # it would not make sense to send an error message here,
            # as something seems to be wrong with the socket
            print 'could not read SCL message'
            continue
         # parse message into protobuf structure:
         req = IcarusReq()
         try:
            req.ParseFromString(data)
         except:
            # syntactic error in ParseFromString
            self.send_err(E_SYNTAX, 'could not parse protobuf payload')
            continue
         # handle parsed protobuf message and send IcarusRep:
         try:
            self._delegate.handle(req)
            self.send_ok()
         except ICARUS_Exception, ex:
            # semantic error:
            self.send_err(E_SEMANTIC, ex.msg)


   def send_err(self, code, msg):
      '''
      IcarusRep with error code and message
      '''
      rep = IcarusRep()
      rep.status = code
      rep.message = msg
      self._send_rep(rep)


   def send_ok(self):
      '''
      IcarusRep with OK message
      '''
      rep = IcarusRep()
      rep.status = OK
      self._send_rep(rep)


   def _send_rep(self, rep):
      '''
      serialize and send message via _socket
      '''
      self._socket.send(rep.SerializeToString())

