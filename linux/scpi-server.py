#!/usr/bin/env python

"""
Run a multi-threaded single-client SCPI Server implemented in Python.

Using a single-client server is sensible for many SCPI servers
where state would need to be shared between the multiple clients
and thus access to it would need to be made thread-safe.
In most cases, this doesn't make sense. Everything is
simply much easier when allowing only one client at a time.

The design choice for a multi-threaded server was made in
order to be able to actively disconnect additional clients
while another one is already connected.

Contains code from https://gist.github.com/pklaus/db709c8c1279348e0638
"""

# Make it work on Python 2 and Python 3:
try:
    import socketserver
    import os
    import string
    import sys
except ImportError:
    import SocketServer as socketserver
import socket, threading
import argparse, random, logging
from logging import DEBUG, INFO, WARNING, ERROR, CRITICAL

logger = logging.getLogger('scpi-server')

class CmdTCPServer(socketserver.ThreadingTCPServer):
    """
    A TCP server made to respond to line based commands.
    """

    #: newline character(s) to be added to string responses
    newline = '\n'
    #: Ctrl-C will cleanly kill all spawned threads
    daemon_threads = True
    #: much faster rebinding possible
    allow_reuse_address = True
    address_family = socket.AF_INET6

    class CmdRequestHandler(socketserver.StreamRequestHandler):
        def handle(self):
            if not self.server.lock.acquire(blocking=False):
                self.log(DEBUG, 'An additional cliend tried to connect from {client}. Denying...')
                return
            self.log(DEBUG, 'Connected to {client}.')
            try:
                while True:
                    self.single_cmd()
            except Disconnected:
                pass
                self.log(DEBUG, 'The client {client} closed the connection')
            finally:
                self.server.lock.release()
        def read_cmd(self):
            return self.rfile.readline().decode('utf-8').strip()
        def log(self, level, msg, *args, **kwargs):
            if type(level) == str:
                level = getattr(logging, level.upper())
            msg = msg.format(client=self.client_address[0])
            logger.log(level, msg, *args, **kwargs)
        def send_reply(self, reply):
            if type(reply) == str:
                if self.server.newline: reply += self.server.newline
                reply = reply.encode('utf-8')
            self.wfile.write(reply)
        def single_cmd(self):
            cmd = self.read_cmd()
            if not cmd: raise Disconnected
            self.log(DEBUG, 'Received a cmd: {}'.format(cmd))
            try:
                reply = self.server.process(cmd)
                self.log(DEBUG, 'reply: {}'.format(reply))
                if reply is not None:
                    self.send_reply(reply)
            except:
                self.send_reply('ERR')

    def __init__(self, server_address, name=None):
        socketserver.TCPServer.__init__(self, server_address, self.CmdRequestHandler)
        self.lock = threading.Lock()
        self.name = name if name else "{}:{}".format(*server_address)

    def process(self, cmd):
        """
        Implement this method to handle command processing.
        For each command, this method will be called.
        Return a string or bytes as appropriate.
        If your the message is only a command (not a query), return None.
        """
        raise NotImplemented

class SCPIServerExample(CmdTCPServer):

    def process(self, cmd):
        """
        This is the method to process each SCPI command
        received from the client.
        """
        if cmd.startswith('*IDN?'):
            return 'Rigol Technologies,MSO2302A,DS1EXXXXXXXXXX,00.02.05.02.00'


        if cmd.startswith(':CHAN1:DISP?'):
            return '1'

        if cmd.startswith(':CHAN2:DISP?'):
            return '0'
        
        
        if cmd.startswith(':LA:STAT?'):
            return '1'
        
               
        if cmd.startswith(':LA:DIG0:DISP?'):
            return '1'
        if cmd.startswith(':LA:DIG1:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG2:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG3:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG4:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG5:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG6:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG7:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG8:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG9:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG10:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG11:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG12:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG13:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG14:DISP?'):
            return '0'
        if cmd.startswith(':LA:DIG15:DISP?'):
            return '0'
        
        if cmd.startswith(':TIM:SCAL?'):
            return '1e-6'
        
        
        if cmd.startswith(':CHAN1:PROB?'):
            return '1'
        if cmd.startswith(':CHAN2:PROB?'):
            return '1'
                
        if cmd.startswith(':CHAN1:SCAL?'):
            return '1'
        if cmd.startswith(':CHAN2:SCAL?'):
            return '1'
                
        if cmd.startswith(':CHAN1:OFFS?'):
            return '0'
        if cmd.startswith(':CHAN2:OFFS?'):
            return '0'

        if cmd.startswith(':CHAN1:COUP?'):
            return 'DC'
        if cmd.startswith(':CHAN2:COUP?'):
            return 'DC'
        
        if cmd.startswith(':TRIG:EDGE:SOUR?'):
            return 'D0'

        if cmd.startswith(':TIM:OFFS?'):
            return '0'

        if cmd.startswith(':TRIG:EDGE:SLOP?'):
            return 'POS'

        if cmd.startswith(':TRIG:EDGE:LEV?'):
            return '0'

        if cmd.startswith(':RUN'):
            return
        
        
        if cmd.startswith('*OPC?'):
            return '1'

        if cmd.startswith(':ACQ:MDEP 0'):
            return
        if cmd.startswith(':ACQ:MDEP 1400'):
            return
    
        if cmd.startswith(':STOP'):
            return
        
        if cmd.startswith(':WAV:FORM BYTE'):
            return
        
        if cmd.startswith(':WAV:MODE NORM'):
            return
        if cmd.startswith(':WAV:MODE RAW'):
            return
        
        if cmd.startswith(':WAV:SOUR CHAN1'):
            return
        
        if cmd.startswith(':WAV:YREF?'):
            return '127'
        
        if cmd.startswith(':WAV:STAT?'):
            return 'IDLE,1400'
        
        
        if cmd.startswith(':CHAN1:DISP OFF'):
            return

        if cmd.startswith(':LA:DIG0:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG1:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG2:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG3:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG4:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG5:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG6:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG7:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG8:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG9:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG10:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG11:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG12:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG13:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG14:DISP OFF'):
            return
        if cmd.startswith(':LA:DIG15:DISP OFF'):
            return
        
        if cmd.startswith(':LA:DIG0:DISP ON'):
            return
        if cmd.startswith(':LA:DIG1:DISP ON'):
            return
        if cmd.startswith(':LA:DIG2:DISP ON'):
            return
        if cmd.startswith(':LA:DIG3:DISP ON'):
            return
        if cmd.startswith(':LA:DIG4:DISP ON'):
            return
        if cmd.startswith(':LA:DIG5:DISP ON'):
            return
        if cmd.startswith(':LA:DIG6:DISP ON'):
            return
        if cmd.startswith(':LA:DIG7:DISP ON'):
            return
        if cmd.startswith(':LA:DIG8:DISP ON'):
            return
        if cmd.startswith(':LA:DIG9:DISP ON'):
            return
        if cmd.startswith(':LA:DIG10:DISP ON'):
            return
        if cmd.startswith(':LA:DIG11:DISP ON'):
            return
        if cmd.startswith(':LA:DIG12:DISP ON'):
            return
        if cmd.startswith(':LA:DIG13:DISP ON'):
            return
        if cmd.startswith(':LA:DIG14:DISP ON'):
            return
        if cmd.startswith(':LA:DIG15:DISP ON'):
            return
        
        if cmd.startswith(':LA:STAT OFF'):
            return 'OFF'
        
        if cmd.startswith(':SING'):
            return
        
        if cmd.startswith(':TRIG:STAT?'):
            return 'STOP'
        
        if cmd.startswith('*ESR?'):
            return '1'
        
         
        
        if cmd.startswith(':WAV:DATA?'):
            replystring = '#90000014001400'+str(''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(4096)))
            return replystring
       
       
       

        


        else:
            sys.exit(cmd)

def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--port', type=int, default=5025, help='TCP port to listen to.')
    parser.add_argument('--host', default='::', help='The host / IP address to listen at.')
    parser.add_argument('--loglevel', default='INFO', help='log level',
        choices=['CRITICAL', 'ERROR', 'WARNING', 'INFO', 'DEBUG'])
    args = parser.parse_args()
    logging.basicConfig(format='%(message)s', level=args.loglevel.upper())
    scpi_server = SCPIServerExample((args.host, args.port))
    try:
        scpi_server.serve_forever()
    except KeyboardInterrupt:
        logger.info('Ctrl-C pressed. Shutting down...')
    scpi_server.server_close()

class Disconnected(Exception): pass

if __name__ == "__main__":
    main()
