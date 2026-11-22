"""Only one local panel may own the HTTP port."""
from http.server import BaseHTTPRequestHandler
import unittest
from app import LocalHTTPServer


class ServerTests(unittest.TestCase):
    def test_second_panel_cannot_share_the_port(self):
        first=LocalHTTPServer(('127.0.0.1',0),BaseHTTPRequestHandler)
        try:
            with self.assertRaises(OSError):
                second=LocalHTTPServer(first.server_address,BaseHTTPRequestHandler)
                second.server_close()
        finally: first.server_close()


if __name__=='__main__': unittest.main()
