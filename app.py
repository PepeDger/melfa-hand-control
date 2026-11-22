"""Local-only control panel. Python 3.9+, standard library, no dependencies."""
import argparse
import hmac
import json
import logging
import secrets
import socket
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from control import Controller, ROOT, load_config
from hand_control import HandControl


class LocalHTTPServer(ThreadingHTTPServer):
    # Windows otherwise permits two HTTPServer instances to share the port,
    # which can route requests to an old panel with another session token.
    allow_reuse_address = False

    def server_bind(self):
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def serve(port=8765, open_browser=True, auto_connect=True):
    log_name = 'melfa.log' if port == 8765 else f'melfa-{port}.log'
    logging.basicConfig(filename=ROOT/log_name, level=logging.INFO,
                        format='%(asctime)s %(levelname)s %(message)s', encoding='utf-8')
    controller = Controller(load_config())
    hand = HandControl(controller)
    token = secrets.token_urlsafe(32)
    origin = f'http://127.0.0.1:{port}'

    def snapshot():
        state = controller.snapshot()
        state['hand_control'] = hand.snapshot()
        return state

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # Never log the session token in the URL.

        def reply(self, status, body, content_type='application/json; charset=utf-8'):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' blob:; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'; connect-src 'self'")
            self.end_headers()
            self.wfile.write(body)

        def trusted_host(self):
            return self.headers.get('Host') == f'127.0.0.1:{port}'

        def authorized(self):
            supplied = self.headers.get('X-Session-Token', '')
            return self.trusted_host() and hmac.compare_digest(supplied, token)

        def do_GET(self):
            if not self.trusted_host():
                return self.reply(403, {'error':'Недопустимый Host'})
            if self.path == '/':
                return self.reply(200, (ROOT/'panel.html').read_bytes(), 'text/html; charset=utf-8')
            if self.path == '/kinematics.js':
                return self.reply(200, (ROOT/'kinematics.js').read_bytes(), 'text/javascript; charset=utf-8')
            if self.path == '/api/state' and self.authorized():
                return self.reply(200, snapshot())
            if self.path == '/api/hand-preview' and self.authorized():
                with hand.lock:
                    jpeg=hand.preview_jpeg
                if jpeg:
                    return self.reply(200,jpeg,'image/jpeg')
                return self.reply(404,{'error':'Кадр камеры ещё не получен'})
            self.reply(404, {'error':'Нет доступа или неизвестный путь'})

        def do_POST(self):
            if not self.authorized() or self.headers.get('Origin') != origin:
                return self.reply(403, {'error':'Нет доступа'})
            try:
                length = int(self.headers.get('Content-Length', '0'))
                maximum=180000 if self.path=='/api/hand-preview' else 4096
                if not 0 < length <= maximum: raise ValueError('Размер запроса')
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict): raise ValueError('Неверный запрос')
                action = self.path
                if action == '/api/hand-packet':
                    return self.reply(200, hand.packet(body))
                if action == '/api/hand-preview':
                    hand.camera_preview(body.get('jpeg'))
                    return self.reply(200,{'ok':True})
                if action in ('/api/move', '/api/home', '/api/speed', '/api/servo',
                              '/api/gripper', '/api/gripper-read', '/api/check-connection',
                              '/api/connect', '/api/diagnose', '/api/limits'):
                    hand.ensure_manual()
                if action == '/api/hand-enable':
                    hand.enable(body.get('real', False), body.get('gripper', False), body.get('acknowledged', False))
                elif action == '/api/hand-disable': hand.pause('Режим кисти выключен', disable=True)
                elif action == '/api/hand-camera':
                    hand.launch_camera(port, token, body.get('camera', 0), body.get('sensitivity', 1.))
                elif action == '/api/hand-input': hand.camera_input(body.get('key'))
                elif action == '/api/hand-options': hand.set_options(body.get('sensitivity'), body.get('gripper'), body.get('step_deg'), body.get('whole_target'), body.get('multi_axis'))
                elif action == '/api/limits': controller.set_motion_limits(body.get('limits'))
                elif action == '/api/heartbeat': controller.heartbeat()
                elif action == '/api/connect': controller.connect()
                elif action == '/api/check-connection': controller.check_connection()
                elif action == '/api/disconnect':
                    hand.pause('Отключение робота', disable=True)
                    controller.disconnect()
                elif action == '/api/arm': controller.arm(body.get('acknowledged'))
                elif action == '/api/speed': controller.set_speed(body.get('percent'), body.get('pose_revision'))
                elif action == '/api/gripper-read': controller.read_gripper()
                elif action == '/api/gripper': controller.set_gripper(body.get('closed'), body.get('pose_revision'))
                elif action == '/api/servo': controller.set_servo(body.get('enabled'), body.get('acknowledged'))
                elif action == '/api/diagnose': controller.diagnose()
                elif action == '/api/move': controller.move(body.get('target'), body.get('pose_revision'))
                elif action == '/api/home': controller.home(body.get('pose_revision'))
                elif action == '/api/stop':
                    hand.pause('STOP из веб-пульта')
                    controller.stop()
                else: raise ValueError('Неизвестное действие')
                self.reply(200, snapshot())
            except Exception as exc:
                self.reply(400, {'error':str(exc)})

    server = LocalHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    url = origin + '/#' + token
    if auto_connect and controller.config.get('auto_connect_read_only', False):
        controller.check_connection()  # Reading only; never acquires control or enables servo.
    print('MELFA RV-3SDB: ' + url, flush=True)
    print(('Read-only connection on startup.' if auto_connect and controller.config.get('auto_connect_read_only', False)
           else 'Started without contacting the robot.') +
          ' Movement requires operator preparation. Ctrl+C closes the application.', flush=True)
    if open_browser:
        threading.Timer(.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        hand.closed = True
        hand.pause('Приложение закрывается', disable=True)
        controller.closed = True
        controller.stop()
        server.server_close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--no-connect', action='store_true', help='Start without contacting the robot')
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535: parser.error('port must be 1024..65535')
    serve(args.port, not args.no_browser, not args.no_connect)
