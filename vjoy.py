from flask import Flask, request
import json


class VJoy:
    def __init__(self, callback=None):
        self.axes = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0}
        self.buttons = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0,
                        5: 0, 6: 0, 7: 0, 8: 0, 9: 0, 10: 0}

    def setJoyState(self, joy_state):
        self.axes = joy_state['Axes']
        self.buttons = joy_state['Buttons']
        print(self.axes, self.buttons)

    def start_open_loop(self):
        app = Flask(__name__)
        # CORS(app, supports_credentials=True)

        @app.route('/')
        def index():
            with open('./controller.html', 'r') as f:
                return f.read()

        @app.route('/set')
        def set():
            data = request.args.get('joy_state')
            joy_state = json.loads(data)
            self.setJoyState(joy_state)
            return data
        app.run(port=8080, host='0.0.0.0')


if __name__ == '__main__':
    vJoy = VJoy()
    vJoy.start_open_loop()
