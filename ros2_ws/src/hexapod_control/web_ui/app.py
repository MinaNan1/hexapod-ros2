import os
from flask import Flask, render_template

app = Flask(__name__, static_folder='static', template_folder='templates')

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    # Run on all interfaces so an iPad/phone on the network can connect
    app.run(host='0.0.0.0', port=5000, debug=True)
