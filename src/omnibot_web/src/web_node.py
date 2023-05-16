from flask import Flask,render_template
app = Flask(__name__)

@app.route("/")
def root():
    return render_template('index.html')

@app.route("/statisticks.html")
def home():
    return render_template('statisticks.html')

@app.route("/navigation.html")
def navigation():
    return render_template('navigation.html')

@app.route("/manipulator.html")
def manipulator():
    return render_template('manipulator.html')

@app.route("/platform.html")
def platfrom():
    return render_template('platform.html')

@app.route("/cartographer.html")
def cartographer():
    return render_template('cartographer.html')

@app.route("/input_data")
def input_data():
    return '{"a":123,"b":333}';


if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8080, debug=False)  