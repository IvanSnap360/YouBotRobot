import json
import random
from flask import Flask, render_template
import psutil
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


@app.route("/battery_1")
def battery_1_upd_f():
    return json.dumps({"cell_1": random.randint(0, 100), "cell_2": random.randint(0, 100), "cell_3": random.randint(0, 100)})


@app.route("/battery_2")
def battery_2_upd_f():
    return json.dumps({"cell_1": random.randint(0, 100), "cell_2": random.randint(0, 100), "cell_3": random.randint(0, 100)})


@app.route("/sys_info")
def get_sys_info():
    cpufreq = psutil.cpu_freq()
    svmem = psutil.virtual_memory()

    def get_cpu_temp():
        path = "/sys/class/thermal/thermal_zone0/temp"
        f = open(path, "r")
        temp_raw = int(f.read().strip())
        temp_cpu = float(temp_raw / 1000.0)
        return temp_cpu

    data = {
        "cpu_freq": cpufreq.current,
        "cpu_load": psutil.cpu_percent(),
        "mem_usage": svmem.percent,
        "cpu_temperature": get_cpu_temp() 
    }

    return json.dumps(data)


if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8080, debug=False)
