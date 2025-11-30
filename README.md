# VALOR
VALOR: Vector-based Autonomous Landing & Orientation Regulator




python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
pip install jsbsim numpy pandas matplotlib scipy
pip install jupyter ipykernel
python -m ipykernel install --user --name VALOR

Run baseline suite:

python scripts/run_suite.py

CSV logs will be written to results/logs/. Configure scenarios in data/config.json (wind model, sim timing).


## FlightGear Visualization (optional)

1) Enable streaming in data/config.json:

- Set viz.flightgear.enabled to true
- Optionally adjust host/port/rate

2) Start FlightGear to listen for our generic UDP stream:

- On macOS/Linux:

    bash scripts/launch_fg_generic.sh

- Or run manually (adjust paths/headings as needed):

    fgfs --aircraft=c172p --fdm=external \
         --generic=socket,in,60,127.0.0.1,5501,udp,$PWD/flightgear/Protocols/valor_generic.xml \
         --lat=40.6400 --lon=-73.7800 --altitude=213 --heading=90

3) Run the sim (streams at each step):

    python scripts/run_suite.py

You should see the aircraft on short final in FG moving per our JSBSim state. If FG isn’t running, the sim still runs and logs; the streamer silently drops packets.
