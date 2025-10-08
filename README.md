# VALOR
VALOR: Vector-based Autonomous Landing & Orientation Regulator




python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
pip install jsbsim numpy pandas matplotlib scipy
pip install jupyter ipykernel
python -m ipykernel install --user --name VALOR  