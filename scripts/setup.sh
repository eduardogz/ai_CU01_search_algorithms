pip install virtualenv
virtualenv -p "C:\Program Files (x86)\Python36-32\python.exe" ../venv
source ../venv/Scripts/activate
pip install -r ../requirements/dev.txt
deactivate
