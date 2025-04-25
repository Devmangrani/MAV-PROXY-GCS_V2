import os
import sys
from functionality_added_v5 import celery, app

# Add the current directory to the Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from functionality_added_v5 import celery, app

if __name__ == '__main__':
    with app.app_context():
        celery.start()