import subprocess
import sys
import os

cwd = os.getcwd()

def install():
    subprocess.call([sys.executable, '-m', 'pip', 'install', '-r', f'{cwd}/Scripts/requirements.txt'])


if __name__ == '__main__':
    install()
