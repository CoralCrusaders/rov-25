import os, sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)
import gi_stream.py

def main():
    gi_stream.play_video0()

main()