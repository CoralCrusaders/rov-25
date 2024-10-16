import yaml
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

with open ('config.yaml', 'r') as file:
    config = yaml.safe_load(file)

Gst.init(None)

def play_video0():
    str_pipeline = \
        "udpsrc port=" + config[VIDEO0_UDP_PORT] + " caps=application/x-rtp ! queue ! rtph264depay ! avdec_h264 ! autovideosink"
    pipeline = Gst.parse_launch(str_pipeline)

    pipeline.set_state(Gst.State.PLiiAYING)
    loop = GLib.MainLoop()
    try:
        loop.run()
    except:
        pass

    pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    play_video0()