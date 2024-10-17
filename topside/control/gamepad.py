import typing
from inputs import get_gamepad

class Gamepad:
    def get_events() -> str:
        events = get_gamepad()
        ret: str = ""
        for event in events:
            if(event.code != "SYN_REPORT"):
                ret += f"{event.code}:{event.state},"