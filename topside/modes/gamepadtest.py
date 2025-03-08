import inputs

def main():
    while True:
        events = inputs.get_gamepad()
        for event in events:
            if(event.code != 'SYN_REPORT'):
                print(event.ev_type, event.code, event.state)

main()