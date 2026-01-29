

from colorama import Fore, Style
from controller import Controller

def run_cli():

    ctrl = Controller()
    prompt = Fore.WHITE + "▶ " + Style.RESET_ALL

    print(Fore.CYAN + "Controller CLI" + Style.RESET_ALL)
    print(Fore.CYAN + "Type 'quit' or 'exit' to return to REPL." + Style.RESET_ALL)

    while True:
        try:
            cmd = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print(Fore.CYAN + "\nexiting CLI" + Style.RESET_ALL)
            break
        if not cmd:
            continue
        elif cmd in ("quit", "exit"):
            print(Fore.CYAN + "bye." + Style.RESET_ALL)
            break
        try:
            ctrl.process(cmd)
        except Exception as e:
            print(Fore.RED + "{} raised by control: {}".format(type(e), e) + Style.RESET_ALL)

run_cli()

#EOF
