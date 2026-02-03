
import sys, gc
from colorama import Fore, Style
from controller import Controller

# force module reload
for mod in ['main', 'device', 'i2c_slave', 'controller']:
    if mod in sys.modules:
        del sys.modules[mod]
gc.collect()

def run_cli():

    ctrl = Controller()
    prompt = Fore.WHITE + "▶ " + Style.RESET_ALL

    print(Fore.CYAN + "Controller CLI" + Style.RESET_ALL)
    print(Fore.CYAN + "Type 'quit' or 'exit' to return to REPL." + Style.RESET_ALL)

    while True:
        print('🌰 a.')
        try:
            cmd = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print(Fore.CYAN + "\nexiting CLI" + Style.RESET_ALL)
            break
        print('🌰 b.')
        if not cmd:
            print('🌰 c.')
            continue
        elif cmd in ("quit", "exit"):
            print('🌰 d.')
            print(Fore.CYAN + "bye." + Style.RESET_ALL)
            break
        try:
            print("🌰 e. cmd: '{}'".format(cmd))
            ctrl.process(cmd)
            print('🌰 f.')
        except Exception as e:
            print(Fore.RED + "{} raised by control: {}".format(type(e), e) + Style.RESET_ALL)
            sys.print_exception()
        print('🌰 z.')

run_cli()

#EOF
