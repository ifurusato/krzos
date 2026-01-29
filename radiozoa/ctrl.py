from controller import Controller   # or wherever Controller lives

def run_cli():
    ctrl = Controller()

    print("Controller CLI")
    print("Type 'quit' or 'exit' to return to REPL.")

    while True:
        try:
            cmd = input("▶ ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting CLI")
            break
        if not cmd:
            continue
        elif cmd in ("quit", "exit"):
            print("Bye.")
            break
        try:
            ctrl.process(cmd)
        except Exception as e:
            print("Error:", e)


run_cli()

