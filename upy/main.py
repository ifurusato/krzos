import time

print("waiting 5 seconds for REPL connect or skip...")
time.sleep(5)

try:
    import uart_slave_app
    uart_slave_app.exec()
except Exception as e:
    import sys
    sys.print_exception(e)
