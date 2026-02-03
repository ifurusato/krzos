
import sys
import time

from cardinal import Cardinal
from controller import Controller

# auto-clear: remove cached modules to force reload
for mod in ['test']:
    if mod in sys.modules:
        del sys.modules[mod]

try:

    controller = Controller()

    for cardinal in Cardinal._registry:
        print('cardinal: {}; pixel: {}'.format(cardinal.name, cardinal.pixel))
        controller.process('ring {} orange'.format(cardinal.pixel))
        time.sleep(2.0)
        controller.process('ring {} black'.format(cardinal.pixel))
        time.sleep(0.2)

except KeyboardInterrupt:
    print("Ctrl-C caught.")
except Exception as e:
    print("{} raised: {}".format(type(e), e))
finally: 
    print("complete.")

