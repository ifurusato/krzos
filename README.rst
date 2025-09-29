*******************************************
KRZOS: Operating System for the KRZ04 Robot
*******************************************

**KRZOS** provides the core functionality of the *KRZ04 Robot*, a Raspberry
Pi-based robot OS written in Python 3.

* **Notice**
    As of 2025-08-30 the KRZ04 project has been retired, with development
    now focused on its successor, the KRZ04. This repository was previous
    named "krzos" but has been renamed "krz03-os", with the KRZ04 taking
    over the krzos repository name.


.. figure:: https://service.robots.org.nz/wiki/attach/KRZ04/krz04-model.png
   :width: 1200px
   :align: center
   :alt: A 3D model of the KRZ04 Robot

   The 3D model of the KRZ04 Robot, used to plan the hardware build.

|

Background
**********

The *KRZOS* library provides essential support designed as the basis of a
`Behaviour-Based Robot (BBR) <https://en.wikipedia.org/wiki/Behavior-based_robotics>`_.
This library is relatively "low-level" and, in theory, could be used for any Python 3
based robot.


Functional Description
**********************

The basic function is for sensors or other data sources to act as "Publishers" in a
"Publish-Subscribe" model, firing messages onto an asynchronous message bus. Messages
contain an Event type and a data payload. Subscribers to the message bus can filter
which event types they are interested in. The flow of messages are thus filtered
by the Subscribers, who pass on to an Arbitrator messages they have consumed. Once all
Subscribers have acknowledged a message it is passed to a Garbage Collector (a specialised
Subscriber).

Subscribers can themselves act upon received messages, though generally these types of
reactions are typically reflected as direct outputs such as lighting and sound effects,
or used to monitor sensor thresholds, which upon reaching may themselves publish messages
to that effect, such as low battery warnings, or high-priority bumper events.

The robot's movement is not controlled by Subscribers but by higher-level Behaviours,
which are all Subscribers and sometimes even Publishers. Some Behaviours are characterised
as "servo", meaning they may execute continually (when active) and their effect on the
robot may be intermixed with other Behaviours. Some Behaviours are "ballistic", meaning
they take over the complete function of the robot during the duration of their activity,
like a human reflex action, a hand touching a hot stove.

For example, a Roam (servo) Behaviour may be running, with the robot moving freely across
the landscape. It subscribes to a DistanceSensorsPublisher which publishes proximity and
bumper events, and also monitors a MotorControllerPublisher for motor change events. If
Roam receives a message either indicating a bumper has been triggered or the motors have
stopped (due to being too close to an obstacle), Roam is suppressed and an Avoid (ballistic)
Behaviour is released. The robot will begin whatever the Avoid Behaviour entails, perhaps
stopping, backing up while turning clockwise, then suppressing itself and releasing Roam
to proceed again on a new trajectory.


Software Features
*****************

* message and event handling
* an asynchronous message bus that forms the basis of a `Subsumption Architecture <https://en.wikipedia.org/wiki/Subsumption_architecture>`_ [#f1]_, with an "exactly-once' message delivery guarantee
* YAML-based configuration
* timestamped, multi-level, colorised [#f2]_ logging
* written in Python 3 (currently 3.11.2)

.. [#f1] Uses finite state machines, an asynchronous message bus, an arbitrator and controller for task prioritisation.
.. [#f2] Colorised console output tested only on Unix/Linux operating systems.


Hardware Features
*****************

.. figure:: https://service.robots.org.nz/wiki/attach/KRZ04/krz04-initial.jpg
   :width: 1200px
   :align: center
   :alt: The KRZ04 Robot

   The KRZ04 Robot on its first day.

TBD.


Requirements
************

This library requires Python 3.8.5 or newer. It's currently being written using
Python 3.11.2. Some portions (modules) of the KRZOS code will only run on a
Raspberry Pi, though KRZOS Core should function independently of the various Pi
libraries.

KRZOS requires installation of a number of dependencies (support libraries).
There is currently no dependency management set up for this project.

First::

  sudo apt install python3-pip

then:

* sphinx:       https://www.sphinx-doc.org/en/master/index.html
    with:         sudo apt-get install python3-sphinx
     and:         sudo pip3 install sphinx_rtd_theme --break-system-packages
* pytest:       https://docs.pytest.org/en/stable/getting-started.html
    with:         sudo apt install python3-pytest
* numpy:        https://numpy.org/
    with:         sudo apt install python3-numpy
* psutil:       https://pypi.org/project/psutil/
    with:         sudo apt install python3-psutil
* pyyaml:       https://pypi.org/project/PyYAML/
    with:         sudo apt install python3-yaml
* colorama:     https://pypi.org/project/colorama/
    with:         sudo apt install python3-colorama
* luma:         https://pypi.org/project/luma.core/
    with:         sudo pip3 install luma.core --break-system-packages
    (used only for the Monitor)
* ina260:       https://pypi.org/project/ina260/
    with:         sudo pip3 install ina260 --break-system-packages
* ltr559:       https://pypi.org/project/ltr559/
    with:         sudo pip3 install ltr559 --break-system-packages

If a Luxonis OAK-D Lite camera is attached, install:

* depthai:      https://github.com/luxonis/depthai-python
    with:         sudo pip3 install depthai --break-system-packages

The Radiozoa board contains eight VL53L0X sensors. This can be connected to the
default I2C bus 1, or configured to operate on the alternate I2C bus 0 (which is
the configuration choice set in config.yaml).

In order to enable I2C bus 0 you must add the following to /boot/firmware/config.txt::

    dtparam=i2c_vc=on

This line enables I2C bus 0, which by default uses GPIO 0 (SDA) and GPIO 1 (SCL).
Note that if you are using any HAT with an EEPROM, such as a Raspberry Pi Sense HAT,
I2C bus 0 is not available. But because this project uses a lot of GPIO pins required
by the Sense HAT, they cannot be used together in any case.

You can likely also disable the cups.service, avahi-daemon and other services
to free up resources:

    #!/bin/bash
    sudo systemctl disable avahi-daemon
    sudo systemctl mask avahi-daemon
    sudo systemctl disable cups.service
    sudo systemctl mask cups.service
    sudo systemctl disable triggerhappy.service
    sudo systemctl mask triggerhappy.service
    sudo systemctl disable ModemManager.service
    sudo systemctl mask ModemManager.service

Status
******

* 2025-08-30: the KRZ03 project is retired, as development is now focused
  on its successor, the KRZ04.

* 2025-08-29: the shipment of parts from goBILDA arrives.

* 2024-2025: the design begins as a 3D model in OnShape.


Support & Liability
*******************

This project comes with no promise of support or acceptance of liability. Use at
your own risk.


Copyright & License
*******************

All contents (including software, documentation and images) Copyright 2020-2025
by Murray Altheim. All rights reserved.

Software and documentation are distributed under the MIT License, see LICENSE
file included with project.


