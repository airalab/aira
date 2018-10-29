Passing Dynamic Parameters
==========================

In `previous <A_Wink_from_Arduino.html>`_ example we discussed how to create a simple CPS with Arduino. Our CPS is able to do only one task: to blink a led. Now let's expand the example and teach our board to blink blue or red led depending on objective parameters.

.. note::

  The source code of this lesson is `here <https://github.com/airalab/robonomics_tutorials/tree/master/arduino_with_args>`_.

Arduino 
-------

The only difference in Arduino source code is in ``check_input()`` function:

.. code-block:: c

  void check_input(String input) {
    // Turn red led on
    if (cmd("red")) {
      blink(13, 500);
      blink(13, 500);
      blink(13, 500);

    // Turn blue led on
    } else if(cmd("blue")) {
      blink(12, 500);
      blink(12, 500);
      blink(12, 500);
      
    // Call reboot
    } else if (cmd("reboot")) {
      Serial.println("Rebooting...");
      delay(200);
      reboot();
    } 

    stringComplete = false;
    inputString = "";
  }

We've already created a basement for multiple commands input. So all we have to do is replace ``blink`` command with ``red`` and ``blue``. 

And here is the diagram of all connections:

.. image:: ../img/6.png
  :alt: Arduino schema
  :align: center


ROS
---

We can pass arguments via objective which points to rosbag file. Have a look at ``blink.py`` script. The main difference is ``blink()`` method:

.. code-block:: python

  def blink(data):
      ser = serial.Serial('/dev/ttyUSB0', 9600)

      if data.data == "blue":
          rospy.loginfo("Blinking blue...")
          ser.write(b"blue\n")

      if data.data == "red":
          rospy.loginfo("Blinking red...")
          ser.write(b"red\n")
      
      ser.flush()
      ser.close()

      rospy.wait_for_service("liability/finish")
      fin = rospy.ServiceProxy("liability/finish", Empty)
      rospy.loginfo("finishing...")
      fin()

Now ``/blink`` topic has a ``String`` type. You can find prepared rosbags in ``rosbag`` folder. 

AIRA
----

Connect to AIRA client via SSH as described `here <Connecting_via_SSH.md>`_. You can either upload code from your host OS or make a clone from Github.

To build a ros package run the following commands::

  $ mkdir -p ws/src && cd ws/src
  $ cp -r path/to/arduino_blink . 
  $ catkin_init_workspace && cd .. && catkin_make 

And launch

.. code-block:: bash

  $ source devel/setup.bash
  $ rosrun arduino_blink blink.py


Also we need to add rosbag files to IPFS::

  $ ipfs add rosbag/blink_blue.bag
  $ ipfs add rosbag/blink_red.bag

In case we want to light the blue led, create the following a demand and an offer messages::

  $ rostopic pub /lighthouse/infochan/signing/ask robonomics_lighthouse/Ask "model: 'QmSuajKuDiL8A5vhbsQJnVVNwhhC5ni6shSZxNXVWvpikt' \
  objective: 'QmUq7d4jATFnbDgtkv83d3VW9jRqqCRkctZdGUBZE5wGN2' \
  token: '0xbD949595eE52346c225a19724084cE517B2cB735' \
  cost: 1 \
  validator: '0x0000000000000000000000000000000000000000' \
  validatorFee: 0 \
  deadline: 6498193"

  $ rostopic pub /lighthouse/infochan/signing/bid robonomics_lighthouse/Bid "model: 'QmSuajKuDiL8A5vhbsQJnVVNwhhC5ni6shSZxNXVWvpikt'
  objective: 'QmUq7d4jATFnbDgtkv83d3VW9jRqqCRkctZdGUBZE5wGN2'
  token: '0xbD949595eE52346c225a19724084cE517B2cB735'
  cost: 1
  lighthouseFee: 0
  deadline: 6498193"

To light a red one, change ``objective`` to ``QmcoE93MrvAdC789vt6G27WypSimhZxu5ZT2aKy8uviBDM`` in the previous messages.

That's it! Now you are able to pass dynamic parameters to your cyber-physical system agent!
