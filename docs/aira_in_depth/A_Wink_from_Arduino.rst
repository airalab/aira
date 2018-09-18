A Wink from Arduino
===================

In this section we will build the simplest real cyber-physical system!

We will buy a "wink" from Arduino, e.g. make arduino blink with its onboard led. I work with Arduino Uno, but it should be working with any board with led.

First of all, the source code of this lesson is `here <https://github.com/airalab/robonomics_tutorials/tree/master/arduino_blink>`_.

Arduino 
-------

Let's write some code for the board. We're going to create a basement for our future projects, so it might look a little bit redundantly.

The algorithm is following: the board reads byte by byte from serial port and appends to ``inputString``. If we have a ``\n`` byte, go to check the string. In this case we have only ``blink`` and ``reboot`` commands. In the future we will teach the board new commands.

.. code-block:: c

  void loop() {
    serialEvent();
    if (stringComplete) {
      //Call our input handler function
      check_input(inputString);
    }
  }

  //This function works when we have data comming from the serial port
  void serialEvent() {
    while (Serial.available()) {
      char inChar = (char)Serial.read();

      if ( inChar == '\b') {
        inputString.remove(inputString.length() - 1, inputString.length());
      } else if ( int(inChar) == 13 ) {
        inChar = '\n';
      } else {
        //add current character to input command
        inputString += inChar;
      }

      if (inChar == '\n') {
        stringComplete = true;
      }
    }
  }


Every iteration in the ``loop()`` function we check a new command from serial port. And ``check_input()`` function:

.. code-block:: c

  void check_input(String input) {
    //Turn led on
    if (cmd("blink")) {
      blink(13, 500);
      blink(13, 500);
      blink(13, 500);
      
      //Call reboot
    } else if (cmd("reboot")) {
      Serial.println("Rebooting...");
      delay(200);
      reboot();
    } 

    stringComplete = false;
    inputString = "";
  }

The rest of the code is self-explanatory.

ROS
---

All we have to do is wait for a new liability and do the job. In case your Arduino is connected to something different from ``/dev/ttyUSB0``, you should change it in the code.

.. code-block:: python

  #!/usr/bin/env python
  import rospy
  import serial

  from std_msgs.msg import Empty

  def blink(data):
      rospy.loginfo("Blinking...")
      ser = serial.Serial('/dev/ttyUSB0', 9600)
      ser.write(b"blink\n")

  def main():
      rospy.init_node("blink_node")
      rospy.loginfo("Subscribing...")
      rospy.Subscriber("/blink", Empty, blink)
      rospy.spin()

  if __name__ == '__main__':
      main()

Where does a message in the ``/blink`` topic come from? Remember an objective field from `Basic usage <../basic_usage.md>`_? The objective hash points to rosbag file. This rosbag file will be downloaded and played after new liability is created.

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


Also we need to add a rosbag file to IPFS::

  $ ipfs add rosbag/blink.bag

In the next window we create a demand and then an offer::

  $ rostopic pub /lighthouse/infochan/signing/ask robonomics_lighthouse/Ask "model: 'QmdVAKj4y91Q4ddMUf96AHonrTszMjKFoziZd7V5enonFh' \
  objective: 'QmYYZWNd9esP3YBuuyUBVMH3ymaLDbQFB35S79duYiobcD' \
  token: '0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE' \
  cost: 1 \
  validator: '0x0000000000000000000000000000000000000000' \
  validatorFee: 0 \
  deadline: 6393332"

  $ rostopic pub /lighthouse/infochan/signing/bid robonomics_lighthouse/Bid "model: 'QmdVAKj4y91Q4ddMUf96AHonrTszMjKFoziZd7V5enonFh'
  objective: 'QmYYZWNd9esP3YBuuyUBVMH3ymaLDbQFB35S79duYiobcD'
  token: '0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE'
  cost: 1
  lighthouseFee: 0
  deadline: 6393332 "

Do not forget to change token address and deadline. 

When transaction is mined you should see Arduino's blinking. Our simple agent will finish the liability by itself. Congratulations on the first agent!


