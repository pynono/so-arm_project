"""SO-ARM101 SDK package.

Contains two complementary client libraries:

  driver_sdk.py — Hardware SDK. Speaks the Feetech STS3215 UART protocol
                  over a serial port (pyserial). The controller's backend
                  imports this to drive the physical arm. Students can
                  also use it directly when running their script on the
                  same PC the robot is wired into.

  soarm.py      — Network SDK. Speaks the controller's HTTP API. Students
                  run this on their own laptops over Wi-Fi to talk to the
                  team's controller. Only depends on `requests`.
"""
