ML8 Submodule
=============

``{SLOT}``:
    Denotes an I/O module submodule slot 1,2, or 3
``{SLOT_CHANNEL}``:
    Denotes a channel number 1,2,3,4,5,6,7 or 8

Parameter Types
---------------

ml8_speed_select
~~~~~~~~~~~~~~~~

input filter selection::

  Min:         0
  Max:         2
  Options:
     0 = 2 Khz - 50 Khz
     1 = 50 Khz - 100 Khz
     2 = 100 Khz - 200 Khz


ml8_gain
~~~~~~~~

Encoded Analog Gain Value::

  Type:        integer
  Min:         0
  Max:         56
  Options:
      0 dB -> 0
      1 dB to 9 dB not active 
     10 dB -> 10
     11 dB -> 11
     ...
     65 dB -> 65



Addresses
---------


Card type
~~~~~~~~~

::

  Address:     /slot/{SLOT}/type
  Access:      read
  Parameters:  submodule_type, always "ML8"


Input Audio Channel Count
~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/inputs
  Access:      read
  Parameters:  int32_t, always 8

Output Audio Channel Count
~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/outputs
  Access:      read
  Parameters:  int32_t, always 0


Global A/D Converter Reset
~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/ML8/raw/ADC_RESET
  Access:      rw
  Parameters:  bool


Global A/D High Pass Filter Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/ML8/raw/ADC_HPF_ENABLE
  Access:      rw
  Parameters:  bool


Global A/D Filter Speed Select
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/ML8/raw/SPEED_SELECT
  Access:      rw
  Parameters:  ml8_speed_select

Global Phantom Enable
~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/ML8/raw/PHANTO_ENA
  Access:      rw
  Parameters:  bool

Channel DC Servo Enable
~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/ML8/raw/DC{SLOT_CHANNEL}
  Access:      rw
  Parameters:  bool

Channel Common Mode Servo Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/ML8/raw/CM{SLOT_CHANNEL}
  Access:      rw
  Parameters:  bool


Channel PGA Bypass
~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/ML8/raw/FOLLOW{SLOT_CHANNEL}
  Access:      rw
  Parameters:  bool
  Notes:       Change only if gain is set to 0 dB

Channel 18 dB Pad
~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/ML8/raw/PAD{SLOT_CHANNEL}
  Access:      rw
  Parameters:  bool


Channel Phantom Power Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/ML8/raw/PHANTOM{SLOT_CHANNEL}
  Access:      rw
  Parameters:  bool


Channel Gain
~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/ML8/raw/GAIN{SLOT_CHANNEL}
  Access:      rw
  Parameter:   ml8_gain

