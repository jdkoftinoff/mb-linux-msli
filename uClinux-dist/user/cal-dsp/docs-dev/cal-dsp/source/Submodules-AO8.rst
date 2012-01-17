A08 Submodule
=============

``{SLOT}``:
    Denotes an I/O module submodule slot 1,2, or 3
``{AO8_PAIR}``:
    Denotes a channel pair: "12", "34", "56", or "78"
``{SLOT_CHANNEL}``:
    Denotes a channel number 1,2,3,4,5,6,7 or 8

Parameter Types
---------------


ao8_volume
~~~~~~~~~~

Encoded D/A Analog Volume::
  Type:        int32_t
  Min:         0
  Max:         255
  Notes:       todo


Addresses
---------

Card type
~~~~~~~~~

::

  Address:     /slot/{SLOT}/type
  Access:      read
  Parameters:  submodule_type, always "AO8"


Input Audio Channel Count
~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/ins
  Access:      read
  Parameters:  int32_t, always 0

Output Audio Channel Count
~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/outs
  Access:      read
  Parameters:  int32_t, always 8


Global D/A Converter Reset
~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AO8/raw/DAC_RESET
  Access:      rw
  Parameters:  bool



Global Link Mute
~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AO8/raw/LINK_MUTE
  Access:      rw
  Parameters:  bool


Channel Pair Auto Mute
~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AO8/raw/AUTOMUTE{AO8_PAIR}
  Access:      rw
  Parameters:  bool

Channel Pair Filter Select
~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AO8/raw/FILTER{AO8_PAIR}
  Access:      rw
  Parameters:  bool


Channel Pair Ramp Enable
~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AO8/raw/RAMP{AO8_PAIR}
  Access:      rw
  Parameters:  bool


Channel Pair Zero Cross Control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AO8/raw/ZERO{AO8_PAIR}
  Access:      rw
  Parameters:  bool


Channel Soft Mute
~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AO8/raw/MUTE{AO8_PAIR}
  Access:      rw
  Parameters:  bool

Channel Relay Mute
~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AO8/raw/MUTE_RELAY{SLOT_CHANNEL}
  Access:      rw
  Parameters:  bool

Channel Invert
~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AO8/raw/INVERT{SLOT_CHANNEL}
  Access:      rw
  Parameters:  bool


Channel Volume
~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AO8/raw/VOLUME{SLOT_CHANNEL}
  Access:      rw
  Parameter:   ao8_volume


Channel Gain Relay
~~~~~~~~~~~~~~~~~~

::

  Address:     /slot/{SLOT}/AO8/raw/GAIN_RELAY{SLOT_CHANNEL}
  Access:      rw
  Parameter:   bool
