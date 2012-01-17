Media IO Settings
=================

``{SLOT}``:
  Denotes an I/O module submodule slot 1,2, or 3
``{PAIR}``:
  Denotes a media channel pair number within a slot: 1,2,3, or 4
``{SLOT_CHANNEL}``:
  Denotes a media channel on a slot: 1,2,3,4,5,6,7 or 8
``{CHANNEL}``:
  Denotes a channel number 1 to 24


Media Inputs
------------

Input Count
~~~~~~~~~~~

::

  Address:     /media/ins
  Access:      read
  Parameters:  int32_t


Input Channel Name
~~~~~~~~~~~~~~~~~~

::

  Address:     /media/in/{CHANNEL}/name
  Access:      r/w
  Parameters:  string

Input Channel Type
~~~~~~~~~~~~~~~~~~

::

  Address:     /media/in/{CHANNEL}/type
  Access:      read only
  Parameters:  string
  Description: Describes which type of channel: AES, AO8, ML8, CBR

Input Channel Slot
~~~~~~~~~~~~~~~~~~

::

  Address:     /media/in/{CHANNEL}/slot
  Access:      read only
  Parameters:  int32_t
  Description: Denotes which Submodule slot handles this channel

Input Channel Slot Channel
~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /media/in/{CHANNEL}/slot_channel
  Access:      read only
  Parameters:  int32_t SLOT_CHANNEL
  Description: Denotes which channel in the submodule maps to this channel


Media Outputs
-------------

Output Count
~~~~~~~~~~~~

::

  Address:     /media/outs
  Access:      read
  Parameters:  int32_t

Output Channel Type
~~~~~~~~~~~~~~~~~~~

::

  Address:     /media/out/{CHANNEL}/type
  Access:      read only
  Parameters:  string
  Description: Describes which type of channel: AES, AO8, ML8, CBR

Output Channel Slot
~~~~~~~~~~~~~~~~~~~

::

  Address:     /media/out/{CHANNEL}/slot
  Access:      read only
  Parameters:  int32_t
  Description: Denotes which Submodule slot handles this channel

Output Channel Slot Channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  Address:     /media/out/{CHANNEL}/slot_channel
  Access:      read only
  Parameters:  int32_t SLOT_CHANNEL
  Description: Denotes which channel in the submodule maps to this channel
